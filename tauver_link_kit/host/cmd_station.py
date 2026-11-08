#!/usr/bin/env python3
"""
TAUVER Link Kit · cmd_station.py · link-1.0.0
Runs on MACHINE B: the command station, or your laptop.

It talks to the Nucleo ONLY through ROS 2 topics. It never touches a cable, so it
works the same whether the bridge runs on the laptop next to you or on a headless
Jetson inside the rover.

    tauver> p        ping once (shows the round-trip time)
    tauver> t        ping test: 20 pings, then PASS / FAIL
    tauver> l        LED on / off   (look at the green LED on the Nucleo)
    tauver> d        lid demo: fake scale climbs, at 100 g the station closes the lid
    tauver> s        status of every link
    tauver> w        watch every raw frame on / off
    tauver> c CMD:HELP   send any command line
    tauver> h        help        q   quit

Automatic mode (exit code 0 = PASS, 1 = FAIL), handy for scripts and for tv test:
    python3 cmd_station.py --auto test
    python3 cmd_station.py --auto demo
"""

import argparse
import json
import os
import subprocess
import sys
import threading
import time

VERSION = "link-1.1.0"
TOPIC_TEL = "/tauver/science/telemetry"
TOPIC_EVENTS = "/tauver/science/events"
TOPIC_CMD = "/tauver/science/cmd"
TOPIC_STATUS = "/tauver/link/status"

STALE_S = 3.0           # no frame for this long = that link is down
LID_THRESHOLD_G = 100.0  # ERC sample rule: >= 100 g, then shut the lid

COLOR = sys.stdout.isatty() and os.environ.get("NO_COLOR") is None


def c(text, code):
    return "\033[%sm%s\033[0m" % (code, text) if COLOR else text


GOOD = lambda t: c(t, "32")    # green
BAD = lambda t: c(t, "31")     # red
WARN = lambda t: c(t, "33")    # yellow
DIM = lambda t: c(t, "2")
BOLD = lambda t: c(t, "1")

PRINT_LOCK = threading.Lock()


def say(*parts):
    with PRINT_LOCK:
        print(" ".join(str(p) for p in parts), flush=True)


def my_ips():
    try:
        out = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=3).stdout
        return [ip for ip in out.split() if "." in ip]
    except Exception:
        return []


def weight_line(d):
    """One readable line from a tel frame's data."""
    if "scale" not in d:
        return "weight: this firmware has no real scale (the pinger)"
    if d.get("scale") != "OK":
        return BAD("scale OFF") + "  (NAU7802 not answering: wiring / power)"
    g = d.get("rock_g")
    stable = GOOD("stable") if d.get("stable") else WARN("moving")
    if isinstance(g, (int, float)):
        return "weight %8.1f g   %s   raw %s" % (g, stable, d.get("raw"))
    need = "tare (z)" if not d.get("tared") else "calibrate (k <grams>)"
    return "raw %10s   %s   not in grams yet: %s" % (d.get("raw"), stable, need)


def result_line(o):
    k = o.get("kind")
    if k == "tare":
        nxt = "" if o.get("calibrated") else "  -> now put a known mass on and run: k <grams>"
        return "TARE  zero = %s counts (spread %s over %s samples)%s" % (o.get("offset"), o.get("spread"), o.get("n"), nxt)
    if k == "cal":
        return "CALIBRATED  %s g = %s counts -> %s counts per gram, saved in flash" % (
            o.get("grams"), o.get("delta"), o.get("cpg"))
    if k == "weigh":
        if o.get("calibrated"):
            return "WEIGH  %s g  (spread %s g, %s)" % (o.get("grams"), o.get("spread_g"),
                                                      "stable" if o.get("stable") else "NOT stable")
        return "WEIGH  raw %s (not calibrated yet)" % o.get("raw")
    return json.dumps(o)


HELP = """
  p          ping once (round-trip time)        t   ping test x20 -> PASS/FAIL
  l          LED on/off (watch LD2 on the board) d   lid demo (closed loop, 100 g)
  s          status of every link               w   watch raw frames on/off
  c <line>   send any command, e.g.  c CMD:HELP  ·  c CMD:LID=ROCK,OPEN
  h          this help                          q   quit
 SCALE (scale firmware):
  z          tare: scale EMPTY and still        k <grams>  calibrate, e.g. k 100
  g          live weight on/off (2 per second)  e          weigh once (averaged)
"""


def make_station():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import String

    best_effort = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
    reliable = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                          reliability=ReliabilityPolicy.RELIABLE)

    class Station(Node):
        def __init__(self, quiet=False):
            super().__init__("tauver_cmd_station")
            self.String = String
            self.quiet = quiet                   # auto mode: only the important lines
            self.lock = threading.Lock()
            self.pub = self.create_publisher(String, TOPIC_CMD, reliable)
            # Subscribing best-effort matches BOTH best-effort and reliable publishers.
            self.create_subscription(String, TOPIC_TEL, self.on_tel, best_effort)
            self.create_subscription(String, TOPIC_EVENTS, self.on_event, reliable)
            self.create_subscription(String, TOPIC_STATUS, self.on_status, best_effort)
            self.create_timer(0.5, self.watchdog)

            self.t_start = time.monotonic()
            self.last_tel_t = None
            self.last_tel = None
            self.tel_count = 0
            self.last_status_t = None
            self.last_status = None
            self.bridge_up = False
            self.nucleo_up = False
            self.hinted = False
            self.watch = False
            self.led_on = False
            self.live = False                    # print the live weight
            self.last_live = 0.0

            self.ping_id = int(time.time()) % 100000 * 10
            self.pending = {}                    # id -> send time
            self.rtts = []                       # filled while a test runs
            self.collect = False

            self.demo = None                     # dict while the lid demo runs
            self.demo_done = threading.Event()

        # ---------------- incoming ----------------
        def on_tel(self, msg):
            try:
                obj = json.loads(msg.data)
            except Exception:
                return
            now = time.monotonic()
            with self.lock:
                self.last_tel_t = now
                self.last_tel = obj
                self.tel_count += 1
            if self.watch:
                say(DIM("  tel  " + msg.data))
            if self.live and now - self.last_live >= 0.5:
                self.last_live = now
                say("  " + weight_line(obj.get("data") or {}))
            if self.demo is not None:
                self.demo_step(obj, now)

        def on_status(self, msg):
            try:
                obj = json.loads(msg.data)
            except Exception:
                return
            with self.lock:
                self.last_status_t = time.monotonic()
                self.last_status = obj

        def on_event(self, msg):
            now = time.monotonic()
            if self.watch:
                say(DIM("  evt  " + msg.data))
            try:
                obj = json.loads(msg.data)
            except Exception:
                return
            t = obj.get("type")
            if t == "pong":
                pid = obj.get("id")
                with self.lock:
                    if pid is None and self.pending:          # firmware without ids
                        pid = min(self.pending, key=self.pending.get)
                    sent = self.pending.pop(pid, None)
                if sent is None:
                    return
                rtt = (now - sent) * 1000.0
                if self.collect:
                    self.rtts.append(rtt)
                else:
                    say(GOOD("  PONG") + "  #%s   round trip %.1f ms" % (pid, rtt))
            elif t == "ack":
                cmd = str(obj.get("cmd", ""))
                if cmd.upper().startswith("CMD:PING") and obj.get("ok"):
                    return                                      # the pong already said it
                if self.collect:
                    return
                tag = GOOD("  ACK ") if obj.get("ok") else BAD("  NACK")
                if not self.quiet or not obj.get("ok"):
                    say(tag, " %s  ->  %s" % (cmd, obj.get("msg", "")))
            elif t == "event":
                code = obj.get("code", "?")
                if self.demo is not None and code == "LID_CLOSED":
                    self.demo_closed(now, obj.get("ms"))
                elif not self.quiet:
                    extra = obj.get("detail") or ("rock_g %s" % obj.get("rock_g") if "rock_g" in obj else "")
                    line = "  EVENT  %s  %s" % (code, extra)
                    say(BAD(line) if obj.get("lvl") == "warn" else c(line, "36"))
            elif t == "result":
                say(GOOD("  RESULT") + "  " + result_line(obj))
            elif t == "bridge":
                code = obj.get("code", "?")
                line = "  BRIDGE  %s  %s" % (code, {k: v for k, v in obj.items()
                                                    if k not in ("type", "code")})
                say(BAD(line) if code in ("CMD_DROPPED", "SERIAL_DOWN", "BAD_LINE",
                                          "CMD_REJECTED") else c(line, "36"))
            elif t in ("boot", "health", "help"):
                if not self.quiet:
                    say(c("  " + t.upper(), "36"), " " + msg.data)
            elif not self.quiet:
                say("  <<   " + msg.data)

        # ---------------- link watchdog ----------------
        def watchdog(self):
            now = time.monotonic()
            with self.lock:
                st_age = None if self.last_status_t is None else now - self.last_status_t
                tel_age = None if self.last_tel_t is None else now - self.last_tel_t
                status = self.last_status or {}
            bridge = st_age is not None and st_age < STALE_S
            nucleo = tel_age is not None and tel_age < STALE_S
            if bridge != self.bridge_up:
                self.bridge_up = bridge
                if bridge:
                    say(GOOD("✔ BRIDGE FOUND") + "  on %s %s  (serial: %s)" % (
                        status.get("host", "?"), ",".join(status.get("ips", [])),
                        status.get("serial", "?")))
                else:
                    say(BAD("✘ BRIDGE LOST") + "  no status for %d s -> Jetson off? network? "
                        "bridge stopped? (on the Jetson: tv logs)" % STALE_S)
            if nucleo != self.nucleo_up:
                self.nucleo_up = nucleo
                if nucleo:
                    say(GOOD("✔ NUCLEO ALIVE") + "  heartbeat #%s arriving through ROS 2"
                        % (self.last_tel or {}).get("seq", "?"))
                else:
                    why = "USB unplugged or board not running"
                    if bridge and status.get("serial") != "UP":
                        why = "the bridge can't open the board: %s" % status.get("detail", "")
                    say(BAD("✘ NUCLEO SILENT") + "  no heartbeat for %d s -> %s" % (STALE_S, why))
            if not self.hinted and not bridge and not nucleo and now - self.t_start > 6:
                self.hinted = True
                say(WARN("! Nothing heard yet after 6 s.") + " Is the bridge running? Same "
                    "ROS_DOMAIN_ID (%s) on both machines? Run: tv topics"
                    % os.environ.get("ROS_DOMAIN_ID", "0"))

        # ---------------- outgoing ----------------
        def send(self, line):
            m = self.String()
            m.data = line
            self.pub.publish(m)

        def ping(self):
            with self.lock:
                self.ping_id += 1
                pid = self.ping_id
                self.pending[pid] = time.monotonic()
            self.send("CMD:PING=%d" % pid)
            return pid

        def ping_test(self, n=20, gap=0.25):
            say(BOLD("PING TEST") + "  %d pings, %.2f s apart ..." % (n, gap))
            self.rtts = []
            self.collect = True
            ids = []
            for _ in range(n):
                ids.append(self.ping())
                time.sleep(gap)
            t_end = time.monotonic() + 2.0
            while time.monotonic() < t_end and any(i in self.pending for i in ids):
                time.sleep(0.05)
            self.collect = False
            with self.lock:
                for i in ids:
                    self.pending.pop(i, None)
            got = len(self.rtts)
            lost = n - got
            if got:
                r = sorted(self.rtts)
                line = "sent %d · back %d · lost %d (%.0f%%) · rtt min %.1f / avg %.1f / max %.1f ms" % (
                    n, got, lost, 100.0 * lost / n, r[0], sum(r) / got, r[-1])
            else:
                line = "sent %d · back 0 · lost %d (100%%)" % (n, n)
            ok = lost == 0
            say((GOOD("PASS  ") if ok else BAD("FAIL  ")) + line)
            if not ok and got == 0:
                say(WARN("  -> nothing came back. Run s (status) to see which link is down."))
            elif not ok:
                say(WARN("  -> some pings lost: weak Wi-Fi? Try a cable, then run t again."))
            return ok

        # ---------------- the scale ----------------
        def scale_data(self):
            with self.lock:
                return dict((self.last_tel or {}).get("data") or {})

        def tare(self):
            d = self.scale_data()
            if d.get("scale") == "OFF":
                say(BAD("  The scale is OFF (NAU7802 not found). Check the 4 I2C wires, then c CMD:SCALE_RESET"))
                return
            self.send("CMD:TARE")
            say("  TARE sent: keep the scale EMPTY and still for half a second ...")

        def calibrate(self, arg):
            try:
                grams = float(arg)
                if not grams > 0:
                    raise ValueError
            except ValueError:
                say("  usage: k <grams of the known mass>, e.g.  k 100")
                return
            d = self.scale_data()
            if d.get("tared") == 0:
                say(WARN("  Tare first (z) with the scale EMPTY, then put the %g g mass on and run k again." % grams))
                return
            self.send("CMD:CALW=%g" % grams)
            say("  CALIBRATE sent: %g g on the scale, hands off, 1 second ..." % grams)

        def toggle_live(self):
            self.live = not self.live
            if self.live:
                say("  live weight ON (g again to stop)")
                self.last_live = 0.0
            else:
                say("  live weight OFF")

        def toggle_led(self):
            self.led_on = not self.led_on
            self.send("CMD:LED=" + ("ON" if self.led_on else "OFF"))
            say("  LED " + ("ON" if self.led_on else "OFF") +
                " sent -> look at the green LED (LD2) on the Nucleo. "
                "(c CMD:LED=AUTO gives the heartbeat blink back)")

        # ---------------- the closed-loop demo ----------------
        def run_demo(self, timeout=40.0):
            if not self.nucleo_up:
                say(BAD("  The Nucleo isn't sending heartbeats, so the demo can't run. "
                        "Fix the link first (s for status)."))
                return False
            data = self.scale_data()
            real = "scale" in data                     # scale firmware: the weight is real
            if real and (data.get("scale") != "OK" or not data.get("cal") or not data.get("tared")):
                say(BAD("  The real scale isn't ready: scale %s, tared %s, calibrated %s. "
                        "Do z (empty) and k <grams> first." % (data.get("scale"), data.get("tared"), data.get("cal"))))
                return False
            if real:
                timeout = max(timeout, 60.0)
                say(BOLD("LID DEMO, REAL WEIGHT") + "  put %.0f g or more on the scale within %.0f s -> this "
                    "station sees it -> sends CMD:LID=ROCK,CLOSE -> the board confirms" % (LID_THRESHOLD_G, timeout))
            else:
                say(BOLD("LID DEMO") + "  fake scale climbs -> this station watches -> at %.0f g it "
                    "sends CMD:LID=ROCK,CLOSE -> the board confirms" % LID_THRESHOLD_G)
            self.demo_done.clear()
            self.demo = {"phase": "fill", "t_close_sent": None, "t_seen": None,
                         "t_done": None, "grams": None, "last_print": -1,
                         "board_ms_frame": None, "board_ms_closed": None}
            self.send("CMD:LID=ROCK,OPEN")
            time.sleep(0.3)
            self.send("CMD:SIM=ON")
            ok = self.demo_done.wait(timeout)
            d, self.demo = self.demo, None
            if not ok or d is None or d["t_done"] is None:
                say(BAD("FAIL  ") + "no LID_CLOSED confirmation within %.0f s" % timeout)
                self.send("CMD:SIM=OFF")
                return False
            total = (d["t_done"] - d["t_seen"]) * 1000.0
            loop = ""
            if isinstance(d["board_ms_frame"], (int, float)) and isinstance(d["board_ms_closed"], (int, float)):
                loop = " · on the board's own clock: %d ms from sending the %.0f g frame to shutting the lid" % (
                    d["board_ms_closed"] - d["board_ms_frame"], d["grams"])
            say(GOOD("PASS  ") + "lid closed at %.1f g%s · seen here -> confirmed here %.1f ms"
                % (d["grams"], loop, total))
            say("  The LED on the Nucleo is now SOLID = lid shut. Reset it with:  "
                "c CMD:LID=ROCK,OPEN")
            return True

        def demo_step(self, tel, now):
            d = self.demo
            if d is None or d["phase"] != "fill":
                return
            g = (tel.get("data") or {}).get("rock_g")
            if not isinstance(g, (int, float)):
                return
            bars = max(0, int(min(g, LID_THRESHOLD_G) / LID_THRESHOLD_G * 20))
            if int(g) // 8 != d["last_print"] or g >= LID_THRESHOLD_G:
                d["last_print"] = int(g) // 8
                say("  rock_g %6.1f g  [%s%s]" % (g, "#" * bars, "." * (20 - bars)))
            data = tel.get("data") or {}
            if g >= LID_THRESHOLD_G and data.get("stable", 1) != 1:
                return                                  # real scale: wait until it settles
            if g >= LID_THRESHOLD_G:
                d["phase"] = "closing"
                d["t_seen"] = now
                d["grams"] = g
                d["board_ms_frame"] = tel.get("ms")
                self.send("CMD:LID=ROCK,CLOSE")
                d["t_close_sent"] = time.monotonic()
                say(WARN("  >= %.0f g -> CMD:LID=ROCK,CLOSE sent" % LID_THRESHOLD_G))

        def demo_closed(self, now, board_ms):
            d = self.demo
            if d is not None and d["phase"] == "closing":
                d["phase"] = "done"
                d["t_done"] = now
                d["board_ms_closed"] = board_ms
                self.demo_done.set()

        # ---------------- status ----------------
        def print_status(self):
            now = time.monotonic()
            with self.lock:
                st, st_t, tel, tel_t = (self.last_status, self.last_status_t,
                                        self.last_tel, self.last_tel_t)
            say(BOLD("STATUS") + "   this machine: %s · ROS_DOMAIN_ID=%s" % (
                ", ".join(my_ips()) or "no IP", os.environ.get("ROS_DOMAIN_ID", "0 (not set!)")))
            if st_t is None:
                say("  link 3  network/ROS 2  : " + BAD("✘ no bridge status ever received") +
                    "  -> phase 3 checks: ping the other machine, tv topics")
            else:
                age = now - st_t
                mark = GOOD("✔") if age < STALE_S else BAD("✘")
                say("  link 3  network/ROS 2  : %s bridge on %s %s, last status %.1f s ago" % (
                    mark, st.get("host"), ",".join(st.get("ips", [])), age))
                ser = st.get("serial")
                mark = GOOD("✔") if ser == "UP" else BAD("✘")
                say("  link 2  bridge<->serial: %s %s %s" % (mark, ser, st.get("detail", "")))
                say("          rx %s · bad %s · cmds %s · dropped %s · bridge up %s s" % (
                    st.get("rx_lines"), st.get("bad_lines"), st.get("tx_cmds"),
                    st.get("dropped_cmds"), st.get("uptime_s")))
            if tel_t is None:
                say("  link 1  board alive    : " + BAD("✘ no heartbeat ever received"))
            else:
                age = now - tel_t
                mark = GOOD("✔") if age < STALE_S else BAD("✘")
                say("  link 1  board alive    : %s heartbeat #%s, %.1f s ago · %s" % (
                    mark, tel.get("seq"), age, json.dumps(tel.get("data", {}))))

    return rclpy, Station


def wait_for_link(station, seconds):
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end and not station.nucleo_up:
        time.sleep(0.1)
    return station.nucleo_up


def main():
    ap = argparse.ArgumentParser(description="TAUVER command station (link kit)")
    ap.add_argument("--auto", choices=["test", "demo", "ping"],
                    help="run one check and exit (0 = PASS, 1 = FAIL)")
    ap.add_argument("--wait", type=float, default=15.0,
                    help="auto mode: seconds to wait for the link (default 15)")
    args = ap.parse_args()

    try:
        rclpy, Station = make_station()
    except ImportError as e:
        if os.environ.get("ROS_DISTRO"):
            print("ERROR: ROS 2 %s is loaded, but this Python (%s) can't import it: %s\n"
                  "fix: run it with the system Python: /usr/bin/python3 (conda/pyenv/venv active?)"
                  % (os.environ["ROS_DISTRO"], sys.executable, e))
        else:
            print("ERROR: ROS 2 is not loaded in this terminal.\n"
                  "fix: open a new terminal (setup.sh adds the kit to ~/.bashrc) or use: tv cmd")
        return 2

    rclpy.init()
    st = Station(quiet=bool(args.auto))
    from rclpy.executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()
    ex.add_node(st)
    def spin_quietly():
        try:
            ex.spin()
        except Exception:
            pass                         # normal when we shut ROS down on exit

    spin = threading.Thread(target=spin_quietly, daemon=True)
    spin.start()

    say(BOLD("TAUVER command station %s" % VERSION) + "  ·  this machine %s  ·  ROS_DOMAIN_ID=%s"
        % (", ".join(my_ips()) or "no IP", os.environ.get("ROS_DOMAIN_ID", "0 (not set!)")))

    code = 0
    try:
        if args.auto:
            say("waiting up to %.0f s for the Nucleo's heartbeat ..." % args.wait)
            if not wait_for_link(st, args.wait):
                say(BAD("FAIL  ") + "no heartbeat from the Nucleo within %.0f s" % args.wait)
                st.print_status()
                code = 1
            elif args.auto == "test":
                code = 0 if st.ping_test() else 1
            elif args.auto == "demo":
                code = 0 if st.run_demo() else 1
            else:
                st.collect = True
                st.rtts = []
                st.ping()
                t_end = time.monotonic() + 3
                while time.monotonic() < t_end and not st.rtts:
                    time.sleep(0.05)
                st.collect = False
                if st.rtts:
                    say(GOOD("PASS  ") + "pong in %.1f ms" % st.rtts[0])
                else:
                    say(BAD("FAIL  ") + "no pong within 3 s")
                    code = 1
        else:
            say("waiting for the bridge and the Nucleo ...  (h = help)")
            while True:
                try:
                    line = input("tauver> ").strip()
                except EOFError:
                    break
                if not line:
                    continue
                cmd, _, rest = line.partition(" ")
                cmd = cmd.lower()
                if cmd in ("q", "quit", "exit"):
                    break
                elif cmd in ("h", "help", "?"):
                    say(HELP)
                elif cmd == "p":
                    pid = st.ping()
                    say("  PING #%d sent" % pid)
                elif cmd == "t":
                    st.ping_test()
                elif cmd == "z":
                    st.tare()
                elif cmd == "k":
                    st.calibrate(rest.strip())
                elif cmd == "g":
                    st.toggle_live()
                elif cmd == "e":
                    st.send("CMD:WEIGH")
                    say("  WEIGH sent: hands off for half a second ...")
                elif cmd == "l":
                    st.toggle_led()
                elif cmd == "d":
                    st.run_demo()
                elif cmd == "s":
                    st.print_status()
                elif cmd == "w":
                    st.watch = not st.watch
                    say("  watch " + ("ON: every frame is printed (w again to stop)" if st.watch else "OFF"))
                elif cmd == "c":
                    if rest.strip():
                        st.send(rest.strip())
                        say("  sent: " + rest.strip())
                    else:
                        say("  usage: c CMD:HELP")
                elif line.upper().startswith(("CMD:", "SET:", "STATE:")):
                    st.send(line)
                    say("  sent: " + line)
                else:
                    say("  ? unknown. h = help")
    except KeyboardInterrupt:
        pass
    finally:
        # Order matters: stop ROS first so the spin thread returns, then clean up.
        try:
            rclpy.shutdown()
        except Exception:
            pass
        spin.join(timeout=2.0)
        try:
            ex.shutdown(timeout_sec=1.0)
            st.destroy_node()
        except Exception:
            pass
    return code


if __name__ == "__main__":
    sys.exit(main())
