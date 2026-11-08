#!/usr/bin/env python3
"""
TAUVER Link Kit · link_bridge.py · link-1.0.0
Runs on MACHINE A: the Jetson (or, for rehearsals, the laptop playing the Jetson).

    Nucleo --USB serial, JSON lines-->  [ this bridge ]  --ROS 2 topics-->  anyone on the network
    Nucleo <--command lines-----------  [ this bridge ]  <--ROS 2 topic---  command station

The bridge is a dumb pipe on purpose. It does not understand the payload; it moves
lines. That is why the same bridge keeps working when the real firmware replaces
the pinger.

TOPICS (the same names the real Science Pack bridge uses)
  /tauver/science/telemetry   std_msgs/String  OUT  every "tel" frame          best-effort
  /tauver/science/events      std_msgs/String  OUT  every other frame          reliable
                                                     (boot, ack, pong, event, health, help,
                                                      plus the bridge's own "bridge" notes)
  /tauver/science/cmd         std_msgs/String  IN   one command line -> the Nucleo
  /tauver/link/status         std_msgs/String  OUT  the bridge's own health, 1 Hz (JSON)
                                                     -> lets you debug a headless Jetson from afar

RUN IT
  python3 link_bridge.py                     real Nucleo, port found automatically
  python3 link_bridge.py --port /dev/ttyACM0
  python3 link_bridge.py --fake              no board: a simulated Nucleo inside the bridge
  python3 link_bridge.py --no-ros            no ROS at all: frames to the screen, type commands
  (or just: tv bridge · tv fake · tv serial)

As a service (tv service install) it starts at boot, waits for the network, reconnects
the USB by itself, and restarts itself when the machine's IP addresses change, so DDS
always listens on the network that is really there.
"""

import argparse
import glob
import json
import os
import select
import signal
import socket
import subprocess
import sys
import time

VERSION = "link-1.0.0"
TOPIC_TEL = "/tauver/science/telemetry"
TOPIC_EVENTS = "/tauver/science/events"
TOPIC_CMD = "/tauver/science/cmd"
TOPIC_STATUS = "/tauver/link/status"

CMD_MAX = 200          # longest command line we forward
RECONNECT_S = 1.0      # how often to retry the serial port
IP_CHECK_S = 5.0       # how often to look at the machine's IP addresses


def log(msg):
    print(time.strftime("%H:%M:%S") + "  " + msg, flush=True)


# ---------------------------------------------------------------------------
# Network helpers
# ---------------------------------------------------------------------------
def local_ips():
    """Non-loopback IPv4 addresses of this machine, as 'iface=ip' strings, sorted."""
    try:
        out = subprocess.run(["ip", "-4", "-o", "addr", "show"], capture_output=True,
                             text=True, timeout=3).stdout
        ips = []
        for line in out.splitlines():
            parts = line.split()
            if len(parts) >= 4 and parts[1] != "lo":
                ips.append(parts[1] + "=" + parts[3].split("/")[0])
        return sorted(ips)
    except Exception:
        try:
            out = subprocess.run(["hostname", "-I"], capture_output=True, text=True,
                                 timeout=3).stdout
            return sorted(ip for ip in out.split() if "." in ip and not ip.startswith("127."))
        except Exception:
            return []


def wait_for_network(max_s):
    """Wait until this machine has at least one real IPv4 address (max_s seconds)."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < max_s:
        ips = local_ips()
        if ips:
            log("network up: " + ", ".join(ips))
            return ips
        time.sleep(1.0)
    log("WARNING: no network after %d s, starting anyway (DDS will only see this machine)" % max_s)
    return local_ips()


# ---------------------------------------------------------------------------
# Serial port discovery
# ---------------------------------------------------------------------------
def find_port():
    """Best guess for the Nucleo's port. The ST-LINK shows up under /dev/serial/by-id
    with a name containing STMicroelectronics / STLINK, which never changes between
    reboots (unlike ttyACM0 / ttyACM1)."""
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    for key in ("STMicro", "STLINK", "STLink", "STM32"):
        for p in by_id:
            if key.lower() in p.lower():
                return p
    for p in by_id:
        if "arduino" in p.lower():
            return p
    acm = sorted(glob.glob("/dev/ttyACM*"))
    if acm:
        return acm[0]
    usb = sorted(glob.glob("/dev/ttyUSB*"))
    return usb[0] if usb else None


class SerialLink:
    """The USB cable. Opens, reads whole lines, writes lines, reconnects by itself."""

    def __init__(self, port, baud):
        self.want_port = port          # None = find it automatically
        self.baud = baud
        self.ser = None
        self.port = None
        self.buf = b""
        self.last_try = 0.0
        self.state = "SEARCHING"       # SEARCHING | UP | NO_PORT | BUSY | DENIED
        self.detail = ""
        self.just_connected = False
        self.just_lost = False

    @property
    def up(self):
        return self.ser is not None

    def _try_open(self):
        import serial  # pyserial
        port = self.want_port or find_port()
        if not port:
            self._set_state("NO_PORT", "no /dev/ttyACM* or /dev/serial/by-id device")
            return
        try:
            # exclusive=True: a second bridge on the same port fails loudly instead of
            # silently stealing half the bytes.
            self.ser = serial.Serial(port, self.baud, timeout=0, write_timeout=0.5,
                                     exclusive=True)
            self.port = port
            self.buf = b""
            self._set_state("UP", port)
            self.just_connected = True
        except PermissionError:
            self._set_state("DENIED", port + ": permission denied (are you in the dialout group?)")
        except Exception as e:
            msg = str(e)
            if "lock" in msg.lower() or "busy" in msg.lower() or "Resource temporarily" in msg:
                self._set_state("BUSY", port + ": in use by another program (Arduino IDE? another bridge?)")
            elif "Permission" in msg:
                self._set_state("DENIED", port + ": permission denied (are you in the dialout group?)")
            else:
                self._set_state("SEARCHING", port + ": " + msg)

    def _set_state(self, state, detail):
        if state != self.state or detail != self.detail:
            self.state, self.detail = state, detail
            log("serial: %s  %s" % (state, detail))

    def close(self, why=""):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.just_lost = True
            self._set_state("SEARCHING", "lost " + str(self.port) + (" (" + why + ")" if why else ""))

    def read_lines(self):
        """Whole lines received since last call (decoded, stripped). Never blocks."""
        now = time.monotonic()
        if self.ser is None:
            if now - self.last_try >= RECONNECT_S:
                self.last_try = now
                self._try_open()
            return []
        try:
            n = self.ser.in_waiting
            data = self.ser.read(n if n else 1)
        except Exception as e:
            self.close(str(e).split(":")[0])
            return []
        if not data:
            return []
        self.buf += data
        if len(self.buf) > 8192 and b"\n" not in self.buf:
            self.buf = b""             # a runaway line with no newline: drop it
        lines = []
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            lines.append(raw.decode("utf-8", errors="replace").strip())
        return [l for l in lines if l]

    def write_line(self, text):
        if self.ser is None:
            return False
        try:
            self.ser.write((text + "\n").encode("utf-8", errors="replace"))
            return True
        except Exception as e:
            self.close(str(e).split(":")[0])
            return False


class FakeNucleo:
    """A simulated Nucleo that speaks exactly what tauver_link_fw.ino speaks.
    Lets you test the whole network chain with no board plugged in."""

    TEL_MS_IDLE, TEL_MS_SIM, SIM_G, SIM_MAX = 1.0, 0.25, 4.0, 400.0

    def __init__(self):
        self.t0 = time.monotonic()
        self.out = []
        self.seq = 0
        self.rx = 0
        self.bad = 0
        self.led = "AUTO"
        self.sim = False
        self.lid_closed = False
        self.rock = 0.0
        self.last_tel = time.monotonic()
        self.state, self.port, self.detail = "UP", "fake", "simulated Nucleo"
        self.just_connected, self.just_lost = True, False
        self._emit({"type": "boot", "v": 3, "kit": "link", "fw": VERSION + "-fake",
                    "board": "FAKE", "baud": 115200})

    up = property(lambda self: True)

    def ms(self):
        return int((time.monotonic() - self.t0) * 1000)

    def _emit(self, obj):
        self.out.append(json.dumps(obj, separators=(",", ":")))

    def _ack(self, cmd, ok, msg):
        self._emit({"type": "ack", "cmd": cmd, "ok": 1 if ok else 0, "msg": msg})

    def _event(self, lvl, code):
        self._emit({"type": "event", "ms": self.ms(), "lvl": lvl, "code": code,
                    "rock_g": round(self.rock, 1)})

    def _lid(self):
        return "CLOSED" if self.lid_closed else "OPEN"

    def write_line(self, text):
        s = text.strip()
        up = s.upper()
        if not s:
            return True
        if up.startswith("CMD:PING"):
            rest = up[8:]
            if rest.startswith("="):
                arg = rest[1:]
                if not arg.lstrip("-").isdigit():
                    self.bad += 1
                    self._ack(s, False, "BAD_ID")
                    return True
                pid = int(arg)
            elif rest == "":
                pid = None
            else:
                self.bad += 1
                self._ack(s, False, "UNKNOWN_CMD")
                return True
            self.rx += 1
            self._emit({"type": "pong", "id": pid, "ms": self.ms()})
            self._ack(s, True, "PONG")
        elif up.startswith("CMD:LED="):
            a = up[8:]
            if a in ("ON", "OFF", "AUTO"):
                self.led = a
                self.rx += 1
                self._ack(s, True, a)
            else:
                self.bad += 1
                self._ack(s, False, "USE_ON_OFF_AUTO")
        elif up.startswith("CMD:SIM="):
            a = up[8:]
            if a == "ON":
                self.rx += 1
                if self.lid_closed:
                    self._ack(s, False, "LID_CLOSED_OPEN_IT_FIRST")
                else:
                    self.sim, self.rock = True, 0.0
                    self._ack(s, True, "SIM_ON")
                    self._event("info", "SIM_START")
            elif a == "OFF":
                self.sim = False
                self.rx += 1
                self._ack(s, True, "SIM_OFF")
                self._event("info", "SIM_STOP")
            else:
                self.bad += 1
                self._ack(s, False, "USE_ON_OFF")
        elif up.startswith("CMD:LID="):
            a = up[8:]
            if not a.startswith("ROCK,"):
                self.bad += 1
                self._ack(s, False, "UNKNOWN_BAY")
            elif a[5:] == "CLOSE":
                self.lid_closed, self.sim = True, False
                self.rx += 1
                self._ack(s, True, "LID_CLOSED")
                self._event("info", "LID_CLOSED")
            elif a[5:] == "OPEN":
                self.lid_closed, self.rock = False, 0.0
                self.rx += 1
                self._ack(s, True, "LID_OPEN")
                self._event("info", "LID_OPENED")
            else:
                self.bad += 1
                self._ack(s, False, "USE_CLOSE_OPEN")
        elif up == "CMD:STATUS":
            self.rx += 1
            self._emit({"type": "health", "v": 3, "kit": "link", "fw": VERSION + "-fake",
                        "up_s": self.ms() // 1000, "rx_lines": self.rx, "bad_lines": self.bad,
                        "led": self.led, "sim": int(self.sim), "lid_rock": self._lid()})
            self._ack(s, True, "OK")
        elif up == "CMD:HELP":
            self.rx += 1
            self._emit({"type": "help", "cmds": ["CMD:PING[=id]", "CMD:LED=ON|OFF|AUTO",
                        "CMD:SIM=ON|OFF", "CMD:LID=ROCK,CLOSE|OPEN", "CMD:STATUS",
                        "CMD:HELP", "CMD:RESET"]})
            self._ack(s, True, "OK")
        elif up == "CMD:RESET":
            self.rx += 1
            self.led, self.sim, self.lid_closed, self.rock = "AUTO", False, False, 0.0
            self._ack(s, True, "RESET")
            self._event("info", "DEMO_RESET")
        else:
            self.bad += 1
            self._ack(s, False, "UNKNOWN_CMD")
        return True

    def read_lines(self):
        now = time.monotonic()
        period = self.TEL_MS_SIM if self.sim else self.TEL_MS_IDLE
        if now - self.last_tel >= period:
            self.last_tel = now
            if self.sim:
                self.rock += self.SIM_G
                if self.rock >= self.SIM_MAX:
                    self.rock, self.sim = self.SIM_MAX, False
                    self._event("warn", "SIM_MAX_NOBODY_CLOSED_LID")
            self._emit({"type": "tel", "seq": self.seq, "ms": self.ms(),
                        "data": {"rock_g": round(self.rock, 1), "lid_rock": self._lid(),
                                 "sim": int(self.sim), "led": self.led}})
            self.seq += 1
        out, self.out = self.out, []
        return out

    def close(self, why=""):
        pass


# ---------------------------------------------------------------------------
# The pipe itself (no ROS in here, so it can be tested on its own)
# ---------------------------------------------------------------------------
class BridgeCore:
    """Classifies lines from the board and forwards commands to it.
    publish(kind, text) is called with kind in {"tel", "event"}."""

    def __init__(self, link, publish):
        self.link = link
        self.publish = publish
        self.rx_lines = 0
        self.bad_lines = 0
        self.bad_since_note = 0
        self.last_bad_note = 0.0
        self.tx_cmds = 0
        self.dropped = 0
        self.last_rx = None
        self.first_frame_logged = False
        self.t0 = time.monotonic()

    def note(self, code, **extra):
        obj = {"type": "bridge", "code": code}
        obj.update(extra)
        self.publish("event", json.dumps(obj, separators=(",", ":")))

    def poll(self):
        for line in self.link.read_lines():
            self.on_line(line)
        if self.link.just_connected:
            self.link.just_connected = False
            self.note("SERIAL_UP", port=self.link.port)
            # The board does not reset when the port opens, so its boot frame is
            # long gone. Ask for its state instead.
            self.link.write_line("CMD:STATUS")
        if self.link.just_lost:
            self.link.just_lost = False
            self.first_frame_logged = False
            self.note("SERIAL_DOWN", why=self.link.detail)

    def on_line(self, line):
        try:
            obj = json.loads(line)
            if not isinstance(obj, dict) or "type" not in obj:
                raise ValueError("no type")
        except Exception:
            self.bad_lines += 1
            self.bad_since_note += 1
            now = time.monotonic()
            if now - self.last_bad_note >= 1.0:   # at most one note per second
                self.last_bad_note = now
                self.note("BAD_LINE", count=self.bad_since_note, raw=line[:120])
                log("not JSON (%d in the last second): %r" % (self.bad_since_note, line[:80]))
                self.bad_since_note = 0
            return
        self.rx_lines += 1
        self.last_rx = time.monotonic()
        if not self.first_frame_logged:
            self.first_frame_logged = True
            log("first frame from the board: " + line[:100])
        self.publish("tel" if obj.get("type") == "tel" else "event", line)

    def on_cmd(self, text):
        text = (text or "").strip()
        if not text:
            return
        if len(text) > CMD_MAX or "\n" in text or "\r" in text:
            self.dropped += 1
            self.note("CMD_REJECTED", cmd=text[:60], why="one line, max %d chars" % CMD_MAX)
            return
        if self.link.write_line(text):
            self.tx_cmds += 1
            log("cmd -> board: " + text)
        else:
            self.dropped += 1
            self.note("CMD_DROPPED", cmd=text, why="board not connected (" + self.link.state + ")")
            log("cmd DROPPED (board not connected): " + text)

    def status(self, ips, fake):
        age = None if self.last_rx is None else round(time.monotonic() - self.last_rx, 2)
        return {
            "type": "link_status", "bridge": VERSION, "host": socket.gethostname(),
            "ips": ips, "mode": "fake" if fake else "serial",
            "serial": self.link.state, "port": self.link.port, "detail": self.link.detail,
            "rx_lines": self.rx_lines, "bad_lines": self.bad_lines,
            "tx_cmds": self.tx_cmds, "dropped_cmds": self.dropped,
            "last_rx_age_s": age, "uptime_s": int(time.monotonic() - self.t0),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
        }


# ---------------------------------------------------------------------------
# Mode 1: no ROS — a plain serial monitor
# ---------------------------------------------------------------------------
def run_no_ros(link):
    log("NO-ROS mode: frames below, type a command + Enter (Ctrl+C to quit)")
    log("try:  CMD:PING=1   CMD:LED=ON   CMD:HELP")

    def show(kind, text):
        print(("  TEL  " if kind == "tel" else "  <<   ") + text, flush=True)

    core = BridgeCore(link, show)
    try:
        while True:
            core.poll()
            r, _, _ = select.select([sys.stdin], [], [], 0.02)
            if r:
                line = sys.stdin.readline()
                if line == "":
                    break
                if line.strip():
                    core.on_cmd(line)
    except KeyboardInterrupt:
        pass
    link.close()
    return 0


# ---------------------------------------------------------------------------
# Mode 2: ROS 2 node
# ---------------------------------------------------------------------------
def run_ros(link, fake):
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from std_msgs.msg import String
    except ImportError as e:
        if os.environ.get("ROS_DISTRO"):
            log("ERROR: ROS 2 %s is loaded, but this Python (%s) can't import it: %s"
                % (os.environ["ROS_DISTRO"], sys.executable, e))
            log("fix: run it with the system Python: /usr/bin/python3 (conda/pyenv/venv active?)")
        else:
            log("ERROR: ROS 2 is not loaded in this terminal.")
            log("fix: open a new terminal (setup.sh adds the kit to ~/.bashrc) or: tv bridge")
        return 2

    best_effort = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
    reliable = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                          reliability=ReliabilityPolicy.RELIABLE)

    class BridgeNode(Node):
        def __init__(self):
            super().__init__("tauver_link_bridge")
            self.pub_tel = self.create_publisher(String, TOPIC_TEL, best_effort)
            self.pub_ev = self.create_publisher(String, TOPIC_EVENTS, reliable)
            self.pub_status = self.create_publisher(String, TOPIC_STATUS, best_effort)
            self.create_subscription(String, TOPIC_CMD, self.on_cmd_msg, reliable)
            self.core = BridgeCore(link, self.publish)
            self.ips = local_ips()
            self.last_ip_check = time.monotonic()
            self.exit_code = None
            self.create_timer(0.01, self.core.poll)          # 100 Hz serial poll
            self.create_timer(1.0, self.tick_status)
            self.last_summary = time.monotonic()

        def publish(self, kind, text):
            m = String()
            m.data = text
            (self.pub_tel if kind == "tel" else self.pub_ev).publish(m)

        def on_cmd_msg(self, msg):
            self.core.on_cmd(msg.data)

        def tick_status(self):
            now = time.monotonic()
            if now - self.last_ip_check >= IP_CHECK_S:
                self.last_ip_check = now
                ips = local_ips()
                if ips != self.ips:
                    log("network changed: %s -> %s" % (self.ips or "none", ips or "none"))
                    self.ips = ips
                    if os.environ.get("INVOCATION_ID"):
                        # Running as a systemd service: restart so DDS binds to the
                        # interfaces that exist now (systemd brings us back in 3 s).
                        log("restarting so ROS 2 uses the new network")
                        self.exit_code = 3
                    else:
                        log("tip: stop me (Ctrl+C) and start me again so ROS 2 uses the new network")
            m = String()
            m.data = json.dumps(self.core.status(self.ips, fake), separators=(",", ":"))
            self.pub_status.publish(m)
            if now - self.last_summary >= 30.0:
                self.last_summary = now
                s = self.core.status(self.ips, fake)
                log("alive · serial %s · rx %d · bad %d · cmds %d · dropped %d"
                    % (s["serial"], s["rx_lines"], s["bad_lines"], s["tx_cmds"], s["dropped_cmds"]))

    rclpy.init()
    node = BridgeNode()
    stop = {"flag": False}

    def on_signal(signum, frame):
        stop["flag"] = True

    signal.signal(signal.SIGTERM, on_signal)
    log("ROS 2 node 'tauver_link_bridge' up · ROS_DOMAIN_ID=%s · %s"
        % (os.environ.get("ROS_DOMAIN_ID", "0 (not set!)"), ", ".join(node.ips) or "no network"))
    log("publishing %s, %s, %s · listening on %s" % (TOPIC_TEL, TOPIC_EVENTS, TOPIC_STATUS, TOPIC_CMD))
    code = 0
    try:
        while rclpy.ok() and not stop["flag"] and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        code = node.exit_code or 0
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    log("bridge stopped")
    return code


def main():
    ap = argparse.ArgumentParser(description="TAUVER link bridge: Nucleo serial <-> ROS 2")
    ap.add_argument("--port", help="serial port (default: find the Nucleo automatically)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--fake", action="store_true", help="simulated Nucleo, no hardware")
    ap.add_argument("--no-ros", action="store_true", help="plain serial monitor, no ROS")
    ap.add_argument("--wait-net", type=float, default=0,
                    help="wait up to N s for a network address before starting (service use)")
    args = ap.parse_args()

    log("TAUVER link bridge %s on %s" % (VERSION, socket.gethostname()))
    if args.wait_net > 0 and not args.no_ros:
        wait_for_network(args.wait_net)

    if args.fake:
        log("FAKE NUCLEO: no hardware involved, the board is simulated in here")
        link = FakeNucleo()
    else:
        try:
            import serial  # noqa: F401
        except ImportError:
            log("ERROR: pyserial is missing. fix: sudo apt install python3-serial")
            return 2
        link = SerialLink(args.port, args.baud)

    if args.no_ros:
        return run_no_ros(link)
    return run_ros(link, args.fake)


if __name__ == "__main__":
    sys.exit(main())
