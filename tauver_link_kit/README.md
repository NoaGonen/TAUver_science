# TAUVER Link Kit · link-1.1.0

Mission 1 of TAUVER 2027: prove the pipe **Nucleo → Jetson → network → command station**, and back, before any
real sensor is involved. The full step-by-step protocol, the troubleshooter and "Ask NIX" live in the
**TAUVER Link field manual** artifact. This file is the short version.

```
 NUCLEO-G491RE ──USB serial, JSON lines──► MACHINE A: link_bridge.py ──ROS 2 over the network──► MACHINE B: cmd_station.py
 (tauver_link_fw)  ◄──command lines────────  (Jetson, or laptop A)       ◄──/tauver/science/cmd────   (CMD station, or laptop B)
```

## What's in the box

| Path | What it is | Runs on |
|---|---|---|
| `firmware/tauver_link_fw/tauver_link_fw.ino` | the pinger: heartbeat every 1 s, `CMD:PING=<id>` → `pong`, LED, fake scale + fake lid | Nucleo (or a Mega) |
| `host/link_bridge.py` | serial ⇄ ROS 2. Finds the board, reconnects, reports its own health. `--fake` simulates the board, `--no-ros` is a plain serial monitor | Machine A |
| `host/cmd_station.py` | ping, 20-ping test, LED, closed-loop lid demo, link status | Machine B |
| `tv` | the one command to remember: type `tv` for the menu | every machine |
| `setup.sh` | one-time setup per machine (ROS env line in `~/.bashrc`, pyserial, serial permission) | every machine |
| `tauver_env.sh` · `tauver.conf` | the single place for ROS settings (`ROS_DOMAIN_ID=27`) · this machine's settings | every machine |

## The 60-second version

```bash
# every machine, once
unzip ~/Downloads/tauver_link_kit.zip -d ~ && bash ~/tauver_link_kit/setup.sh
# (log out/in if it says so) then
tv doctor

# Machine A (Nucleo plugged in)        # Machine B
tv serial      # link 1: board talks?   tv cmd     # p ping · t test · l LED · d demo · s status
tv bridge      # link 2+3: on ROS 2     tv test    # PASS/FAIL
```

## Headless Jetson

From your laptop, over one USB-C cable (the Jetson is `192.168.55.1`):

```bash
tv set jetson_user <name> ; tv sshkey ; tv push ; tv jetson setup
tv jetson static 192.168.27.10/24 ; tv jetson service install
```

From then on: power on = bridge on. `tv jetson logs` shows its live log.

## Topics (same names as the Science Pack 3.0 bridge)

| Topic | Type | Direction |
|---|---|---|
| `/tauver/science/telemetry` | String (JSON `tel` frames) | out, best-effort |
| `/tauver/science/events` | String (boot, ack, pong, event, health, bridge notes) | out, reliable |
| `/tauver/science/cmd` | String (one command line) | in |
| `/tauver/link/status` | String (bridge health JSON, 1 Hz) | out |

## Firmware commands

`CMD:PING[=id]` · `CMD:LED=ON|OFF|AUTO` · `CMD:SIM=ON|OFF` · `CMD:LID=ROCK,CLOSE|OPEN` · `CMD:STATUS` · `CMD:HELP` ·
`CMD:RESET`. Every command is answered by an `ack`. Same shape as serial contract v3.

## Tested (2026-10-31, before shipping)

- The firmware compiled for a Mega with the real Arduino core (9 KB, no warnings in the sketch), and ran natively
  against scripted commands: every reply is valid JSON, including bad input (unknown command, bad id, a 200-char line).
- The full chain on real ROS 2 (Jazzy, Fast DDS): firmware on a virtual serial port → bridge → station. Results:
  20/20 pings, rtt ≈ 2-12 ms locally; lid demo PASS at 100 g; USB pulled → `SERIAL_DOWN` and `NUCLEO SILENT`;
  command while unplugged → `CMD_DROPPED`; replugged → reconnect + `NUCLEO ALIVE`; second bridge on the same
  port → refused as BUSY; garbage bytes → `BAD_LINE`; Plan B `peers.xml` accepted by Fast DDS; fake mode PASS.
- Service mode: the systemd unit passes `systemd-analyze verify`, and the bridge restarts itself (exit 3) when an
  Ethernet IP appears after boot.
- **Not tested here:** a real Nucleo and USB cable, a real Jetson, two physical machines, Humble itself. Those are
  exactly what the field manual walks you through.

## Mission 2 · one real scale (link-1.1.0)

`firmware/tauver_scale_fw/tauver_scale_fw.ino` (scale-1.0.0): the pinger plus a NAU7802 + one 4-wire load cell.
Same board settings, same wire contract, same topics: **the bridge and the Jetson need no change.**

| File | Status | What changed |
|---|---|---|
| `firmware/tauver_scale_fw/tauver_scale_fw.ino` | **new** | NAU7802 driver (no library), live weight in every `tel` (5 Hz, `rock_g`), `CMD:TARE`, `CMD:CALW=<g>`, `CMD:WEIGH`, `CMD:CAL_CLEAR`, `CMD:SCALE_RESET`; tare + calibration saved in flash; lid demo on real weight |
| `host/cmd_station.py` | changed (laptop only) | keys `z` tare · `k <grams>` calibrate · `g` live weight · `e` weigh once; `RESULT` lines; the demo `d` waits for a **stable** ≥ 100 g |
| `tv` | changed (menu text + version) | lists the new keys |
| `host/link_bridge.py`, `tauver_env.sh`, `setup.sh`, the pinger | unchanged | |

Wiring: NAU7802 VIN → 3V3 · GND → GND · SDA → D14 (PB9) · SCL → D15 (PB8). Load cell red → E+, black → E−,
white → A−, green → A+. Address 0x2A. Gain ×128, internal LDO 3.0 V, 80 samples/s.

`tel.data` now carries: `rock_g` (grams, `null` until tared + calibrated), `raw` (counts, mean of 8), `stable`
(the last 0.5 s spans < 1 g), `tared`, `cal`, `scale` (`OK`/`OFF`). LD2: one flash per second = scale OK, two = scale
not found.

Calibration order: empty + still → `z` (tare) · known mass on → `k <grams>` · check with `e` and a second mass.
Refusals: `TARE_FIRST`, `UNSTABLE_KEEP_IT_STILL` (> 20 000 counts of movement during the average),
`NO_WEIGHT_SEEN_CHECK_WIRING` (< 1 000 counts change), `SCALE_OFFLINE`, `BUSY`.

Tested 2026-11-03 against a simulated NAU7802 (register-level, 80 SPS, ±60 counts noise) through the real bridge on
ROS 2: tare, calibrate (100 g → 1047 counts/g), weigh 37.5 g / 250 g exact, TARE_FIRST, BUSY, bad argument, calibration
reloaded after a restart, scale unplugged → `SCALE_LOST` and back by itself, lid demo PASS on 130 g, `tv test` 20/20.
Compiled for a Mega with the real Wire + EEPROM libraries (19.5 KB). **Not tested here: the real NAU7802 and load cell,
and the STM32 build itself** (same STM32 calls as the Science Pack: `Wire.setSDA/SCL`, `Serial.setRx/Tx`).
