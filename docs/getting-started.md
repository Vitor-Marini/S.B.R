# Getting started with S.B.R

Day-to-day guide: flash, connect, watch live data, start balancing and tune.
The robot talks over USB **or** WiFi (same text protocol). Hardware in [hardware.md](hardware.md); the
history of every test in [journal.md](journal.md); the theory behind each part in the [labs](../labs/).

All commands run from the **repository root**:

```bash
cd ~/personal/dev/S.B.R
```

---

## 1. Flash the firmware

The USB port has **one owner**: stop the bridge in USB mode (Ctrl+C) before flashing.
Put the robot **on its stand** first (it reboots while flashing).

```bash
~/.platformio/penv/bin/pio run -d firmware -t upload
```

Settings (gains, setpoint, controller mode...) live in `/config.json` on the flash and **survive** a flash.
WiFi is saved once over USB with `wifi <network> <password>` (2.4 GHz, no spaces in the name).

---

## 2. Control it

### From the phone (no PC needed)

Open `http://sbr.local` (or the robot's IP) on the phone: state, start/stop/calibrate, on-screen joystick,
live charts, a 2D trail of where it went, turn buttons (↺ 90° / 180° / ↻ 90°), a console and CSV download.
A DS4 paired **with the phone** works on the page too (Gamepad API).

### From the PC

```bash
tools/sbr.sh                 # WiFi (sbr.local) + PlotJuggler + bridge; `tools/sbr.sh usb` = over the cable
python3 tools/gamepad.py     # other terminal: DS4 plugged into the PC (USB or Bluetooth)
python3 tools/teleop.py      # or the keyboard
```

The DS4 can't pair with the ESP32-C3 directly (it only has Bluetooth LE; the DS4 is Bluetooth Classic).

DS4: left stick = forward/back, right stick = turn · △ start · ○ stop moving · □ calibrate gyro (IDLE, still) ·
✕ **stop** (balance off: hold the robot) · L1/R1 speed · ←/→ turn 90° (page) · **Options** = motor test mode
(IDLE only, wheels in the air).

### From a program

`tools/sbr.py` is the Python SDK (standard library only, talks to the robot's WebSocket):

```python
from sbr import Robot
r = Robot("sbr.local")
r.wait_state(r.ACTIVE)
r.move(50); r.turn(180)      # block until "move ok" / "giro ok"
r.drive(0.2, 0)              # free driving: repeat at 10–20 Hz (0.5 s watchdog on the robot)
```

`python3 tools/sbr.py demo` (go 50 cm, turn 180°, come back) · `python3 tools/sbr.py medir 50` (measure one move).

---

## 3. Live charts and logs

`tools/sbr.sh` opens PlotJuggler with `tools/sbr_layout.xml` already listening on UDP 9870 and turns the
`stream` on. Everything the stream sends is logged to `logs/YYYYMMDD-HHMMSS.csv`; summarize it with:

```bash
python3 tools/analyze_log.py        # latest log: starts, driving, turns, falls
```

| Signal | Meaning |
|--------|---------|
| `pitch` | filtered tilt (°), positive = leaning forward |
| `acc` / `gyroint` | tilt from the accelerometer only / from the integrated gyro only (diagnostics) |
| `rate` | pitch rate (°/s) |
| `yaw` / `head` | heading from the gyro / from the wheels (°) |
| `speed` | wheel speed (m/s) |
| `accel` | controller output: wheel acceleration (m/s²) |
| `err` | pitch − (setpoint + voff) (°) |
| `voff` | angle offset: cascade output (PID) or θ_ref (LQR) (°) |
| `vtgt` / `wtgt` | requested speed (m/s) and turn rate (°/s) |
| `yrate` | turn rate (°/s, + = left) |
| `state` | 0 INIT · 1 CALIBRATE · 2 IDLE · 3 STARTING · 4 ACTIVE · 6 FAULT |
| `tick` / `temp` | control loop rate (must be 200) / MPU temperature (°C) |

---

## 4. Balancing

1. **Calibrate on the stand, still** (□ or `cal`): the gyro zero is measured here.
2. Floor with grip. Slack USB cable or none (a pulling cable hides real behavior).
3. `start` (△). It waits in STARTING until it's within 3° of the setpoint for 0.3 s, then goes ACTIVE.
   With the right setpoint (~−1.8°) it engages straight from the stand.
4. To stop: **hold it**, then `stop` (✕). Never `stop` with it standing free: the motors stop and it falls.
5. If it falls (> 45°) → FAULT. △ starts again from FAULT.

The default controller is **LQR** (see [labs/03-lqr](../labs/03-lqr/)). `ctl pid` switches to the PD + cascade
([labs/01-pid-cascade](../labs/01-pid-cascade/)) while not balancing; `save` keeps the choice.

---

## 5. Terminal commands

| Command | What it does |
|---------|--------------|
| `start` / `stop` | balance on / off |
| `drive <m/s> <°/s>` | drive / turn (+ = forward / left). Lasts 0.5 s: repeat at 10–20 Hz |
| `move <cm>` · `turn <deg>` | go a distance / turn in place; reply `move ok` / `giro ok` |
| `ctl [lqr\|pid]` | controller (only while not balancing) |
| `cal` | recalibrate the gyro (IDLE, robot still) |
| `state` · `status` (`st`) | current state · full config and telemetry |
| `angle` · `raw` · `regs` | tilt sources and loop timing · raw MPU data · MPU registers |
| `stream [on\|off]` | 50 Hz telemetry |
| `motors <L> <R>` · `motor left\|right\|both <steps/s>` | motor tests (IDLE, wheels in the air) |
| `wifi [network password]` | network status, or save a network |
| `tune <param> <value>` | change a parameter (saved now, or when balancing stops) |
| `save` · `load` · `reset` | write config · reload · reset integrators |

### `tune` parameters

| Param | Value | Meaning |
|-------|-------|---------|
| `lkt` `lkw` `lkx` `lkv` | 0.744 · 0.0707 · 27.67 · 14.04 | **LQR** gains: tilt (m/s² per °), pitch rate (per °/s), position (per m), speed (per m/s) |
| `kp` `kd` | 0.35 · 0.03 | PD balance gains (PID mode) |
| `vkp` `vki` | 4 · 3 | cascade speed loop (° per m/s) (PID mode) |
| `hkp` | 0.7 | cascade position hold (1/s) (PID mode) |
| `mva` `mvs` | 0.1 · 0.15 | `move` profile: acceleration (m/s²), top speed (m/s) |
| `sp` | ~−1.8 | setpoint (°): balance angle |
| `vmax` | 0.8 | wheel speed limit (m/s) |
| `ykp` `yki` | 0.001 · 0.01 | turn PI |
| `gzs` · `track` | 1.002 · 0.160 | gyro Z scale · effective wheel track (m), see [labs/04](../labs/04-odometry-calibration/) |
| `gxz` · `alpha` | −0.007 · 0.995 | gyro cross-axis leak · complementary filter |
| `spr` `accel` | 3200 · 400000 | steps per rev (1/16) · motor accel safety cap (steps/s²) |
| `lrev` `rrev` · `abx` `abz` | 1 1 · 0 0 | motor direction · accelerometer bias |

---

## 6. Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| upload fails / "port busy" | bridge open on USB: Ctrl+C it |
| page loads but shows 0 frames | `stream` off (the page re-enables it) or USB connected with nobody reading |
| `/dev/ttyACM0` disappears when tilting | bad/short USB cable |
| `i2cWriteReadNonStop returned Error -1` | MPU read failed: loose/long I²C wires or motor noise |
| `angle` shows `Tick` well below 200 Hz | something blocking the loop |
| drifts one way standing still (PID mode) | wrong setpoint; LQR tolerates ~3° |
| `sbr.local` doesn't resolve | robot off or WiFi not connected; use the IP |
