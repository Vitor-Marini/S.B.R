# S.B.R hardware

Reference for the hardware **as it is built today** and what's left to improve.
How each thing was found out is in [journal.md](journal.md).

## Parts

| Part | Detail |
|------|--------|
| MCU | ESP32-C3 Supermini (RISC-V, 160 MHz, **single core, no FPU**, WiFi + **Bluetooth LE only**) |
| IMU | MPU6050 (I²C 400 kHz, DLPF 44 Hz, ±8 g, ±500 °/s) |
| Motors | 2× NEMA17, 200 steps/rev |
| Drivers | 2× DRV8825, **1/16 microstepping** (3200 steps/rev) |
| Wheels | printed PLA, 74 mm diameter |
| Battery | Makita BL1016 (10.8–12 V); the ESP is powered from it through a DC-DC |
| Chassis | rigid since 2026-11-21 (beam between the side plates); parts in `hardware/3d-models/` |

## Pinout

| Signal | ESP32-C3 | Note |
|--------|----------|------|
| SDA (MPU) | GPIO 8 | |
| SCL (MPU) | GPIO 9 | |
| STEP motor "L" | GPIO 0 | **firmware "L" is the robot's LEFT wheel** (looking forward; measured 2026-11-21) |
| DIR motor "L" | GPIO 1 | |
| DIR motor "R" | GPIO 2 | |
| STEP motor "R" | GPIO 3 | |
| M2 on both drivers | 3.3 V (or 5 V) | 1/16; M0 and M1 floating (internal pull-down) |
| Driver ENABLE | not wired | drivers always powered (whine and warm up when idle) |

DRV8825 microstepping:

| M2 | M1 | M0 | Mode | `tune spr` |
|----|----|----|------|------------|
| 0 | 0 | 0 | full step | 200 |
| **1** | **0** | **0** | **1/16 (current)** | **3200** |
| 1 | 0 | 1 | 1/32 | 6400 |

## Conventions checked on the robot

- **Front = the power switch side** (−Y in CAD). Positive pitch = leaning that way, and a positive wheel
  command drives that way. Check against this physical reference, not "front" from memory.
- Fall axis: accelerometer X/Z + **gyro Y** (MPU lying flat, chip up).
- **Positive gyro Z = turning left** (counter-clockwise seen from above).
- Since the 2026-11-21 rebuild both motors are reversed: `tune lrev 1`, `tune rrev 1` (`motor both +N` drives
  forward). After a rebuild, if the robot "accelerates and falls", check this sign first.
- **Firmware "L" (GPIO0) = left wheel** looking forward. Measured turning in place: `motors 1600 -1600` →
  clockwise (yaw decreases). If turning "never stops", this mapping changed: flip the sign in `balance.cpp`.
- With the MPU in this orientation, `regs` should read `75=68 6B=01 19=00 1A=03 1B=08 1C=10`.

## Physical parameters

| Quantity | Value | Source |
|---|---|---|
| Total / body mass (without wheels) | 1.06 kg / 1.02 kg | mk2 CAD, scale-calibrated |
| Body center of mass above the axle (d) | 37 mm (±15 %) | CAD |
| Body inertia about the axle (I) | 3.93e-3 kg·m² | CAD |
| **Effective pendulum length L = I/(m·d)** | **0.09 m** (CAD: 0.10) | **pendulum test** |
| Wheel radius | 37 mm (firmware: 74 mm diameter, measured) | measured |
| Wheel track, center to center / effective in turns | 150 mm / **160 mm** | measured / 5-turn test |
| Gyro Z scale | 1.002 | 5-turn test |
| MPU chip above the axle | 37 mm | CAD |

- **Pendulum test (2026-11-22):** robot upside down swinging about the wheel axle, gyro recorded from the stream.
  Period at 15–30° amplitude: **T ≈ 0.61 s** → `L = g·(T/2π)² ≈ 0.09 m`. Above ~50° the period grows (non-linear)
  and the gyro **saturates at ±500 °/s**.
- **Turn calibration (2026-11-23):** `turn ±1800` against a tape line. Gyro off by ~3° in 5 turns both ways;
  wheels over-counted 4.9 % / 8.7 % (uneven tire slip in turns) → effective track 0.160 m.
  Details in [labs/04-odometry-calibration](../labs/04-odometry-calibration/).

## Where to mount the IMU

- The **gyro** reads the same anywhere on a **rigid** body: only mounting stiffness matters.
- The **accelerometer** reads gravity + the acceleration of its own point. In the linear pendulum
  (L = effective length, a = wheel acceleration, h = sensor height above the axle):

  `reading_x ≈ a·(1 − h/L) + g·θ·(1 + h/L)`

  - **At the axle (h = 0):** wheel acceleration shows up fully, but it is **known** (stepper: commanded = real),
    so the firmware could subtract it. Recommended, together with stiffness.
  - **At the center of percussion (h ≈ L):** wheel acceleration cancels, but L is uncertain and the chassis flexes
    up there. Not worth it.
- The complementary filter (α = 0.995) trusts the gyro below ~1 s, so the accelerometer only matters for slow effects.

## What's left to improve (by priority)

1. ✅ **Rigid chassis** (2026-11-21).
2. ✅ **Low, rigid MPU** (2026-11-21), ~37 mm above the axle. Still worth moving away from the drivers (heat → gyro drift).
3. **Power.** There were MPU resets and ESP reboots from bad contacts; a 100–470 µF cap near the ESP/MPU and firm connectors.
4. **Battery monitor.** Divider (e.g. 100 kΩ + 33 kΩ → ~3.1 V at 12.6 V) on a free ADC pin.
5. **ENABLE on a GPIO**, to turn the motors off in IDLE/FAULT (no whine, no heat).
6. **Driver Vref**: ~0.6–0.7 V for a ~1.2–1.5 A NEMA17 (never measured). More current = more torque, fewer missed steps.
7. **Tires with more grip**: uneven slip shows up in turns and on tile grout.
8. ✅ **Battery on top** (2026-11-21).
