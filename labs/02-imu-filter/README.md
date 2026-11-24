# Lab 02 — IMU filter and gyro zero tracking

> **In one sentence:** blend the gyro (fast, drifts) with the accelerometer (noisy, no drift) into one tilt angle,
> and keep correcting the gyro zero while the robot runs.

**Status:** ✅ in use.

## In the code

| Where | What |
|---|---|
| `firmware/src/imu_manager.cpp` → `read()` | complementary filter `pitch = a·(pitch + rate·dt) + (1−a)·acc`, α 0.995 (τ ≈ 1 s) |
| same, `read()` | gyro Y zero tracked from `pitch − acc` (τ 20 s) |
| `trackGyroZ()` (called from `velocity.cpp`) | gyro Z zero tracked when there's no turn command (wheels say "not turning") |
| `calibrateGyro()` / `trackGyroBias()` | 500-sample zero at start; slow correction in IDLE |

## Key results (measured)

- A gyro Y zero error of b °/s shifts the tilt by b·τ: measured ~1.1° → the "right setpoint" changed every session.
  After tracking: pitch matches the accelerometer within ~0.1°; real setpoint stable at −1.7°.
- Heading drifted ~1.7 °/s standing still → after Z tracking: 0.8° in 17 s.

*Full write-up coming next.* History: [journal](../../docs/journal.md).
