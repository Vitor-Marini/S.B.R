# Lab 04 — Odometry calibration

> **In one sentence:** measure how wrong the gyro and the wheels are about turning, with a tape line and five turns.

**Status:** ✅ done. `gzs` 1.002, `track` 0.160 m.

## The 5-turn test

1. Robot balancing, front aligned with a tape line on the floor.
2. `turn 1800` (and `turn -1800`): five full turns, read what the gyro and the wheels (`head`) measured.
3. Compare with how far off the tape it ended.

| | gyro | wheels (150 mm track) | real |
|---|---|---|---|
| left, over grout | 1799.7° | 1887.7° | ~1803° |
| right, mid tile | 1799.8° | ~1957° | ~1803° |

→ the gyro is good (0.2 %); the wheels over-count by 5–9 % and unevenly (tire slip in turns) → effective track
0.160 m, and turns use the gyro. `turn` now stops within 0.3° (was 1°, always ~1° short).

Result: the demo (go 50, turn 180, come back, turn 180) ends **0.5 cm** from the start, < 1° off.

*Full write-up coming next.* Code: `firmware/src/velocity.cpp` (`turn`), `firmware/src/cli.cpp` (`head`).
