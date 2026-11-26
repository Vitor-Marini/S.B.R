# Lab 08 — Isaac Lab (GPU)

> **In one sentence:** the same robot as lab 07, but simulated thousands at a time on the RTX 4060 Ti — to see what
> GPU-parallel training changes compared with MuJoCo on the CPU.

**Status:** 🟡 steps 1–3 done: model validated, v6 trained on the GPU (300M steps, 15 min) — better than v4/v5 standing, still not the LQR.

## Where it runs

The Windows PC with the RTX 4060 Ti (8 GB), inside **WSL2 Ubuntu** — on Windows itself, Smart App Control blocks
NVIDIA Warp's unsigned DLL. Isaac Lab 3.0 with the **Newton** physics backend (MuJoCo Warp: MuJoCo on the GPU); no
Isaac Sim needed (it asks for 16 GB of VRAM).

```bash
# once, in WSL (no sudo needed: uv brings its own Python 3.12)
uv venv --python 3.12 ~/isaac/env_isaaclab
uv pip install --python ~/isaac/env_isaaclab/bin/python "isaaclab[all]==3.0.0" --index https://pypi.nvidia.com --torch-backend cu130
source ~/isaac/env_isaaclab/bin/activate
```

**First number:** Isaac's cart-pole example, 4096 copies, trains in ~40 s at **~252k steps/s** — MuJoCo on 32 CPU
processes did ~7.7k steps/s on our robot (not the same task: the robot is heavier to simulate).

## Step 1: the robot model (`model/build_usd.py`, `model_check.py`)

Isaac loads robots as **USD**, so `model/build_usd.py` writes `model/sbr.usda` from the same `model/params.json` as
the MuJoCo model (no URDF: that converter ships with Isaac Sim). The Fusion 360 meshes (`model/meshes/`) are the
visuals; physics stays measured:

| | from |
|---|---|
| body mass 1.02 kg, CoM 37 mm, pitch inertia from L = 0.09 m | `params.json` (measured) |
| wheels Ø74 mm, 150 mm track, stepper = velocity servo (0.2 N·m·s, 0.3 N·m) | `params.json` |
| look | Fusion meshes (body, wheels, battery) |
| collisions | wheel cylinders + a box for the body + a box for the **battery** |

The MuJoCo model never needed body collisions (a fall ends the episode); here they are real: falling forward it lies
on the body's front (89.9°, wheels lifted 2 mm — the body reaches 40 mm forward, the wheel only 37.5); falling
**backward the battery hits first** (87.2°): it sticks out 5 mm past the back.

```bash
python model/build_usd.py                       # → model/sbr.usda
python labs/08-isaac-lab/model_check.py         # checks: masses, falls the right way, rests above the floor
python labs/08-isaac-lab/model_check.py --viz   # window (WSLg): 16 robots dropped at random tilts, every 3 s
```

## Step 2: validation with the firmware LQR (`validate.py`)

Same loop as lab 07's `sim.py` (physics 1 ms, control 5 ms, the firmware's complementary filter, 1-tick sensor delay,
gyro noise, the calibrated 0.008 N·m "dirt"), the firmware LQR vectorized — and every scenario at once, one robot per
parallel world (8 standing, 4 doing `move 50`, 6 pushed at 2–14 N). **No tuning: first run.**

| LQR | real robot | lab 07 MuJoCo (CPU) | **lab 08 Newton (GPU)** |
|---|---|---|---|
| standing still | ±0.3–1 cm, ~±0.2° | ±0.5–0.7 cm, ±0.15° | **±0.4–0.6 cm, ±0.12–0.18°** |
| `move 50` | ~1 cm over, 4.8 s | 1.4 cm, 4.0 s | **1.0–1.2 cm, 3.9–4.0 s, ends 0.1–0.9 cm off** |
| push 0.1 s at the top | (LQR > PID) | 6 N | **6 N** |

Two physics engines, one set of measured numbers, same robot: the model (`params.json`) is what makes them agree.

**Speed — and why this loop is not the training loop yet:**

| robots in parallel | control ticks/s | robot-steps/s |
|---|---|---|
| 18 | 28 | 510 |
| 512 | 30 | 15k |
| 4096 | 25 | **102k** (MuJoCo, 32 CPU processes: 7.7k) |

The time per tick is flat (~35 ms) no matter how many robots: it is Python + copying GPU↔CPU every 1 ms substep, not
physics. Even so, 4096 robots already run 13× faster than lab 07. For training, the controller/filter/reward move onto
the GPU (torch/warp) and the 5 substeps get captured in a CUDA graph. (`--viz` therefore plays ~7× slower than real
time.)

```bash
python labs/08-isaac-lab/validate.py          # report (~2.5 min)
python labs/08-isaac-lab/validate.py --viz    # watch the 18 robots
```

## Step 3: v6 trained on the GPU (`sbr_env.py`, `train.py`, `eval_in_mujoco.py`)

`sbr_env.py` is lab 07's v5 environment (scenarios, events, randomization, delays, firmware reference generator,
complementary filter, 24-input observation) as tensors over 8192 robots, physics batched per robot (mass, L, torque,
friction) and the 5 physics substeps recorded in a CUDA graph. The LQR survives it like in MuJoCo (86 % at
difficulty 0.8 vs 88 %). PPO from Isaac Lab (rsl_rl), same 2×64 tanh network as lab 07.

| | lab 07 (MuJoCo, CPU) | lab 08 (Newton, GPU) |
|---|---|---|
| speed (env + PPO) | 7.7k steps/s | **~330k steps/s** |
| run | v5: 30M steps, 62 min | **v6: 300M steps, 15 min** |

**First v6 attempt failed:** with the tilt weight at 0.5° from the start, any tilt > 3° saturates the state cost; a
robot that can't balance yet only gets "use less action" → action noise collapsed to 0, 0.3 s episodes. Fix: a
curriculum on the weight too, 4° → 0.5° with the difficulty (20M-step tests: 0.5° fixed 0.19 s episodes, 4° → 0.5°
6.0 s).

**Judged in MuJoCo, on lab 07's battery** (a policy trained in Newton, tested in another simulator: it transfers):

| standing | tilt | peak | action saturated |
|---|---|---|---|
| LQR | ±0.42° | 20 Hz | 45 % |
| v4 (lab 07) | ±1.56° | 8.6 Hz | 97 % |
| v5 (lab 07) | ±2.35° | 1.5 Hz | 9 % |
| v6 it 254 → 1524 | ±2.31° → **±1.03°** | 3.9 → 9.2 Hz | 26 → 32 % |

| test | LQR | v6 it 1016 | v6 it 1524 (final) |
|---|---|---|---|
| still | ±0.7 cm ±0.43° | ±0.9 cm ±1.20° | ±0.9 cm ±1.04° |
| still, setpoint +2.75° | ±0.7 cm | **drives away (±6 m)** | **FELL** |
| joystick drive | max tilt 9.4° | **7.3°** | **6.4°** |
| demo | 9.6 cm off | 11.5 cm | 20.1 cm |
| push ladder | 6 N | 6 N | 4 N |
| grout at 0.3 m/s | 9.0° | 8.7° | **4.6°** |
| 50 random robots (diff 1.0) | 32/50 | 27/50 | 22/50 |
| 50 random robots (diff 0.8) | 43/50 | 31/50 | 33/50 |

Tightening the tilt weight worked as intended — standing wobble went from v5's lazy ±2.3° to ±1.0°, and v6 drives
and rolls over grout with **less** tilt than the LQR — but it is creeping back toward chatter (9 Hz, 32 %
saturated) and lost robustness: a constant +2.75° setpoint error (the real gyro-zero transient that broke v1) makes
it drive away or fall, and it survives fewer random robots. Not for the robot yet.

## Step 4: v7 residual (LQR + network)

v4 → v5 → v6 each fixed one weight and broke something else, so v7 changes the setup instead of the weights:

- **action = firmware LQR + network × ±1 m/s²** — the robot starts as the LQR (zero correction = same 86 % survival
  as the LQR in the selftest), the network only learns corrections;
- **privileged critic** — during training the critic also sees 12 true values (real tilt, setpoint error, mass, L,
  torque, friction, pushes, grout); the actor (what goes to the ESP) keeps the 24 real inputs;
- **effort penalty on the correction only** (the LQR itself is saturated 45 % of the time);
- **checkpoint chosen by a short GPU battery** (`train.py score`: the same 4096 robots, difficulty 0.8, 10 s).

```bash
python labs/08-isaac-lab/train.py 300e6          # 16 min, then scores all 13 checkpoints
.venv/bin/python labs/08-isaac-lab/eval_in_mujoco.py labs/08-isaac-lab/models/ppo_v7_isaac_1270.npz
```

GPU score (episodes were ~13.5 s long from the first iteration — v6 started at 0.3 s):

| | survive | tilt rms | action jerk rms |
|---|---|---|---|
| LQR | 85.2 % | 3.39° | 1.49 |
| v7 it 127 | 86.2 % | 3.37° | 1.42 |
| v7 it 635 | 83.1 % | 2.96° | 1.13 |
| v7 it 1270 | 83.1 % | 2.81° | 1.19 |

Lab 07 battery in MuJoCo:

| test | LQR | v7 it 127 | v7 it 635 | v7 it 1270 |
|---|---|---|---|---|
| standing wobble | ±0.42°, 20 Hz, 45 % sat. | ±0.41°, 36 % | **±0.08°, 0 %** | **±0.29°, 0 %** |
| still | ±0.7 cm ±0.43° | ±0.6 cm ±0.41° | ±0.9 cm ±0.10° | ±1.0 cm ±0.30° |
| still, setpoint +2.75° | ±0.7 cm | ±0.7 cm | **±27 cm** | **±47 cm** |
| joystick drive | 9.4° | 9.3° | 8.8° | 8.7° |
| demo | 9.6 cm off | 9.4 cm | 12.1 cm | 12.2 cm |
| push ladder | 6 N | 6 N | 4 N | 4 N |
| grout at 0.3 m/s | 9.0° | 8.7° | 7.2° | 7.3° |
| 50 random robots (diff 1.0) | 32/50 | 28/50 | 23/50 | 29/50 |
| 50 random robots (diff 0.8) | 43/50 | 38/50 | 40/50 | 41/50 |

What it learned: to **cancel the LQR's chatter** — standing goes from 45 % saturated at 20 Hz to 0 % and the tilt
drops (±0.08° at it 635), and it tilts less driving and on grout. The price is the same as v6 in a milder form: it
softens the LQR's position hold, so a constant setpoint error makes it creep (27–47 cm instead of 0.7 cm, but it no
longer falls or drives away for metres), pushes drop to 4 N and it loses a few random robots. Early checkpoints
(it 127) are basically the LQR. **Best of all versions so far, but still not ≥ LQR on every line: not for the robot.**

Next to try (one change each): a reward term for wandering off the start point with a setpoint error (the creep is
the failing line), or a smaller correction (±0.5 m/s²) so it can't override the position hold.

## Step 5: v8 = v7 + drift cost — negative result

Idea: v7's reward only saw `ex`, which is clamped at the 9 cm slip limit (and xref slides), so creeping far cost no
more than 9 cm. v8 added a cost on the distance from the commanded path without the slip reset (∫vr dt, 0.15 m).

It did not work: GPU drift rms went **up** during training (28 → 55 cm; the LQR itself scores 27 cm on this metric,
because pushes past the slip limit and move starts legitimately move the path), and in MuJoCo it 1143 creeps 47 cm with
the +2.75° setpoint exactly like v7. The actor can't see that distance (only the critic does), and most of the cost is
unavoidable even for the LQR — wrong lever.

**The actual cause, measured in MuJoCo** (still, setpoint +2.75°, after 3 s):

| | ex | network correction | x over 17 s |
|---|---|---|---|
| v7 it 1270 | −9.0 cm (at the slip limit 100 % of the time) | **+1.00 m/s² constant** (its max) | −23 → −117 cm |
| v8 it 1143 | −9.0 cm (100 %) | **+1.00 m/s²** | −24 → −118 cm |
| v8 it 381 | −8.9 cm (84 %) | +0.33 m/s² | −12 → −19 cm |

With a 2.75° setpoint error the LQR spends 0.744 × 2.75 ≈ 2.05 m/s² of its position hold (27.67 × 0.09 ≈ 2.49 m/s²),
leaving ≈ 0.44 m/s². A **constant** correction bigger than that pins ex at the slip limit and xref slides: steady creep.
The network learned a constant offset (which helps on average in the randomized robots) that the real robot's
setpoint error turns into creep.

## Step 6: v9 = v7 + high-passed correction — the setpoint line is fixed

The network's correction goes through a high-pass (correction − its low-pass, τ = 0.5 s): a constant correction fades
to zero in ~3 s, so standing still it is exactly the LQR for any setpoint error; fast corrections pass. The drift cost
of v8 is gone. `train.py score` now also runs the failing line (still, +2.75°, 256 robots) — it reproduces MuJoCo
(v7 it 1270 creeps 55 cm there too, the LQR 0.5 cm). On the robot this costs one extra float in the firmware.

GPU score: every checkpoint creeps ≤ 0.5 cm with the setpoint error; survival 84–86 % (LQR 85 %); action jerk −20 %.

| test | LQR | v9 it 762 | v9 it 889 | v9 it 1524 |
|---|---|---|---|---|
| standing wobble | ±0.42°, 20 Hz, 45 % sat. | **±0.29°, 0 %** | ±0.32°, 0 % | ±0.93°, 2 % |
| still | ±0.7 cm ±0.43° | ±1.1 cm **±0.18°** | ±2.4 cm ±0.38° | ±4.7 cm ±1.04° |
| still, setpoint +2.75° | ±0.7 cm | **±0.7 cm** | **±0.7 cm** | **±0.6 cm** |
| joystick drive | 9.4° | 9.7° | 9.7° | 10.5° |
| demo | 9.6 cm off | 11.0 cm | 11.9 cm | 12.4 cm |
| push ladder | 6 N | 4 N | 4 N | 4 N |
| grout at 0.3 m/s | 9.0° | 8.5° | 8.5° | 9.3° |
| 50 random robots (diff 1.0) | 32/50 | 29/50 | 26/50 | 26/50 |
| 50 random robots (diff 0.8) | 43/50 | 42/50 | 39/50 | 40/50 |

The creep is gone (v7/v8: 27–47 cm) and it 762 keeps the chatter win (0 % saturated, tilt ±0.18° standing). Still not
≥ LQR everywhere: **pushes 4 N instead of 6 N** (every learned version v6–v9 lost here), a few random robots fewer,
the demo ends 1.4 cm farther and joystick tilt is 0.3° higher. Late checkpoints get worse (it 1524 sways ±1°): the
mid-training ones are the ones to keep. Not for the robot yet.

**Why pushes drop to 4 N — measured** (v9 it 762, MuJoCo, 6 N for 0.1 s; the LQR alone survives it):

| after the push | tilt | LQR asks | correction | sent |
|---|---|---|---|---|
| 40 ms | +1.8° | +4.00 (saturated) | **−0.93** | +3.07 |
| 80 ms | +5.0° | +4.00 | **−0.55** | +3.45 |
| 120 ms | +10.2° | +4.00 | **−0.31** | +3.69 |
| 140 ms → | 12–24° | +4.00 | 0 | +4.00 — too late, falls |

For the first ~120 ms the correction **pulls back exactly when the LQR needs all 4 m/s²** — the chatter-cancelling
reflex (standing, the LQR saturates for nothing; here it saturates for real) can't tell the two apart in time. And it
never had to learn the difference for a push this big: training pushes are `±6 N × difficulty`, at most **±4.8 N** with
the curriculum's d_max 0.8 — the 6 N rung of the battery is outside what it ever saw.

## Next

- v10: training pushes up to ±8 N regardless of the difficulty curriculum (one line), so it learns not to pull back on
  a real push; same battery, same rule.
- If a policy passes: firmware + `export.py` with 24 inputs (and the LQR + correction sum).
