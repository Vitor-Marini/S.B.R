# Lab 08 — Isaac Lab (GPU)

> **In one sentence:** the same robot as lab 07, but simulated thousands at a time on the RTX 4060 Ti — to see what
> GPU-parallel training changes compared with MuJoCo on the CPU.

**Status:** 🟡 steps 1–2 done: the model matches the real robot and lab 07 with the firmware LQR. Next: train.

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

## Next

3. Train with thousands of robots in parallel; compare with v3/v4/v5.
