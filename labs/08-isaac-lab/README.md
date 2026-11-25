# Lab 08 — Isaac Lab (GPU)

> **In one sentence:** the same robot as lab 07, but simulated thousands at a time on the RTX 4060 Ti — to see what
> GPU-parallel training changes compared with MuJoCo on the CPU.

**Status:** 🟡 step 1 done: robot model loads and behaves. Next: validate with the firmware LQR, then train.

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

## Next

2. The firmware LQR in Isaac must match the real robot and MuJoCo (same table as lab 07).
3. Train with thousands of robots in parallel; compare with v3/v4/v5.
