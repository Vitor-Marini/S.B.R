#!/usr/bin/env python3
"""Load model/sbr.usda in Newton (MuJoCo Warp on the GPU) and check it; optionally watch it.

    python labs/08-isaac-lab/model_check.py          → headless checks: masses from params.json, falls and rests on
                                                      its body/battery box (not through the floor)
    python labs/08-isaac-lab/model_check.py --viz    → window: 16 robots dropped at random tilts, restart every 3 s

Build the USD first: python model/build_usd.py
"""
import math
import os
import sys

import numpy as np
import warp as wp

import newton

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.insert(0, os.path.join(ROOT, "model"))
from build_mjcf import load_params  # noqa: E402

N, DT = 16, 0.001


def pitch_deg(q):
    """Forward lean of each body from its quaternion (x, y, z, w): angle of the body z axis in the x-z plane."""
    qx, qy, qz, qw = q[:, 3], q[:, 4], q[:, 5], q[:, 6]
    return np.degrees(np.arctan2(2 * (qx * qz + qw * qy), 1 - 2 * (qx * qx + qy * qy)))


def build(tilts):
    """One robot per world (how Isaac Lab runs thousands in parallel: worlds never touch), each with its own tilt."""
    robot = newton.ModelBuilder()
    newton.solvers.SolverMuJoCo.register_custom_attributes(robot)
    robot.add_usd(os.path.join(ROOT, "model", "sbr.usda"), floating=True, enable_self_collisions=False)
    robot.add_ground_plane()
    b = newton.ModelBuilder()
    newton.solvers.SolverMuJoCo.register_custom_attributes(b)
    b.replicate(robot, len(tilts), spacing=(0.0, 0.4, 0.0))
    q = b.joint_q
    nq = len(q) // len(tilts)                         # free joint (x y z qx qy qz qw) + 2 wheels
    r = load_params()["wheel"]["radius"]
    for i, t in enumerate(tilts):
        h = math.radians(t) / 2
        q[i * nq + 2] = r
        q[i * nq + 3:i * nq + 7] = [0.0, math.sin(h), 0.0, math.cos(h)]
    return b.finalize()


def main(viz):
    tilts = np.random.default_rng(0).uniform(-10, 10, N)
    m = build(tilts)
    p = load_params()
    mass = m.body_mass.numpy()
    assert abs(mass[0] - p["body"]["mass"]) < 1e-6 and abs(mass[1] - p["wheel"]["mass"]) < 1e-6, mass
    solver = newton.solvers.SolverMuJoCo(m)
    s0, s1, c = m.state(), m.state(), m.control()
    newton.eval_fk(m, m.joint_q, m.joint_qd, s0)
    q0 = s0.body_q.numpy().copy()
    if viz:
        from newton.viewer import ViewerGL
        viewer = ViewerGL()
        viewer.set_model(m)
        k = 0
        while viewer.is_running():
            for _ in range(int(1 / 60 / DT)):
                solver.step(s0, s1, c, None, DT)
                s0, s1 = s1, s0
            k += 1
            if k % 180 == 0:                                      # every 3 s: drop them again
                s0.body_q.assign(q0)
                s0.body_qd.zero_()
                newton.eval_fk(m, m.joint_q, m.joint_qd, s0)
            viewer.begin_frame(k / 60)
            viewer.log_state(s0)
            viewer.end_frame()
        return
    for _ in range(3000):
        solver.step(s0, s1, c, None, DT)
        s0, s1 = s1, s0
    q = s0.body_q.numpy()
    bodies = q[0::3]                                              # body, wheel_left, wheel_right per robot
    lean = pitch_deg(bodies)
    print(f"after 3 s: lean {np.abs(lean).min():.1f}–{np.abs(lean).max():.1f}°, "
          f"axle height {bodies[:, 2].min() * 1000:.1f}–{bodies[:, 2].max() * 1000:.1f} mm")
    assert np.all(np.sign(lean) == np.sign(tilts)), "each robot must fall the way it was tilted"
    assert np.all(np.abs(lean) > 60) and np.all(bodies[:, 2] > 0.03), "must rest on its body/battery, above the floor"
    print("model ok")


if __name__ == "__main__":
    main("--viz" in sys.argv)
