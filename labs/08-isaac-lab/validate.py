#!/usr/bin/env python3
"""Step 2: the firmware LQR in Newton (MuJoCo Warp, GPU) — does it behave like the real robot and like lab 07?

Same loop as labs/07-mujoco-rl/sim.py: physics 1 ms, control 5 ms, the firmware's complementary filter, 1-tick sensor
delay, gyro noise, the calibrated random pitch torque ("dirt"). The difference: every scenario runs at once, one robot
per parallel world.

    python labs/08-isaac-lab/validate.py           → report next to lab 07 (MuJoCo CPU) and the real robot
    python labs/08-isaac-lab/validate.py --viz     → same run in a window, real time (WSLg)
"""
import math
import os
import sys
import time

import numpy as np
import warp as wp

import newton

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.insert(0, os.path.join(ROOT, "model"))
from build_mjcf import load_params  # noqa: E402

P = load_params()
DT, SUB = P["control"]["dt"], 5            # 5 ms control, 1 ms physics (as lab 07)
R, G = P["wheel"]["radius"], 9.81
STILL, MOVES, PUSHES = 8, 4, (2, 4, 6, 8, 10, 14)


class FirmwareLQR:
    """firmware velocity.cpp _updateLqr + balance.cpp, vectorized over n robots (port of lab 07's FirmwareLQR)."""
    KT, KW, KX, KV = 0.744, 0.0707, 27.67, 14.04
    MVA, MVS = 0.1, 0.15

    def __init__(self, n):
        self.wheel, self.x, self.xref = np.zeros(n), np.zeros(n), np.zeros(n)
        self.goal = np.zeros(n)                       # move: 0 = none; p = distance done along the profile
        self.x0, self.p, self.v = np.zeros(n), np.zeros(n), np.zeros(n)

    def start_move(self, mask, goal):
        self.goal[mask], self.x0[mask], self.p[mask], self.v[mask] = goal, self.x[mask], 0.0, 0.0

    def step(self, pitch_deg, rate_dps):
        self.x += self.wheel * DT
        on = self.goal != 0
        g, s = np.abs(self.goal), np.sign(self.goal)
        vn = np.minimum(np.minimum(self.v + self.MVA * DT, self.MVS), np.sqrt(2 * self.MVA * np.maximum(g - self.p, 0)))
        ar = np.where(on, s * (vn - self.v) / DT, 0.0)
        vr = np.where(on, s * vn, 0.0)
        self.p = np.where(on, np.minimum(self.p + np.maximum(vn, 0.005) * DT, g), 0.0)
        self.v = np.where(on, vn, 0.0)
        self.xref = np.where(on, self.x0 + s * self.p, self.xref)
        self.goal = np.where(on & (self.p >= g), 0.0, self.goal)
        slip = min(0.3, 2.5 / self.KX)
        ex = self.x - self.xref
        far = np.abs(ex) > slip
        self.xref = np.where(far, self.x - np.sign(ex) * slip, self.xref)
        ex = np.clip(ex, -slip, slip)
        extra = np.clip(self.KX * ex + self.KV * (self.wheel - vr) + ar, -5, 5)
        a = np.clip(self.KT * (pitch_deg - np.degrees(ar / G)) + self.KW * rate_dps + extra, -4, 4)
        self.wheel = np.clip(self.wheel + a * DT, -P["motor"]["max_speed"], P["motor"]["max_speed"])
        return self.wheel


def quat_rotate_inv(q, v):
    """Rotate world vectors v into each body's frame (q = x, y, z, w)."""
    u, w = -q[:, :3], q[:, 3:4]
    return v + 2 * w * np.cross(u, v) + 2 * np.cross(u, np.cross(u, v))


class World:
    def __init__(self, n, seed=0):
        robot = newton.ModelBuilder()
        newton.solvers.SolverMuJoCo.register_custom_attributes(robot)
        robot.add_usd(os.path.join(ROOT, "model", "sbr.usda"), floating=True, enable_self_collisions=False)
        for k in (6, 7):                                          # wheel dofs: rotor inertia + bearing drag, as MJCF
            robot.joint_armature[k] = P["motor"]["rotor_inertia"]
            if hasattr(robot, "joint_damping"):
                robot.joint_damping[k] = 0.0005
        robot.joint_q[2] = R
        robot.add_ground_plane()
        b = newton.ModelBuilder()
        newton.solvers.SolverMuJoCo.register_custom_attributes(b)
        b.replicate(robot, n, spacing=(0.0, 0.5, 0.0))
        self.m = b.finalize()
        self.solver = newton.solvers.SolverMuJoCo(self.m)
        self.s0, self.s1, self.c = self.m.state(), self.m.state(), self.m.control()
        newton.eval_fk(self.m, self.m.joint_q, self.m.joint_qd, self.s0)
        self.n, self.rng = n, np.random.default_rng(seed)
        self.qd_target = np.zeros((n, 8), np.float32)
        self.force = np.zeros((n, 3, 6), np.float32)
        self.push_f, self.push_until = np.zeros(n), np.zeros(n)
        self.t = 0.0
        q, _ = self.body()
        self.pitch = self.true_pitch(q)
        self.delayed = [(self.pitch.copy(), np.zeros(n))]          # 1-tick sensor delay
        self.v_prev = None

    def body(self):
        return self.s0.body_q.numpy()[0::3], self.s0.body_qd.numpy()[0::3]

    @staticmethod
    def true_pitch(q):
        x, y, z, w = q[:, 3], q[:, 4], q[:, 5], q[:, 6]
        return np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))

    def sensors(self, q, qd, acc_world):
        rate = np.degrees(quat_rotate_inv(q[:, 3:], qd[:, 3:])[:, 1]) + self.rng.normal(0, 0.3, self.n)
        acc = quat_rotate_inv(q[:, 3:], acc_world - np.array([0, 0, -G]))       # accelerometer = a − g, body frame
        acc_angle = np.degrees(np.arctan2(-acc[:, 0], acc[:, 2]))
        self.pitch = 0.995 * (self.pitch + rate * DT) + 0.005 * acc_angle       # firmware filter
        return self.pitch.copy(), rate

    def step(self, ctrl):
        q, qd = self.body()
        acc = np.zeros((self.n, 3)) if self.v_prev is None else (qd[:, :3] - self.v_prev) / (DT / SUB)
        meas = self.delayed.pop(0)
        self.delayed.append(self.sensors(q, qd, acc))
        wheel = ctrl.step(*meas)
        self.qd_target[:, 6] = self.qd_target[:, 7] = wheel / R
        self.c.joint_target_qd.assign(self.qd_target.reshape(-1))
        self.force[:] = 0
        self.force[:, 0, 0] = np.where(self.t < self.push_until, self.push_f, 0.0)
        self.force[:, 0, 4] = self.rng.normal(0, P["disturbance_torque"], self.n)   # dirt: random pitch torque
        for k in range(SUB):
            self.s0.body_f.assign(self.force.reshape(-1, 6))
            if k == SUB - 1:
                self.v_prev = self.s0.body_qd.numpy()[0::3, :3].copy()
            self.solver.step(self.s0, self.s1, self.c, None, DT / SUB)
            self.s0, self.s1 = self.s1, self.s0
        self.t += DT
        return q[:, 0], self.true_pitch(q)


def run(viz=False, T=20.0):
    n = STILL + MOVES + len(PUSHES)
    w, ctrl = World(n), FirmwareLQR(n)
    moving = np.arange(n) >= STILL
    moving[STILL + MOVES:] = False
    pushes = np.arange(STILL + MOVES, n)
    viewer = None
    if viz:
        from newton.viewer import ViewerGL
        viewer = ViewerGL()
        viewer.set_model(w.m)
    xs, ths, fell = [], [], np.zeros(n, bool)
    t0 = time.time()
    for i in range(int(T / DT)):
        if abs(w.t - 1.0) < DT / 2:
            ctrl.start_move(moving, 0.5)
            w.push_f[pushes], w.push_until[pushes] = PUSHES, w.t + 0.1
        x, th = w.step(ctrl)
        xs.append(x)
        ths.append(th)
        fell |= np.abs(th) > 45
        if viewer is not None:
            if not viewer.is_running():
                return
            if i % 3 == 0:
                viewer.begin_frame(w.t)
                viewer.log_state(w.s0)
                viewer.end_frame()
            time.sleep(max(0, w.t - (time.time() - t0)))
    print(f"{n} robots × {T:.0f} s simulated in {time.time() - t0:.1f} s\n")
    xs, ths = np.array(xs), np.array(ths)
    sel = slice(int(5 / DT), None)
    st = range(STILL)
    ok = [k for k in st if not fell[k]]
    x_rng = [np.ptp(xs[sel, k]) * 50 for k in ok]
    jit = [ths[sel, k].std() for k in ok]
    print(f"standing still ({len(ok)}/{STILL} stand): x ±{min(x_rng):.1f}–{max(x_rng):.1f} cm, "
          f"pitch ±{min(jit):.2f}–{max(jit):.2f}°")
    for k in range(STILL, STILL + MOVES):
        if fell[k]:
            print("  move 50: FELL")
            continue
        x = xs[:, k] - xs[0, k]
        far = np.where(np.abs(x - 0.5) > 0.03)[0]
        print(f"move 50: overshoot {max(0, (x.max() - 0.5) * 100):.1f} cm, within 3 cm from "
              f"{(far[-1] * DT if len(far) else 0) - 1:.1f} s, ends {abs(x[-1] - 0.5) * 100:.1f} cm off")
    held = [f for f, k in zip(PUSHES, pushes) if not fell[k]]
    print(f"push 0.1 s: survives up to {max(held) if held else 0} N  (held: {held})")
    print("\nlab 07 MuJoCo CPU: still ±0.5–0.7 cm ±0.15°, move 50 1.4 cm in 4.0 s, push 6 N"
          "\nreal robot:        still ±0.3–1 cm ~±0.2°, move 50 ~1 cm in 4.8 s, LQR survives harder pushes than PID")


if __name__ == "__main__":
    run(viz="--viz" in sys.argv)
