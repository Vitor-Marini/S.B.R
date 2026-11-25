#!/usr/bin/env python3
"""S.B.R in MuJoCo, driven by the SAME controllers as the firmware (LQR and PD + cascade).

Step 1 of lab 07: before training anything, check the simulator behaves like the real robot.

    .venv/bin/python labs/07-mujoco-rl/sim.py              → scenarios (still, pushes, move 50) for LQR and PID,
                                                            next to the numbers measured on the real robot
    .venv/bin/python labs/07-mujoco-rl/sim.py view [pid]   → 3D viewer in real time; keys: P push, M move +50 cm,
                                                            N move −50 cm (the viewer's own keys still work)
    .venv/bin/python labs/07-mujoco-rl/sim.py selftest
"""
import math
import os
import sys
import time

import mujoco
import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.insert(0, os.path.join(ROOT, "model"))
sys.path.insert(0, os.path.join(ROOT, "labs", "03-lqr"))
import build_mjcf  # noqa: E402
from lqr import Cascade  # noqa: E402  (port of the firmware cascade, already used to validate LQR)

P = build_mjcf.load_params()
DT = P["control"]["dt"]
R = P["wheel"]["radius"]
G = 9.81


class FirmwareLQR:
    """firmware/src/velocity.cpp _updateLqr + balance.cpp, same gains and limits."""
    KT, KW, KX, KV = 0.744, 0.0707, 27.67, 14.04
    MVA, MVS = 0.1, 0.15

    def __init__(self):
        self.wheel = self.x = self.xref = 0.0
        self.move = None                       # [goal, start_x, ref_p, ref_v]

    def start_move(self, goal):
        self.move = [goal, self.x, 0.0, 0.0]

    def step(self, pitch_deg, rate_dps):
        self.x += self.wheel * DT
        vr = ar = 0.0
        if self.move:
            goal, x0, p, v = self.move
            g, s = abs(goal), math.copysign(1, goal)
            vn = min(v + self.MVA * DT, self.MVS, math.sqrt(2 * self.MVA * max(g - p, 0)))
            ar, vr = s * (vn - v) / DT, s * vn
            p = min(p + max(vn, 0.005) * DT, g)
            self.xref = x0 + s * p
            self.move = None if p >= g else [goal, x0, p, vn]
        else:
            self.xref += vr * DT
        slip = min(0.3, 2.5 / self.KX)
        ex = self.x - self.xref
        if abs(ex) > slip:
            self.xref, ex = self.x - math.copysign(slip, ex), math.copysign(slip, ex)
        extra = max(-5, min(5, self.KX * ex + self.KV * (self.wheel - vr) + ar))
        theta_ref = math.degrees(ar / G)
        a = max(-4, min(4, self.KT * (pitch_deg - theta_ref) + self.KW * rate_dps + extra))
        self.wheel = max(-P["motor"]["max_speed"], min(P["motor"]["max_speed"], self.wheel + a * DT))
        return self.wheel


class Sim:
    """Physics at 1 ms, control at 5 ms with the firmware's complementary filter and a 1-tick sensor delay."""

    def __init__(self, controller, gyro_noise=0.3, seed=0, pitch0=0.0, disturbance=None, params=None, delay=1,
                 act_delay=0, slope_deg=0.0):
        params = params or P                       # a different robot (domain randomization) or the real one
        self.m = mujoco.MjModel.from_xml_string(build_mjcf.build(params))
        if slope_deg:                              # ramp along x = gravity tilted (simpler than a tilted floor)
            s_ = math.radians(slope_deg)
            self.m.opt.gravity[:] = [-G * math.sin(s_), 0, -G * math.cos(s_)]
        self.d = mujoco.MjData(self.m)
        if pitch0:
            q = np.zeros(4)
            mujoco.mju_axisAngle2Quat(q, np.array([0, 1.0, 0]), math.radians(pitch0))
            self.d.qpos[3:7] = q
        mujoco.mj_forward(self.m, self.d)
        self.c = controller
        self.rng = np.random.default_rng(seed)
        self.noise = gyro_noise
        # random pitch torque on the body (motor vibration, backlash, uneven floor): the real robot's "dirt"
        self.dist = params["disturbance_torque"] if disturbance is None else disturbance
        self.sub = int(round(DT / self.m.opt.timestep))
        self.pitch = self.true_pitch()
        self.delayed = [(self.pitch, 0.0)] * delay      # sensor → controller, in control ticks
        self.act_queue = [(0.0, 0.0)] * act_delay      # controller → wheels (driver/motor lag), in control ticks
        self._bump = None
        self.t = 0.0
        self.body = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "robot")

    def true_pitch(self):
        w, x, y, z = self.d.qpos[3:7]
        return math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x)))))

    def sensors(self):
        gyro = self.d.sensordata[0:3]
        acc = self.d.sensordata[3:6]
        rate = math.degrees(gyro[1]) + self.rng.normal(0, self.noise)
        acc_angle = math.degrees(math.atan2(-acc[0], acc[2]))
        self.pitch = 0.995 * (self.pitch + rate * DT) + 0.005 * acc_angle       # firmware filter
        return self.pitch, rate

    def push(self, newtons, seconds=0.1):
        self._push = (newtons, self.t + seconds)

    def bump(self, torque, seconds):
        """Tile grout: the wheels meet a resisting torque for a moment (and the stepper may not follow)."""
        self._bump = (torque, self.t + seconds)

    def step(self):
        meas = self.delayed.pop(0)
        self.delayed.append(self.sensors())
        wheel = self.c.step(*meas)
        turn = getattr(self.c, "turn_offset", 0.0)    # m/s per wheel, + = left (firmware: L = v − t, R = v + t)
        self.act_queue.append((wheel - turn, wheel + turn))
        left, right = self.act_queue.pop(0)
        self.d.ctrl[:] = (left / R, right / R)
        b = self._bump
        if b and self.t < b[1]:
            self.d.qfrc_applied[6:8] = -np.sign(self.d.qvel[6:8]) * b[0]
        else:
            self.d.qfrc_applied[6:8] = 0.0
            self._bump = None
        f = getattr(self, "_push", None)
        self.d.xfrc_applied[self.body, 0] = f[0] if f and self.t < f[1] else 0.0
        self.d.xfrc_applied[self.body, 4] = self.rng.normal(0, self.dist) if self.dist else 0.0
        if f and self.t >= f[1]:
            self._push = None
        for _ in range(self.sub):
            mujoco.mj_step(self.m, self.d)
        self.t += DT
        return self.d.qpos[0], self.true_pitch()

    def fell(self):
        return abs(self.true_pitch()) > 45


def run(ctrl_cls, T, push=None, move=None, **kw):
    s = Sim(ctrl_cls(), **kw)
    xs, ths = [], []
    for i in range(int(T / DT)):
        if push and abs(s.t - push[0]) < DT / 2:
            s.push(push[1])
        if move and abs(s.t - move[0]) < DT / 2:
            s.c.start_move(move[1])
        x, th = s.step()
        xs.append(x)
        ths.append(th)
        if s.fell():
            return None
    return np.array(xs), np.array(ths)


def report():
    print(f"model: L {P['body']['pendulum_length_L']} m, torque limit {P['motor']['torque_limit']} N·m (guess), "
          f"floor friction {P['floor_friction']}\n")
    for name, cls in (("PID cascade", Cascade), ("LQR", FirmwareLQR)):
        print(name)
        r = run(cls, 20)
        if r is None:
            print("  standing still: FELL")
        else:
            x, th = r
            sel = slice(int(5 / DT), None)
            print(f"  standing still: x {x[sel].min()*100:+.1f}..{x[sel].max()*100:+.1f} cm, pitch ±{th[sel].std():.2f}°")
        r = run(cls, 20, move=(1.0, 0.5))
        if r is None:
            print("  move 50: FELL")
        else:
            x, _ = r
            x = x - x[0]
            far = np.where(abs(x - 0.5) > 0.03)[0]
            print(f"  move 50: overshoot {max(0, (x.max() - 0.5) * 100):.1f} cm, within 3 cm from "
                  f"{(far[-1] * DT if len(far) else 0) - 1:.1f} s, ends {abs(x[-1] - 0.5) * 100:.1f} cm off")
        held = [f for f in (2, 4, 6, 8, 10, 14) if run(cls, 6, push=(1.0, f)) is not None]
        print(f"  push 0.1 s at the top: survives up to {max(held) if held else 0} N")
    print("\nreal robot (journal): PID still ±2.5–4 cm, move 50 overshoot 3–15 cm in 6–25 s;"
          " LQR still ±0.3–1 cm, move 50 ~1 cm in 4.8 s")


def view(ctrl_cls):
    import mujoco.viewer
    s = Sim(ctrl_cls())
    keys = {}
    with mujoco.viewer.launch_passive(s.m, s.d, key_callback=lambda k: keys.update(k=k)) as v:
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 1.2, -15, 120
        while v.is_running():
            t0 = time.time()
            k = keys.pop("k", None)
            if k == ord("P"):
                s.push(6)
            elif k == ord("M"):
                s.c.start_move(0.5)
            elif k == ord("N"):
                s.c.start_move(-0.5)
            s.step()
            v.sync()
            time.sleep(max(0, DT - (time.time() - t0)))


def _selftest():
    s = Sim(FirmwareLQR(), gyro_noise=0, pitch0=5.0)    # sign check: 5° forward must read +5°
    assert abs(s.pitch - 5.0) < 0.2, s.pitch
    for _ in range(20):                                 # 20 ms resting on the wheels (at t=0 it's in free fall)
        mujoco.mj_step(s.m, s.d)
    acc = s.d.sensordata[3:6]
    # same sign, smaller: falling, the IMU point also accelerates (docs/hardware.md, "Where to mount the IMU")
    assert 1.0 < math.degrees(math.atan2(-acc[0], acc[2])) < s.true_pitch(), "accelerometer sign"
    assert run(FirmwareLQR, 3) is not None, "LQR must stand"
    print("selftest ok")


if __name__ == "__main__":
    a = sys.argv[1:]
    if a[:1] == ["selftest"]:
        _selftest()
    elif a[:1] == ["view"]:
        view(Cascade if a[1:] == ["pid"] else FirmwareLQR)
    else:
        report()
