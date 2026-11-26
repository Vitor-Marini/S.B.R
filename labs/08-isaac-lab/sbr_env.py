#!/usr/bin/env python3
"""v6 environment on the GPU: lab 07's env.py (v5) vectorized over thousands of robots in Newton, + the v6 reward.

Everything that lab 07 did per robot in Python is a tensor here: the firmware's reference generator (joystick ramp,
move profile, turn ramp), the complementary filter, sensor/actuator delays, the "day of use" events (scenarios,
pushes, grout, drifting setpoint error), the robot randomization (L, mass, torque, friction, dirt, slope) and the
reward. Observation and action are identical to v5 (24 inputs, wheel acceleration ±4 m/s²), so a policy trained here
runs in lab 07's battery — and on the same firmware path.

v6 = v5 with W_TH 4° → 0.5° (lab 07, step 5: v5 learned a lazy 1.5 Hz ±2.3° sway because tilt was too cheap).
v7 (residual=0.25): action = firmware LQR + network × ±1 m/s². The robot starts as the LQR and the network only learns
corrections; the effort penalty applies to the correction only (the LQR itself is saturated 45 % of the time). The
critic also sees what the robot can't (true tilt, setpoint error, the randomized physics): the "critic" obs group.
v8 = v7 + drift cost: distance from the commanded path WITHOUT the slip limit (ex is clamped at 9 cm and xref slides,
so in v7 creeping 47 cm cost the same as 9 cm — the one battery line v7 failed).

    python labs/08-isaac-lab/sbr_env.py     → selftest: LQR survives, zero action falls, speed
"""
import math
import os
import sys
import time

import numpy as np
import torch
import warp as wp

import newton

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.insert(0, os.path.join(ROOT, "model"))
from build_mjcf import load_params  # noqa: E402

P = load_params()
DT, SUB = P["control"]["dt"], 5
R, G = P["wheel"]["radius"], 9.81
VMAX = P["motor"]["max_speed"]
A_MAX, EPISODE_S, FALL_DEG, SLIP = 4.0, 15.0, 30.0, 0.09          # same constants as lab 07 env.py
V_RAMP, W_RAMP, MOVE_A, MOVE_V, TRACK = 0.6, 240.0, 0.1, 0.15, 0.150
HISTORY, FRAME = 3, 8
W_X, W_V, W_W, W_A, W_JERK = 0.03, 0.4, 50.0, 1.5, 0.5
W_TH_START, W_TH_END = 4.0, 0.5   # v6: tilt weight tightened 4° (v5) → 0.5° along the curriculum. Fixed at 0.5° from
                                  # the start, any tilt > 3° saturates the state cost: a robot that can't balance yet
                                  # only learns 'use less action' and falls (measured: 0.3 s episodes after 300M)
COST_SCALE = 40.0
W_DRIFT = 0.15               # v8: m off the commanded path (∫vr dt, no slip reset); LQR with a 2.75° setpoint ≈ 7 cm
PHYS_EVERY = 20             # ticks between batched physics randomizations (see _refresh_physics)
RING = 4                     # delay buffers: sensor delay 1–3 ticks, actuator 0–2
NQ, NDOF, NBODY = 9, 8, 3    # per robot: free joint (7) + 2 wheels; bodies: body, wheel_left, wheel_right


def rotate_inv(q, v):
    """World vectors → body frame (q = x, y, z, w)."""
    u, w = -q[:, :3], q[:, 3:4]
    t = 2 * torch.cross(u, v, dim=1)
    return v + w * t + torch.cross(u, t, dim=1)


class SbrEnv:
    def __init__(self, num_envs=4096, device="cuda:0", seed=0, difficulty=0.2, curriculum_ticks=0, d_max=0.8,
                 w_th=(W_TH_START, W_TH_END), residual=0.0):
        self.num_envs, self.num_actions, self.device = num_envs, 1, device
        self.max_episode_length = int(EPISODE_S / DT)
        self.cfg = {}
        self.difficulty, self.curriculum_ticks, self.d_max, self.ticks = difficulty, curriculum_ticks, d_max, 0
        self.w_th_range, self.residual = w_th, residual
        self.w_th = w_th[0] if curriculum_ticks else w_th[1]
        self.g = torch.Generator(device=device).manual_seed(seed)
        self._build_world()
        n, z = num_envs, lambda *s: torch.zeros(*s, device=device)
        self.episode_length_buf = torch.zeros(n, dtype=torch.long, device=device)
        # controller / reference state (lab 07: PolicyController + Reference)
        for k in ("wheel", "x", "ex", "action", "prev_action", "bias", "bias_drift", "filt", "pitch_m", "rate_m",
                  "v_target", "w_target", "vr", "ar", "xref", "wr", "goal", "x0", "p", "v", "next_cmd",
                  "push_f", "push_until", "bump_tq", "bump_until", "noise", "dist", "slope", "mass_scale", "l_scale", "res", "xfree"):
            setattr(self, k, z(n))
        self.scenario = torch.zeros(n, dtype=torch.long, device=device)   # 0 still, 1 joystick, 2 moves, 3 mixed
        self.delay = torch.ones(n, dtype=torch.long, device=device)
        self.act_delay = torch.zeros(n, dtype=torch.long, device=device)
        self.sring, self.aring = z(RING, n, 2), z(RING, n, 2)
        self.hist = z(n, HISTORY, FRAME)
        self.ar_idx = torch.arange(n, device=device)
        self.k = 0
        self.ep_ret = z(n)
        self.phys_pending = torch.zeros(n, dtype=torch.bool, device=device)
        self.reset(torch.ones(n, dtype=torch.bool, device=device))
        self._refresh_physics()

    def _refresh_physics(self):
        # shortcut: new robot physics for reset envs is applied in batches every PHYS_EVERY ticks (≤ 0.1 s into the
        # episode) — telling the solver costs ~9 ms per call, per reset it halved the speed
        ids = self.phys_pending.nonzero().squeeze(1)
        if len(ids):
            self._randomize_physics(ids, self.difficulty)
            self.phys_pending[ids] = False

    # ---------------------------------------------------------------- physics (Newton, MuJoCo Warp)
    def _build_world(self):
        robot = newton.ModelBuilder()
        newton.solvers.SolverMuJoCo.register_custom_attributes(robot)
        robot.add_usd(os.path.join(ROOT, "model", "sbr.usda"), floating=True, enable_self_collisions=False)
        for k in (6, 7):
            robot.joint_armature[k] = P["motor"]["rotor_inertia"]
            if hasattr(robot, "joint_damping"):
                robot.joint_damping[k] = 0.0005
        robot.joint_q[2] = R
        robot.add_ground_plane()
        b = newton.ModelBuilder()
        newton.solvers.SolverMuJoCo.register_custom_attributes(b)
        b.replicate(robot, self.num_envs, spacing=(0.0, 0.5, 0.0))
        m = self.m = b.finalize()
        self.solver = newton.solvers.SolverMuJoCo(m)
        self.s0, self.s1, self.c = m.state(), m.state(), m.control()
        newton.eval_fk(m, m.joint_q, m.joint_qd, self.s0)
        n, t = self.num_envs, wp.to_torch
        self.nshape = m.shape_count // n
        self.jq, self.jqd = t(self.s0.joint_q).view(n, NQ), t(self.s0.joint_qd).view(n, NDOF)
        self.bq, self.bqd = t(self.s0.body_q).view(n, NBODY, 7), t(self.s0.body_qd).view(n, NBODY, 6)
        self.f0, self.f1 = t(self.s0.body_f).view(n, NBODY, 6), t(self.s1.body_f).view(n, NBODY, 6)
        self.target_qd, self.jf = t(self.c.joint_target_qd).view(n, NDOF), t(self.c.joint_f).view(n, NDOF)
        self.vprev_wp = wp.zeros_like(self.s0.body_qd)
        self.vprev = t(self.vprev_wp).view(n, NBODY, 6)
        self.mass, self.inv_mass = t(m.body_mass).view(n, NBODY), t(m.body_inv_mass).view(n, NBODY)
        self.inertia, self.inv_inertia = t(m.body_inertia).view(n, NBODY, 3, 3), t(m.body_inv_inertia).view(n, NBODY, 3, 3)
        self.effort = t(m.joint_effort_limit).view(n, NDOF)
        self.mu = t(m.shape_material_mu).view(n, self.nshape)
        self.home = self.jq[:, :2].clone()                       # each world's x, y (replicate spacing)
        self.m0 = self.mass.clone()
        self.reset_mask_wp = wp.zeros(n, dtype=bool)             # eval_fk mask: one per articulation (robot)
        self.fk_mask = wp.to_torch(self.reset_mask_wp)
        self._substeps()                                         # compile kernels, then record the CUDA graph
        with wp.ScopedCapture() as cap:
            self._substeps()
        self.graph = cap.graph

    def _substeps(self):
        a, b = self.s0, self.s1
        for k in range(SUB):
            if k == SUB - 1:
                wp.copy(self.vprev_wp, a.body_qd)
            self.solver.step(a, b, self.c, None, DT / SUB)
            a, b = b, a
        for name in ("joint_q", "joint_qd", "body_q", "body_qd"):   # odd count: result is in s1 → back into s0
            wp.copy(getattr(self.s0, name), getattr(a, name))

    def _randomize_physics(self, ids, d):
        u = lambda lo, hi: lo + (hi - lo) * torch.rand(len(ids), device=self.device, generator=self.g)
        b, w = P["body"], P["wheel"]
        ms = u(1 - 0.1 * d, 1 + 0.1 * d)
        L = b["pendulum_length_L"] * u(1 - 0.2 * d, 1 + 0.2 * d)
        mass, dz = b["mass"] * ms, b["com_height"]
        sx, sy, sz = b["size"]
        iyy = L * mass * dz - mass * dz * dz
        izz = mass * (sx ** 2 + sy ** 2) / 12
        ixx = torch.minimum(mass * (sy ** 2 + sz ** 2) / 12, iyy + izz)
        I = torch.zeros(len(ids), 3, 3, device=self.device)
        I[:, 0, 0], I[:, 1, 1], I[:, 2, 2] = ixx, iyy, izz
        self.mass[ids, 0], self.inv_mass[ids, 0] = mass, 1 / mass
        self.inertia[ids, 0], self.inv_inertia[ids, 0] = I, torch.linalg.inv(I)
        self.mass_scale[ids], self.l_scale[ids] = ms, L / b["pendulum_length_L"]
        self.effort[ids, 6] = self.effort[ids, 7] = u(0.3 - 0.1 * d, 0.3 + 0.1 * d)
        self.mu[ids] = u(0.8 - 0.3 * d, 0.8 + 0.2 * d)[:, None]
        self.solver.notify_model_changed(newton.ModelFlags.BODY_INERTIAL_PROPERTIES
                                         | newton.ModelFlags.JOINT_DOF_PROPERTIES | newton.ModelFlags.SHAPE_PROPERTIES)

    # ---------------------------------------------------------------- episodes
    def _rand(self, n, lo=0.0, hi=1.0):
        return lo + (hi - lo) * torch.rand(n, device=self.device, generator=self.g)

    def reset(self, mask):
        ids = mask.nonzero().squeeze(1)
        n = len(ids)
        if n == 0:
            return
        d = self.difficulty
        self.scenario[ids] = torch.randint(0, 4, (n,), device=self.device, generator=self.g)
        self.phys_pending[ids] = True
        self.dist[ids] = self._rand(n, 0.0, 0.015 * max(d, 0.5))
        self.bias[ids] = self._rand(n, -3, 3) * d
        self.bias_drift[ids] = self._rand(n, 0, 0.3) * d
        self.noise[ids] = self._rand(n, 0.1, 0.3 + 0.3 * d)
        self.delay[ids] = torch.randint(1, 2 + round(2 * d), (n,), device=self.device, generator=self.g)
        self.act_delay[ids] = torch.randint(0, 1 + round(2 * d), (n,), device=self.device, generator=self.g)
        self.slope[ids] = torch.deg2rad(self._rand(n, -2, 2) * d)
        pitch0 = self._rand(n, -3, 3) * max(d, 0.3)
        h = torch.deg2rad(pitch0) / 2
        q = torch.zeros(n, NQ, device=self.device)
        q[:, :2], q[:, 2], q[:, 4], q[:, 6] = self.home[ids], R, torch.sin(h), torch.cos(h)
        self.jq[ids], self.jqd[ids] = q, 0.0
        self.fk_mask.zero_()
        self.fk_mask[ids] = True
        newton.eval_fk(self.m, self.s0.joint_q, self.s0.joint_qd, self.s0, mask=self.reset_mask_wp)
        self.vprev[ids] = 0.0
        for k in ("wheel", "x", "ex", "action", "prev_action", "v_target", "w_target", "vr", "ar", "xref", "wr",
                  "goal", "x0", "p", "v", "next_cmd", "push_f", "push_until", "bump_tq", "bump_until", "rate_m", "res", "xfree"):
            getattr(self, k)[ids] = 0.0
        self.filt[ids] = pitch0
        self.pitch_m[ids] = pitch0 + self.bias[ids]
        self.sring[:, ids, 0], self.sring[:, ids, 1] = pitch0, 0.0
        self.aring[:, ids] = 0.0
        self.episode_length_buf[ids] = 0
        self.ep_ret[ids] = 0.0
        self.hist[ids] = self._frame()[ids, None, :]

    def _frame(self):
        theta_ref = torch.rad2deg(self.ar / G)
        return torch.stack([(self.pitch_m - theta_ref) / 10, self.rate_m / 100, (self.wheel - self.vr) / 0.5,
                            self.ex / 0.2, self.vr / 0.5, self.ar / 1.0, self.wr / 45, self.action], 1)

    def get_observations(self):
        from tensordict import TensorDict
        obs = self.hist.reshape(self.num_envs, -1).clamp(-10, 10)
        return TensorDict({"policy": obs, "critic": self._privileged()}, batch_size=[self.num_envs])

    def _privileged(self):
        """Training-only truth for the critic (never on the robot): tilt, rate, setpoint error, physics, events."""
        t = self.episode_length_buf * DT
        q, w = self.bq[:, 0, 3:], self.bqd[:, 0, 3:]
        return torch.stack([(self.true_pitch() - torch.rad2deg(self.ar / G)) / 10,
                            torch.rad2deg(rotate_inv(q, w)[:, 1]) / 100, self.bias / 3, torch.rad2deg(self.slope) / 2,
                            (self.mass_scale - 1) * 10, (self.l_scale - 1) * 5, (self.effort[:, 6] - 0.3) * 10,
                            (self.mu[:, 0] - 0.8) * 5, self.dist / 0.015,
                            torch.where(t < self.push_until, self.push_f, 0.0) / 6,
                            torch.where(t < self.bump_until, self.bump_tq, 0.0) / 0.25,
                            torch.full_like(t, self.difficulty), (self.x - self.xfree) / 0.2], 1).clamp(-10, 10)

    # ---------------------------------------------------------------- one control tick (5 ms)
    def _events(self):
        d, t, n = self.difficulty, self.episode_length_buf * DT, self.num_envs
        rnd = lambda: torch.rand(n, device=self.device, generator=self.g)
        joy = ((self.scenario == 1) | (self.scenario == 3)) & (t >= self.next_cmd)
        self.v_target = torch.where(joy, torch.where(rnd() < 0.25, 0.0, (rnd() * 0.8 - 0.4) * max(d, 0.25)),
                                    self.v_target)
        self.w_target = torch.where(joy, torch.where(rnd() < 0.4, (rnd() * 90 - 45) * d, 0.0), self.w_target)
        self.next_cmd = torch.where(joy, t + 0.5 + 2.5 * rnd(), self.next_cmd)
        mv = (self.scenario == 2) & (t >= self.next_cmd) & (self.goal == 0)
        start = mv & (rnd() < 0.5)
        sign = torch.where(rnd() < 0.5, -1.0, 1.0)
        self.goal = torch.where(start, sign * (0.2 + 0.4 * rnd()), self.goal)
        self.x0 = torch.where(start, self.x, self.x0)
        self.p, self.v = torch.where(start, 0.0, self.p), torch.where(start, 0.0, self.v)
        self.v_target = torch.where(start, 0.0, self.v_target)
        self.w_target = torch.where(start, 0.0, torch.where(mv, sign * 45 * max(d, 0.3), self.w_target))
        self.next_cmd = torch.where(mv, t + 2 + 2 * rnd(), self.next_cmd)
        stop = (self.scenario == 2) & (self.w_target != 0) & (t >= self.next_cmd - 1)
        self.w_target = torch.where(stop, 0.0, self.w_target)
        push = rnd() < d / 700
        self.push_f = torch.where(push, (rnd() * 12 - 6) * d, self.push_f)
        self.push_until = torch.where(push, t + 0.05 + 0.15 * rnd(), self.push_until)
        bump = (self.wheel.abs() > 0.05) & (rnd() < d / 300)
        self.bump_tq = torch.where(bump, (0.05 + 0.2 * rnd()) * d, self.bump_tq)
        self.bump_until = torch.where(bump, t + 0.02 + 0.06 * rnd(), self.bump_until)
        self.bias = (self.bias + torch.randn(n, device=self.device, generator=self.g) * self.bias_drift * math.sqrt(DT)
                     ).clamp(-3.5, 3.5)

    def _reference(self):
        on = self.goal != 0
        g, s = self.goal.abs(), torch.sign(self.goal)
        vn = torch.minimum(torch.minimum(self.v + MOVE_A * DT, torch.full_like(self.v, MOVE_V)),
                           torch.sqrt(2 * MOVE_A * (g - self.p).clamp(min=0)))
        p = torch.minimum(self.p + vn.clamp(min=0.005) * DT, g)
        done = on & (p >= g)
        vr_mv, ar_mv = torch.where(done, 0.0, s * vn), torch.where(done, 0.0, s * (vn - self.v) / DT)
        prev = self.vr
        vr_joy = self.vr + (self.v_target - self.vr).clamp(-V_RAMP * DT, V_RAMP * DT)
        self.vr = torch.where(on, vr_mv, vr_joy)
        self.ar = torch.where(on, ar_mv, (vr_joy - prev) / DT)
        self.xref = torch.where(on, self.x0 + s * p, self.xref + vr_joy * DT)
        self.p, self.v = torch.where(on, p, self.p), torch.where(on, vn, self.v)
        self.goal = torch.where(done, 0.0, self.goal)
        self.wr = self.wr + (self.w_target - self.wr).clamp(-W_RAMP * DT, W_RAMP * DT)
        self.xfree = self.xfree + self.vr * DT

    def true_pitch(self):
        q = self.bq[:, 0, 3:]
        return torch.rad2deg(torch.asin((2 * (q[:, 3] * q[:, 1] - q[:, 2] * q[:, 0])).clamp(-1, 1)))

    def step(self, actions):
        n, ar = self.num_envs, self.ar_idx
        a = actions.reshape(n).clamp(-1, 1)
        prev_res = self.res
        if self.residual:                       # v7: LQR on the same observation + bounded correction
            self.res = a * self.residual
            a = (lqr_from_obs(self.hist.reshape(n, -1).clamp(-10, 10)) + self.res).clamp(-1, 1)
        self.prev_action, self.action = self.action, a
        self._events()
        # sensors from the current state: gyro + accelerometer → firmware complementary filter
        q, qd = self.bq[:, 0, 3:], self.bqd[:, 0]
        acc = (qd[:, :3] - self.vprev[:, 0, :3]) / (DT / SUB)
        rate = torch.rad2deg(rotate_inv(q, qd[:, 3:])[:, 1]) + self.noise * torch.randn(n, device=self.device,
                                                                                         generator=self.g)
        acc_b = rotate_inv(q, acc - torch.tensor([0.0, 0.0, -G], device=self.device))
        self.filt = 0.995 * (self.filt + rate * DT) + 0.005 * torch.rad2deg(torch.atan2(-acc_b[:, 0], acc_b[:, 2]))
        self.sring[self.k % RING] = torch.stack([self.filt, rate], 1)
        meas = self.sring[(self.k - self.delay) % RING, ar]
        # controller (lab 07 PolicyController): reference, odometry, slip limit, integrate the acceleration
        self._reference()
        self.pitch_m, self.rate_m = meas[:, 0] + self.bias, meas[:, 1]
        self.x = self.x + self.wheel * DT
        ex = self.x - self.xref
        far = ex.abs() > SLIP
        self.xref = torch.where(far, self.x - torch.sign(ex) * SLIP, self.xref)
        self.ex = ex.clamp(-SLIP, SLIP)
        self.wheel = (self.wheel + a * A_MAX * DT).clamp(-VMAX, VMAX)
        turn = torch.deg2rad(self.wr) * TRACK / 2
        self.aring[self.k % RING] = torch.stack([self.wheel - turn, self.wheel + turn], 1)
        lr = self.aring[(self.k - self.act_delay) % RING, ar]
        self.target_qd[:, 6], self.target_qd[:, 7] = lr[:, 0] / R, lr[:, 1] / R
        # forces: push, dirt (random pitch torque), slope (gravity tilted along x), grout on the wheels
        t = self.episode_length_buf * DT
        f = torch.zeros(n, NBODY, 6, device=self.device)
        f[:, :, 0] = -self.mass * G * torch.sin(self.slope)[:, None]
        f[:, :, 2] = self.mass * G * (1 - torch.cos(self.slope))[:, None]
        f[:, 0, 0] += torch.where(t < self.push_until, self.push_f, 0.0)
        f[:, 0, 4] = self.dist * torch.randn(n, device=self.device, generator=self.g)
        self.f0.copy_(f)
        self.f1.copy_(f)
        bump = torch.where(t < self.bump_until, self.bump_tq, 0.0)[:, None]
        self.jf[:, 6:8] = -torch.sign(self.jqd[:, 6:8]) * bump
        wp.capture_launch(self.graph)
        self.k += 1
        self.ticks += 1
        if self.k % PHYS_EVERY == 0:
            self._refresh_physics()
        self.episode_length_buf += 1
        if self.curriculum_ticks:
            frac = min(1.0, self.ticks / self.curriculum_ticks)
            self.difficulty = 0.2 + (self.d_max - 0.2) * frac
            self.w_th = self.w_th_range[0] * (self.w_th_range[1] / self.w_th_range[0]) ** frac   # geometric 4° → 0.5°
        # observation, reward (v6), termination
        self.hist = torch.cat([self.hist[:, 1:], self._frame()[:, None]], 1)
        pitch = self.true_pitch()
        th = pitch - torch.rad2deg(self.ar / G)
        state = ((self.ex / W_X) ** 2 + ((self.wheel - self.vr) / W_V) ** 2 + (th / self.w_th) ** 2
                 + (self.rate_m / W_W) ** 2 + ((self.x - self.xfree) / W_DRIFT) ** 2 * (self.residual > 0))
        if self.residual:
            effort = (self.res * A_MAX / W_A) ** 2 + ((self.res - prev_res) * A_MAX / W_JERK) ** 2
        else:
            effort = (a * A_MAX / W_A) ** 2 + ((a - self.prev_action) * A_MAX / W_JERK) ** 2
        rew = 1.0 - (state / COST_SCALE).clamp(max=1.0) - effort / COST_SCALE
        fell = pitch.abs() > FALL_DEG
        timeout = self.episode_length_buf >= self.max_episode_length
        done = fell | timeout
        self.ep_ret += rew
        extras = {"time_outs": timeout, "log": {"difficulty": torch.tensor(self.difficulty), "w_th": torch.tensor(self.w_th),
                                                "fell_per_tick": fell.float().mean()}}
        self.reset(done)
        return self.get_observations(), rew, done, extras


def lqr_from_obs(o):
    """The firmware LQR (lab 07 lqr_policy) read off the newest observation frame; torch or numpy, (..., 24) → (...)."""
    f = o[..., -FRAME:]       # (pitch − θref)/10, rate/100, (wheel − vr)/0.5, ex/0.2, vr/0.5, ar, wr/45, action
    a = 7.44 * f[..., 0] + 7.07 * f[..., 1] + (5.534 * f[..., 3] + 7.02 * f[..., 2] + f[..., 5]).clip(-5, 5)
    return (a / A_MAX).clip(-1, 1)


def lqr_actions(env):
    """The firmware LQR as a batched action (plain env); in a residual env the LQR is already inside: send zeros."""
    if env.residual:
        return torch.zeros(env.num_envs, 1, device=env.device)
    return lqr_from_obs(env.get_observations()["policy"])[:, None]


def _selftest():
    for d in (0.0, 0.8):
        env = SbrEnv(num_envs=1024, difficulty=d, seed=1)
        alive = torch.ones(1024, dtype=torch.bool, device=env.device)
        env.episode_length_buf.zero_()
        t0 = time.time()
        for _ in range(int(10 / DT)):
            _, _, done, ex = env.step(lqr_actions(env))
            alive &= ~(done & ~ex["time_outs"])
        torch.cuda.synchronize()
        rate = 1024 * int(10 / DT) / (time.time() - t0)
        print(f"difficulty {d}: LQR survives 10 s in {alive.float().mean() * 100:.0f}% of 1024 robots  "
              f"({rate:,.0f} robot-steps/s)")
        assert alive.float().mean() > (0.97 if d == 0 else 0.7)
    env = SbrEnv(num_envs=64, difficulty=0.0)
    fallen = torch.zeros(64, dtype=torch.bool, device=env.device)
    for k in range(int(3 / DT)):
        _, _, done, _ = env.step(torch.zeros(64, 1, device=env.device))
        fallen |= done
        if fallen.all():
            break
    assert fallen.all(), "zero action must fall"
    env = SbrEnv(num_envs=1024, difficulty=0.8, seed=1, residual=0.25)        # residual + zero net = the LQR
    alive = torch.ones(1024, dtype=torch.bool, device=env.device)
    for _ in range(int(10 / DT)):
        _, _, done, ex = env.step(lqr_actions(env))
        alive &= ~(done & ~ex["time_outs"])
    print(f"residual env, zero correction: survives in {alive.float().mean() * 100:.0f}%")
    assert alive.float().mean() > 0.7 and env.get_observations()["critic"].shape == (1024, 13)
    print("selftest ok")


if __name__ == "__main__":
    _selftest()
