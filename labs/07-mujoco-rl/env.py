#!/usr/bin/env python3
"""Gymnasium environment v4 — "a day of use" compressed into 15 s episodes (labs/07-mujoco-rl/README.md, step 4).

v1–v3 only ever saw the robot standing still. On the real floor v3 drove fine but jerked at every joystick change and
fell into a ±4 m/s² bang-bang oscillation after a push. v4 trains every situation the robot lives today:

    scenarios   still · joystick driver (speed steps up to ±0.4 m/s, reversals, stops, turning) · move/turn sequences
                · mixed — chosen per episode
    world       pushes, tile grout (wheels briefly resisted), floor slope ±2°, setpoint error ±3° that drifts,
                sensor delay 5–15 ms, wheel/driver lag 0–10 ms, plus v3's robot randomization (L, mass, torque, ...)
    curriculum  `difficulty` 0→1 scales all of the above (train.py raises it during training)

    observation  last 3 frames × 8: measured pitch −θref /10°, rate /100 °/s, (v − v_ref) /0.5, position error /0.2,
                 v_ref /0.5, a_ref /1, turn command /45, last action                        ← all known to the firmware
    action       wheel acceleration, ±1 × 4 m/s² (firmware cap)
    reward       1 − min(1, state/40) − effort/40 (linear; effort = action + jerk, never capped): punishes bang-bang

    .venv/bin/python labs/07-mujoco-rl/env.py selftest
"""
import copy
import math
import os
import sys

import gymnasium as gym
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim  # noqa: E402

A_MAX = 4.0              # m/s², firmware cap
EPISODE_S = 15.0
FALL_DEG = 30.0
SLIP = 0.09              # m: firmware position-error limit
V_RAMP = 0.6             # m/s²: firmware joystick ramp
W_RAMP = 240.0           # °/s²: firmware turn ramp
MOVE_A, MOVE_V = 0.1, 0.15   # firmware move profile (mva, mvs)
TRACK = 0.150
HISTORY = 3
FRAME = 8
# cost weights (Bryson): the deviation each term may reach before it "hurts"
W_X, W_V, W_TH, W_W, W_A, W_JERK = 0.03, 0.4, 4.0, 50.0, 1.5, 0.5
COST_SCALE = 40.0        # LQR standing ≈ 0.48/step, v4's 8.5 Hz shake ≈ 0.26 (measured)


class Reference:
    """The firmware's reference generator (velocity.cpp _updateLqr): joystick ramp or move profile → x_ref, v_ref, a_ref;
    turn command ramped like the firmware turn loop."""

    def __init__(self):
        self.v_target = self.w_target = 0.0
        self.vr = self.ar = self.xref = 0.0
        self.wr = 0.0
        self.move = None            # [goal_abs_x, start_x, ref_p, ref_v]

    def start_move(self, dist, x_now):
        self.move = [dist, x_now, 0.0, 0.0]
        self.v_target = 0.0

    def step(self, dt):
        if self.move:
            goal, x0, p, v = self.move
            g, s = abs(goal), math.copysign(1, goal)
            vn = min(v + MOVE_A * dt, MOVE_V, math.sqrt(2 * MOVE_A * max(g - p, 0)))
            self.ar, self.vr = s * (vn - v) / dt, s * vn
            p = min(p + max(vn, 0.005) * dt, g)
            self.xref = x0 + s * p
            self.move = None if p >= g else [goal, x0, p, vn]
            if not self.move:
                self.vr = self.ar = 0.0
        else:
            prev = self.vr
            self.vr += max(-V_RAMP * dt, min(V_RAMP * dt, self.v_target - self.vr))
            self.ar = (self.vr - prev) / dt
            self.xref += self.vr * dt
        self.wr += max(-W_RAMP * dt, min(W_RAMP * dt, self.w_target - self.wr))


class PolicyController:
    """Same contract as FirmwareLQR (step(pitch, rate) → wheel speed); the acceleration comes from the network."""

    def __init__(self, pitch_bias=0.0):
        self.wheel = self.x = self.ex = 0.0
        self.action = self.prev_action = 0.0
        self.bias = pitch_bias
        self.ref = Reference()
        self.turn_offset = 0.0
        self.pitch = self.rate = 0.0

    def step(self, pitch_deg, rate_dps):
        r = self.ref
        r.step(sim.DT)
        self.pitch, self.rate = pitch_deg + self.bias, rate_dps
        self.x += self.wheel * sim.DT
        self.ex = self.x - r.xref
        if abs(self.ex) > SLIP:
            r.xref = self.x - math.copysign(SLIP, self.ex)
            self.ex = math.copysign(SLIP, self.ex)
        self.wheel = max(-sim.P["motor"]["max_speed"], min(sim.P["motor"]["max_speed"],
                                                           self.wheel + self.action * A_MAX * sim.DT))
        self.turn_offset = math.radians(r.wr) * TRACK / 2      # firmware turn feedforward (stepper)
        return self.wheel

    def frame(self):
        r = self.ref
        theta_ref = math.degrees(r.ar / 9.81)
        return [(self.pitch - theta_ref) / 10, self.rate / 100, (self.wheel - r.vr) / 0.5, self.ex / 0.2,
                r.vr / 0.5, r.ar / 1.0, r.wr / 45, self.action]


def randomized_params(rng, d):
    p = copy.deepcopy(sim.P)
    p["body"]["pendulum_length_L"] *= rng.uniform(1 - 0.2 * d, 1 + 0.2 * d)
    p["body"]["mass"] *= rng.uniform(1 - 0.1 * d, 1 + 0.1 * d)
    p["motor"]["torque_limit"] = rng.uniform(0.3 - 0.1 * d, 0.3 + 0.1 * d)
    p["floor_friction"] = rng.uniform(0.8 - 0.3 * d, 0.8 + 0.2 * d)
    p["disturbance_torque"] = rng.uniform(0.0, 0.015 * max(d, 0.5))
    return p


class BalanceEnv(gym.Env):
    metadata = {"render_modes": []}
    SCENARIOS = ("still", "joystick", "moves", "mixed")

    def __init__(self, difficulty=1.0, scenario=None, randomize=True, bias=None, seed_robot=None):
        self.difficulty = difficulty
        self.scenario_fixed, self.randomize, self.bias_fixed = scenario, randomize, bias
        self.observation_space = gym.spaces.Box(-10, 10, (HISTORY * FRAME,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (1,), np.float32)
        self.sim = None
        self.script = None           # eval: list of (t, callable(env)) events instead of random ones

    def set_difficulty(self, d):     # called by the training callback (curriculum)
        self.difficulty = float(d)

    def _obs(self):
        return np.clip(np.concatenate(self.hist), -10, 10).astype(np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        rng, d = self.np_random, self.difficulty if self.randomize else 0.0
        self.scenario = self.scenario_fixed or self.SCENARIOS[int(rng.integers(len(self.SCENARIOS)))]
        params = randomized_params(rng, d) if self.randomize else sim.P
        bias = self.bias_fixed if self.bias_fixed is not None else rng.uniform(-3, 3) * d
        self.ctrl = PolicyController(bias)
        self.sim = sim.Sim(self.ctrl, gyro_noise=rng.uniform(0.1, 0.3 + 0.3 * d) if self.randomize else 0.3,
                           seed=int(rng.integers(1 << 31)), pitch0=rng.uniform(-3, 3) * max(d, 0.3), params=params,
                           delay=int(rng.integers(1, 2 + round(2 * d))) if self.randomize else 1,
                           act_delay=int(rng.integers(0, 1 + round(2 * d))) if self.randomize else 0,
                           slope_deg=rng.uniform(-2, 2) * d if self.randomize else 0.0)
        self.ctrl.pitch = self.sim.pitch + bias
        self.bias_drift = rng.uniform(0, 0.3) * d       # °/s random walk of the setpoint error
        self.next_cmd = 0.0
        self.steps = 0
        self.hist = [self.ctrl.frame()] * HISTORY
        return self._obs(), {}

    # --- random events of a "day of use" (scaled by difficulty)
    def _events(self):
        rng, d, c, t = self.np_random, self.difficulty, self.ctrl, self.steps * sim.DT
        r = c.ref
        if self.script is not None:
            while self.script and self.script[0][0] <= t:
                self.script.pop(0)[1](self)
            return
        if self.scenario in ("joystick", "mixed") and t >= self.next_cmd:
            r.v_target = 0.0 if rng.random() < 0.25 else rng.uniform(-0.4, 0.4) * max(d, 0.25)
            r.w_target = rng.uniform(-45, 45) * d if rng.random() < 0.4 else 0.0
            self.next_cmd = t + rng.uniform(0.5, 3.0)
        if self.scenario == "moves" and t >= self.next_cmd and r.move is None:
            if rng.random() < 0.5:
                r.start_move(rng.choice([-1, 1]) * rng.uniform(0.2, 0.6), c.x)
                r.w_target = 0.0
            else:
                r.w_target = rng.choice([-45, 45]) * max(d, 0.3)     # turn in place for a while
            self.next_cmd = t + rng.uniform(2.0, 4.0)
        if self.scenario == "moves" and r.w_target and t >= self.next_cmd - 1.0:
            r.w_target = 0.0
        if rng.random() < d / 700:
            self.sim.push(rng.uniform(-6, 6) * d, rng.uniform(0.05, 0.2))
        if abs(c.wheel) > 0.05 and rng.random() < d / 300:           # grout only matters when rolling
            self.sim.bump(rng.uniform(0.05, 0.25) * d, rng.uniform(0.02, 0.08))
        c.bias += rng.normal(0, self.bias_drift * math.sqrt(sim.DT))
        c.bias = max(-3.5, min(3.5, c.bias))

    def step(self, action):
        c = self.ctrl
        c.prev_action, c.action = c.action, float(np.clip(action[0], -1, 1))
        self._events()
        self.sim.step()
        self.steps += 1
        self.hist = self.hist[1:] + [c.frame()]
        r = c.ref
        th = self.sim.true_pitch() - math.degrees(r.ar / 9.81)
        state = (c.ex / W_X) ** 2 + ((c.wheel - r.vr) / W_V) ** 2 + (th / W_TH) ** 2 + (c.rate / W_W) ** 2
        effort = (c.action * A_MAX / W_A) ** 2 + ((c.action - c.prev_action) * A_MAX / W_JERK) ** 2
        fell = abs(self.sim.true_pitch()) > FALL_DEG
        # v5: linear, and only the state part is capped (a push can make it explode); effort is bounded by itself and
        # must never be capped — v4's clipped reward made rare ±1 switches cheap, so bang-bang scored above the LQR
        reward = 1.0 - min(1.0, state / COST_SCALE) - effort / COST_SCALE
        truncated = self.steps * sim.DT >= getattr(self, "max_s", EPISODE_S)
        return self._obs(), reward, fell, truncated, {"x": self.sim.d.qpos[0], "y": self.sim.d.qpos[1]}


def lqr_policy(env):
    """The firmware LQR on the same observation (θref, reference, caps): the baseline every test compares against."""
    c, r = env.ctrl, env.ctrl.ref
    theta_ref = math.degrees(r.ar / 9.81)
    a = 0.744 * (c.pitch - theta_ref) + 0.0707 * c.rate + max(-5, min(5, 27.67 * c.ex + 14.04 * (c.wheel - r.vr) + r.ar))
    return np.array([max(-1, min(1, a / A_MAX))], np.float32)


def _selftest():
    from gymnasium.utils.env_checker import check_env
    check_env(BalanceEnv(), skip_render_check=True)
    env = BalanceEnv(difficulty=0.0, scenario="still", randomize=False)
    env.reset(seed=0)
    n = 0
    while True:
        _, _, term, trunc, _ = env.step(np.zeros(1, np.float32))
        n += 1
        if term or trunc:
            break
    assert term and n * sim.DT < 3, "zero action must fall"
    for scen in BalanceEnv.SCENARIOS:          # the LQR must survive every scenario at full difficulty, mostly
        ok = 0
        for k in range(5):
            env = BalanceEnv(difficulty=1.0, scenario=scen)
            env.reset(seed=k)
            while True:
                _, _, term, trunc, _ = env.step(lqr_policy(env))
                if term or trunc:
                    break
            ok += not term
        print(f"  LQR survives '{scen}' at difficulty 1: {ok}/5")
        assert ok >= 2, f"scenario '{scen}' too hard even for the LQR"
    print("selftest ok")


if __name__ == "__main__":
    _selftest()
