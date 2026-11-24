#!/usr/bin/env python3
"""Gymnasium environment: the validated simulator (sim.py) as a game for a neural network.

The network replaces exactly what the LQR does in the firmware: every 5 ms it reads what the robot can measure and
outputs a wheel acceleration. Everything around it (filter, delay, stepper, physics) stays the same.

    observation (5): filtered pitch /10°, pitch rate /100 °/s, wheel speed /0.5 m/s, position error /0.2 m,
                     last action                                   ← all things the firmware has
    action (1):      wheel acceleration in [−1, 1] × 4 m/s²       ← same output and cap as the firmware LQR
    reward:          1 per step standing − the LQR cost (Bryson weights from labs/03-lqr/lqr.py), so "stand still,
                     stay in place, don't thrash the motor" is what pays
    episode:         10 s, or until it falls (|pitch| > 30°)
    each reset:      a slightly different robot (L, torque, friction, noise, dirt) + random pushes during the episode

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

A_MAX = 4.0            # m/s², same cap as the firmware
EPISODE_S = 10.0
FALL_DEG = 30.0
# Bryson: the deviation each term may reach before it "hurts" (same spirit as the LQR weights)
W_X, W_V, W_TH, W_W, W_A = 0.05, 0.4, 8.0, 100.0, 1.5


class PolicyController:
    """Same contract as FirmwareLQR (step(pitch, rate) → wheel speed), but the acceleration comes from outside."""

    def __init__(self):
        self.wheel = self.x = 0.0
        self.action = 0.0

    def step(self, pitch_deg, rate_dps):
        self.pitch, self.rate = pitch_deg, rate_dps   # what the network will see next
        self.x += self.wheel * sim.DT
        a = self.action * A_MAX
        self.wheel = max(-sim.P["motor"]["max_speed"], min(sim.P["motor"]["max_speed"], self.wheel + a * sim.DT))
        return self.wheel


def randomized_params(rng):
    """A robot that is not quite ours: the policy has to work for all of them (and so, hopefully, for the real one)."""
    p = copy.deepcopy(sim.P)
    p["body"]["pendulum_length_L"] *= rng.uniform(0.8, 1.2)
    p["body"]["mass"] *= rng.uniform(0.9, 1.1)
    p["motor"]["torque_limit"] = rng.uniform(0.2, 0.4)
    p["floor_friction"] = rng.uniform(0.5, 1.0)
    p["disturbance_torque"] = rng.uniform(0.0, 0.015)
    return p


class BalanceEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, randomize=True, pushes=True):
        self.randomize, self.pushes = randomize, pushes
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (5,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (1,), np.float32)
        self.sim = None

    def _obs(self):
        c = self.ctrl
        return np.array([c.pitch / 10, c.rate / 100, c.wheel / 0.5, c.x / 0.2, c.action], np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        rng = self.np_random
        params = randomized_params(rng) if self.randomize else sim.P
        self.ctrl = PolicyController()
        self.sim = sim.Sim(self.ctrl, gyro_noise=rng.uniform(0.1, 0.6) if self.randomize else 0.3,
                           seed=int(rng.integers(1 << 31)), pitch0=rng.uniform(-3, 3), params=params)
        self.ctrl.pitch, self.ctrl.rate = self.sim.pitch, 0.0
        self.steps = 0
        return self._obs(), {}

    def step(self, action):
        self.ctrl.action = float(np.clip(action[0], -1, 1))
        if self.pushes and self.np_random.random() < 1 / 400:     # a push every ~2 s on average
            self.sim.push(self.np_random.uniform(-6, 6))
        self.sim.step()
        self.steps += 1
        c = self.ctrl
        cost = ((c.x / W_X) ** 2 + (c.wheel / W_V) ** 2 + (c.pitch / W_TH) ** 2 + (c.rate / W_W) ** 2
                + (c.action * A_MAX / W_A) ** 2)
        fell = abs(self.sim.true_pitch()) > FALL_DEG
        reward = 1.0 - min(1.0, 0.05 * cost)
        truncated = self.steps * sim.DT >= EPISODE_S
        return self._obs(), reward, fell, truncated, {"x": self.sim.d.qpos[0]}


def _selftest():
    from gymnasium.utils.env_checker import check_env
    env = BalanceEnv()
    check_env(env, skip_render_check=True)
    obs, _ = env.reset(seed=0)
    total, n = 0.0, 0
    while True:                                   # doing nothing: it must fall, and quickly
        obs, r, term, trunc, _ = env.step(np.zeros(1, np.float32))
        total, n = total + r, n + 1
        if term or trunc:
            break
    assert term and n * sim.DT < 3, f"zero action should fall fast (lasted {n * sim.DT:.1f} s)"
    env = BalanceEnv(randomize=False, pushes=False)  # the LQR, written as a policy, must stand the whole episode
    obs, _ = env.reset(seed=1)
    for _ in range(int(EPISODE_S / sim.DT)):
        c = env.ctrl
        a = 0.744 * c.pitch + 0.0707 * c.rate + max(-5, min(5, 27.67 * c.x + 14.04 * c.wheel))
        obs, r, term, trunc, _ = env.step(np.array([np.clip(a / A_MAX, -1, 1)], np.float32))
        assert not term, "LQR as a policy fell"
        if trunc:
            break
    print(f"selftest ok (zero action falls in {n * sim.DT:.2f} s; LQR as a policy stands 10 s)")


if __name__ == "__main__":
    _selftest()
