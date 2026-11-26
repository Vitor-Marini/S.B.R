#!/usr/bin/env python3
"""Run a policy trained in Isaac (Newton) on lab 07's battery, in MuJoCo — a sim-to-sim check and a fair comparison
with the LQR and v4/v5 (same tests, same simulator they were judged in).

    .venv/bin/python labs/08-isaac-lab/eval_in_mujoco.py models/ppo_v6_isaac_1524.npz [more.npz ...]

Prints the standing wobble (tilt std, dominant frequency, % of time the action is saturated: the numbers that told
v4's bang-bang and v5's lazy sway apart) and then lab 07's full battery.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "07-mujoco-rl"))
import env as E  # noqa: E402
import sim  # noqa: E402
import train as T  # noqa: E402


def lqr_from_obs(o):
    """Copy of sbr_env.lqr_from_obs (that module needs warp/newton, not installed here); checked against lab 07 below."""
    f = o[..., -8:]
    a = 7.44 * f[..., 0] + 7.07 * f[..., 1] + (5.534 * f[..., 3] + 7.02 * f[..., 2] + f[..., 5]).clip(-5, 5)
    return (a / E.A_MAX).clip(-1, 1)


class NpzPolicy:
    """tanh MLP from train.py export; same call as a stable-baselines3 model in lab 07's battery.
    v7 (residual > 0): action = LQR + residual × network. v9 (hp_tau > 0): the correction is high-passed, so the policy
    has one state float (the firmware will too), reset when an episode starts."""

    def __init__(self, path):
        self.w = np.load(path)
        self.res = float(self.w["residual"]) if "residual" in self.w else 0.0
        self.tau = float(self.w["hp_tau"]) if "hp_tau" in self.w else 0.0
        self.lp = 0.0

    def predict(self, obs, deterministic=True):
        h = np.tanh(obs @ self.w["W0"] + self.w["b0"])
        h = np.tanh(h @ self.w["W1"] + self.w["b1"])
        a = np.clip(h @ self.w["W2"] + self.w["b2"], -1, 1)
        if self.res:
            raw = self.res * a
            if self.tau:
                if np.array_equal(obs[:8], obs[8:16]) and np.array_equal(obs[8:16], obs[16:]):
                    self.lp = 0.0         # lab 07 reset fills the history with 3 equal frames: new episode
                self.lp = self.lp + (raw - self.lp) * (sim.DT / self.tau)
                raw = raw - self.lp
            a = np.clip(lqr_from_obs(obs) + raw, -1, 1)
        return a.astype(np.float32), None


def _check_lqr():
    e = E.BalanceEnv(difficulty=1.0, scenario="mixed")
    o, _ = e.reset(seed=3)
    for _ in range(400):
        assert abs(lqr_from_obs(o) - E.lqr_policy(e)[0]) < 1e-4
        o, _, f, t, _ = e.step(E.lqr_policy(e))
        if f or t:
            break


def wobble(act):
    e = E.BalanceEnv(difficulty=0, scenario="still", randomize=False)
    o, _ = e.reset(seed=0)
    e.script, e.max_s = [], 20
    th, a = [], []
    while True:
        o, _, f, t, _ = e.step(act(o, e))
        th.append(e.sim.true_pitch())
        a.append(e.ctrl.action)
        if f or t:
            break
    th, a = np.array(th[600:]), np.array(a[600:])
    if len(th) < 100:
        return "FELL"
    F = np.abs(np.fft.rfft(th - th.mean()))
    fr = np.fft.rfftfreq(len(th), sim.DT)
    return f"tilt ±{th.std():.2f}°  peak {fr[F[1:].argmax() + 1]:.1f} Hz  saturated {np.mean(np.abs(a) > 0.95) * 100:.0f}%"


if __name__ == "__main__":
    _check_lqr()
    print(f"{'LQR':28}", wobble(lambda o, e: E.lqr_policy(e)))
    for path in sys.argv[1:]:
        p = NpzPolicy(path)
        print(f"{os.path.basename(path):28}", wobble(lambda o, e: p.predict(o)[0]))
    for path in sys.argv[1:]:
        print(f"\n=== {os.path.basename(path)}")
        T.battery(NpzPolicy(path))
