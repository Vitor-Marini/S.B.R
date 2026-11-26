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


class NpzPolicy:
    """tanh MLP from train.py export; same call as a stable-baselines3 model in lab 07's battery."""

    def __init__(self, path):
        self.w = np.load(path)

    def predict(self, obs, deterministic=True):
        h = np.tanh(obs @ self.w["W0"] + self.w["b0"])
        h = np.tanh(h @ self.w["W1"] + self.w["b1"])
        return np.clip(h @ self.w["W2"] + self.w["b2"], -1, 1).astype(np.float32), None


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
    print(f"{'LQR':28}", wobble(lambda o, e: E.lqr_policy(e)))
    for path in sys.argv[1:]:
        p = NpzPolicy(path)
        print(f"{os.path.basename(path):28}", wobble(lambda o, e: p.predict(o)[0]))
    for path in sys.argv[1:]:
        print(f"\n=== {os.path.basename(path)}")
        T.battery(NpzPolicy(path))
