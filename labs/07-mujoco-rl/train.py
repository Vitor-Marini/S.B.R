#!/usr/bin/env python3
"""Train the v5 policy (every situation of a day of use) and run the test battery against the firmware LQR.

    .venv/bin/python labs/07-mujoco-rl/train.py [steps]       → trains (default 30M, 16 processes, curriculum),
                                                               checkpoints every 5M in models/, then the battery
    .venv/bin/python labs/07-mujoco-rl/train.py eval [model]  → the battery only
    .venv/bin/python labs/07-mujoco-rl/train.py view [model]  → 3D: P push · L LQR/policy · J joystick script ·
                                                               G grout · R random robot
    .venv/bin/python labs/07-mujoco-rl/train.py world         → 3D: the training world (random scenarios, pushes,
                                                               grout, slope...) driven by the LQR; R = new episode

The battery copies the real tests: standing (with and without setpoint error), the joystick drive from the floor
test that made v3 jerk, the demo, a push ladder, rolling over grout, and 50 random robots. The policy only goes to
the robot if it ties or beats the LQR on every line.
"""
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import env as E  # noqa: E402
import sim  # noqa: E402

MODEL = os.path.join(HERE, "models", "ppo_v5.zip")
D_MAX = 0.8                    # LQR survives 60/60 at 0.6, 53/60 at 0.8, 35/60 at 1.0 (v4 trained to 1.0)
N_ENVS = 32                    # measured: 16 → 6.7k steps/s, 32 → 7.7k, 64 → 8.4k but 2× RAM; 128 ran out of 15 GB


def ev(t, **kw):
    """Script event: at time t set reference fields (v, w) or start a move/turn/push/bump."""
    def apply(env):
        r = env.ctrl.ref
        if "v" in kw:
            r.v_target = kw["v"]
        if "w" in kw:
            r.w_target = kw["w"]
        if "move" in kw:
            r.start_move(kw["move"], env.ctrl.x)
        if "push" in kw:
            env.sim.push(kw["push"])
        if "bump" in kw:
            env.sim.bump(kw["bump"], 0.05)
    return (t, apply)


# the real joystick drive (2nd floor test, v3): reverse 0.2→0.4, stop, forward 0.4 with turns, stop
JOYSTICK = [ev(2, v=-0.2), ev(4, v=-0.3), ev(6, v=-0.4), ev(8, v=0.0), ev(9, v=0.4), ev(11, w=-45), ev(12, w=0),
            ev(12.5, w=45), ev(13.5, w=0, v=0.0)]
DEMO = [ev(1, move=0.5), ev(6, w=45), ev(10, w=0), ev(11, move=0.5), ev(16, w=45), ev(20, w=0)]
GROUT = [ev(1, v=0.3)] + [ev(2 + k, bump=0.15) for k in range(8)] + [ev(10, v=0.0)]


def run(act, script=None, seconds=15.0, bias=0.0, randomize=False, scenario="still", seed=0, difficulty=1.0):
    env = E.BalanceEnv(difficulty=difficulty, scenario=scenario, randomize=randomize, bias=bias)
    obs, _ = env.reset(seed=seed)
    env.script, env.max_s = (list(script) if script is not None else None), seconds
    if script is not None:
        env.script = sorted(env.script, key=lambda e: e[0])
    x0 = env.sim.d.qpos[:2].copy()
    xs, ths = [], []
    while True:
        obs, _, fell, trunc, info = env.step(act(obs, env))
        xs.append(info["x"])
        ths.append(env.sim.true_pitch())
        if fell or trunc:
            break
    end = np.hypot(*(env.sim.d.qpos[:2] - x0))
    return fell, np.array(xs), np.array(ths), end


def battery(policy):
    acts = {"LQR": lambda o, e: E.lqr_policy(e)}
    if policy is not None:
        acts["policy"] = lambda o, e: policy.predict(o, deterministic=True)[0]
    rows = {}
    for name, act in acts.items():
        r = {}
        f, x, th, _ = run(act, script=[], seconds=20)
        s = slice(int(3 / sim.DT), None)
        r["still"] = "FELL" if f else f"±{np.ptp(x[s]) * 50:.1f} cm ±{th[s].std():.2f}°"
        f, x, th, _ = run(act, script=[], seconds=20, bias=2.75)
        r["still, sp +2.75°"] = "FELL" if f else f"±{np.ptp(x[s]) * 50:.1f} cm ±{th[s].std():.2f}°"
        f, x, th, _ = run(act, script=JOYSTICK, seconds=16)
        r["joystick drive"] = "FELL" if f else f"max tilt {np.abs(th).max():.1f}°"
        f, x, th, end = run(act, script=DEMO, seconds=25)
        r["demo (back home)"] = "FELL" if f else f"ends {end * 100:.1f} cm off"
        held = [n for n in (2, 4, 6, 8, 10, 12) if not run(act, script=[ev(1, push=n)], seconds=5)[0]]
        r["push ladder"] = f"{max(held) if held else 0} N"
        f, x, th, _ = run(act, script=GROUT, seconds=12)
        r["grout at 0.3 m/s"] = "FELL" if f else f"max tilt {np.abs(th).max():.1f}°"
        ok = sum(not run(act, randomize=True, scenario="mixed", seed=1000 + k)[0] for k in range(50))
        r["50 random robots"] = f"{ok}/50"
        ok = sum(not run(act, randomize=True, scenario="mixed", seed=1000 + k, difficulty=0.8)[0] for k in range(50))
        r["50 robots, diff 0.8"] = f"{ok}/50"
        rows[name] = r
    print(f"\n{'test':20}" + "".join(f"{n:>24}" for n in rows))
    for test in next(iter(rows.values())):
        print(f"{test:20}" + "".join(f"{rows[n][test]:>24}" for n in rows))


def train(steps):
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.vec_env import SubprocVecEnv
    torch.set_num_threads(1)   # measured faster than 2 or 4 with 32 workers

    class Curriculum(BaseCallback):
        """difficulty 0.2 → D_MAX over the first 60 % of training; log + checkpoint every 5M steps."""
        def __init__(self):
            super().__init__()
            self.t0, self.next_log = time.time(), 0

        def _on_rollout_start(self):
            d = min(D_MAX, 0.2 + (D_MAX - 0.2) * self.num_timesteps / (0.6 * steps))
            self.training_env.env_method("set_difficulty", d)

        def _on_step(self):
            if self.num_timesteps >= self.next_log:
                buf = self.model.ep_info_buffer
                if buf:
                    d = self.training_env.get_attr("difficulty", 0)[0]
                    print(f"{self.num_timesteps / 1e6:5.1f}M  {time.time() - self.t0:6.0f} s  difficulty {d:.2f}  "
                          f"episode {np.mean([e['l'] for e in buf]) * sim.DT:4.1f}/15 s  reward "
                          f"{np.mean([e['r'] for e in buf]):7.1f}", flush=True)
                if self.num_timesteps >= 5_000_000 and self.num_timesteps % 5_000_000 < 8192:
                    self.model.save(os.path.join(HERE, "models", f"ppo_v5_{self.num_timesteps // 1_000_000}M"))
                self.next_log += 1_000_000
            return True

    venv = make_vec_env(E.BalanceEnv, n_envs=N_ENVS, vec_env_cls=SubprocVecEnv, seed=0,
                        env_kwargs=dict(difficulty=0.2))
    model = PPO("MlpPolicy", venv, policy_kwargs=dict(net_arch=[64, 64]), n_steps=8192 // N_ENVS, batch_size=4096,
                gamma=0.995, gae_lambda=0.95, learning_rate=3e-4, ent_coef=0.0, verbose=0, seed=0, device="cpu")
    model.learn(steps, callback=Curriculum())
    os.makedirs(os.path.dirname(MODEL), exist_ok=True)
    model.save(MODEL)
    venv.close()
    return model


def view(policy, world=False):
    import mujoco.viewer

    def new_env(randomize):
        e = E.BalanceEnv(difficulty=1.0, scenario=None if world else "still", randomize=randomize or world)
        o, _ = e.reset(seed=int(np.random.randint(1 << 30)))
        e.script, e.max_s = (None if world else []), 1e9     # world: random events of a training episode
        if world:
            print(f"episode: {e.scenario}")
        return e, o

    env, obs = new_env(False)
    keys, use_lqr = {}, policy is None
    while True:
        with mujoco.viewer.launch_passive(env.sim.m, env.sim.d, key_callback=lambda k: keys.update(k=k)) as v:
            v.cam.distance, v.cam.elevation, v.cam.azimuth = 1.5, -15, 120
            while v.is_running():
                t0 = time.time()
                k = keys.pop("k", None)
                now = env.steps * sim.DT
                if k == ord("P"):
                    env.sim.push(float(np.random.choice([-6, 6])))
                elif k == ord("L") and policy is not None:
                    use_lqr = not use_lqr
                    print("controller:", "LQR" if use_lqr else "policy")
                elif k == ord("J"):
                    env.script = sorted([(now + t, f) for t, f in JOYSTICK], key=lambda e: e[0])
                elif k == ord("G"):
                    env.script = sorted([(now + t, f) for t, f in GROUT], key=lambda e: e[0])
                elif k == ord("R"):
                    break
                act = E.lqr_policy(env) if use_lqr else policy.predict(obs, deterministic=True)[0]
                obs, _, fell, _, _ = env.step(act)
                if fell:
                    print("fell — press R")
                    keys.clear()
                    while v.is_running() and keys.get("k") != ord("R"):
                        v.sync()
                        time.sleep(0.05)
                    break
                v.sync()
                time.sleep(max(0, sim.DT - (time.time() - t0)))
            if not v.is_running():
                return
        env, obs = new_env(True)
        print("new random robot")


if __name__ == "__main__":
    from stable_baselines3 import PPO
    arg = sys.argv[1] if len(sys.argv) > 1 else ""
    path = sys.argv[2] if len(sys.argv) > 2 else MODEL
    if arg == "world":
        view(None, world=True)
    elif arg == "view":
        view(PPO.load(path, device="cpu"))
    elif arg == "eval":
        battery(PPO.load(path, device="cpu") if os.path.exists(path) else None)
    else:
        battery(train(int(float(arg)) if arg else 30_000_000))
