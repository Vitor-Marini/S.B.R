#!/usr/bin/env python3
"""Train a balancing policy with PPO and compare it with the firmware LQR on the same scenarios.

    .venv/bin/python labs/07-mujoco-rl/train.py [steps]     → trains (default 3M steps, 8 processes), saves
                                                             models/ppo_balance.zip, prints the comparison
    .venv/bin/python labs/07-mujoco-rl/train.py eval        → comparison only, with the saved model
    .venv/bin/python labs/07-mujoco-rl/train.py view [file] → 3D: the policy balancing in real time
                                                             keys: P push · R new random robot · L switch to LQR and back

Network: 2 hidden layers of 32 (small on purpose: it has to run on the ESP32-C3, which has no FPU).
"""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import env as E  # noqa: E402
import sim  # noqa: E402

MODEL = os.path.join(HERE, "models", "ppo_balance.zip")


def lqr_action(c):
    """The firmware LQR written as a policy (same gains, same caps), for a fair comparison."""
    a = 0.744 * c.pitch + 0.0707 * c.rate + max(-5, min(5, 27.67 * c.x + 14.04 * c.wheel))
    return np.array([np.clip(a / E.A_MAX, -1, 1)], np.float32)


def episode(act, seed, randomize, push=None, seconds=E.EPISODE_S):
    """One episode; push=(time s, newtons). Returns (fell, x array, pitch array)."""
    env = E.BalanceEnv(randomize=randomize, pushes=False)
    obs, _ = env.reset(seed=seed)
    xs, ths = [], []
    for i in range(int(seconds / sim.DT)):
        if push and i == int(push[0] / sim.DT):
            env.sim.push(push[1])
        obs, _, fell, _, info = env.step(act(obs, env.ctrl))
        xs.append(info["x"])
        ths.append(env.sim.true_pitch())
        if fell:
            return True, np.array(xs), np.array(ths)
    return False, np.array(xs), np.array(ths)


def compare(policy):
    acts = {"LQR (firmware)": lambda o, c: lqr_action(c),
            "PPO policy": lambda o, c: policy.predict(o, deterministic=True)[0]}
    print(f"{'':16} {'still: x range':>15} {'pitch jitter':>13} {'push survived':>14} {'random robots':>14}")
    for name, act in acts.items():
        fell, x, th = episode(act, 0, randomize=False)
        sel = slice(int(2 / sim.DT), None)
        still = "FELL" if fell else f"±{(x[sel].max() - x[sel].min()) * 50:.1f} cm"
        jit = "" if fell else f"±{th[sel].std():.2f}°"
        held = [n for n in (2, 4, 6, 8, 10, 12, 14) if not episode(act, 0, False, push=(1.0, n), seconds=5)[0]]
        robots = sum(not episode(act, 100 + k, randomize=True)[0] for k in range(20))
        print(f"{name:16} {still:>15} {jit:>13} {f'{max(held) if held else 0} N':>14} {f'{robots}/20 stand':>14}")


def train(steps):
    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.vec_env import SubprocVecEnv
    venv = make_vec_env(E.BalanceEnv, n_envs=8, vec_env_cls=SubprocVecEnv, seed=0)
    model = PPO("MlpPolicy", venv, policy_kwargs=dict(net_arch=[32, 32]), n_steps=1024, batch_size=4096,
                gamma=0.995, learning_rate=3e-4, ent_coef=0.0, verbose=0, seed=0, device="cpu")
    t0, done = time.time(), 0
    chunk = max(steps // 10, 100_000)
    while done < steps:
        model.learn(chunk, reset_num_timesteps=False)
        done += chunk
        lens = [ep["l"] for ep in model.ep_info_buffer]
        print(f"{done/1e6:5.1f}M steps  {time.time() - t0:5.0f} s  mean episode {np.mean(lens) * sim.DT:4.1f} s "
              f"(10 = never falls)  mean reward {np.mean([ep['r'] for ep in model.ep_info_buffer]):7.1f}", flush=True)
    os.makedirs(os.path.dirname(MODEL), exist_ok=True)
    model.save(MODEL)
    venv.close()
    return model


def view(policy):
    import mujoco.viewer
    env, keys, use_lqr = E.BalanceEnv(randomize=False, pushes=False), {}, False
    obs, _ = env.reset(seed=0)
    while True:
        with mujoco.viewer.launch_passive(env.sim.m, env.sim.d, key_callback=lambda k: keys.update(k=k)) as v:
            v.cam.distance, v.cam.elevation, v.cam.azimuth = 1.2, -15, 120
            while v.is_running():
                t0 = time.time()
                k = keys.pop("k", None)
                if k == ord("P"):
                    env.sim.push(float(np.random.choice([-6, 6])))
                elif k == ord("L"):
                    use_lqr = not use_lqr
                    print("controller:", "LQR" if use_lqr else "PPO policy")
                elif k == ord("R"):
                    break
                act = lqr_action(env.ctrl) if use_lqr else policy.predict(obs, deterministic=True)[0]
                obs, _, fell, trunc, _ = env.step(act)
                if fell:
                    print("fell — press R for a new robot")
                    keys.clear()
                    while v.is_running() and keys.get("k") != ord("R"):
                        v.sync()
                        time.sleep(0.05)
                    break
                v.sync()
                time.sleep(max(0, sim.DT - (time.time() - t0)))
            if not v.is_running():
                return
        env.randomize = True                      # R: a different robot (L, torque, friction, noise)
        obs, _ = env.reset(seed=int(np.random.randint(1 << 30)))
        print("new random robot")


if __name__ == "__main__":
    from stable_baselines3 import PPO
    arg = sys.argv[1] if len(sys.argv) > 1 else ""
    if arg == "view":
        view(PPO.load(sys.argv[2] if len(sys.argv) > 2 else MODEL, device="cpu"))
        sys.exit()
    model = PPO.load(MODEL, device="cpu") if arg == "eval" else train(int(float(arg)) if arg else 3_000_000)
    print()
    compare(model)
