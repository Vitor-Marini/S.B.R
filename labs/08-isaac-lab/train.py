#!/usr/bin/env python3
"""Train on the GPU with Isaac Lab's PPO (rsl_rl), thousands of robots at once.

    python labs/08-isaac-lab/train.py [steps]            → v11 = v10 + 30 checkpoints, score has the 6 N push
                                                           (default 300M), logs/sbr_v11/, models/ppo_v11_isaac_<it>.npz
                                                           (v7 = f8bc142, v8 = d191aa8, v9 = cc95cf0, v10 = bad073e)
    python labs/08-isaac-lab/train.py [steps] --v6       → v6 (network alone, critic = actor inputs)
    python labs/08-isaac-lab/train.py export model.pt    → .pt checkpoint → .npz (weights for lab 07's battery and
                                                           export.py)
    python labs/08-isaac-lab/train.py score a.npz ...    → short GPU battery (4096 robots, difficulty 0.8, 10 s):
                                                           picks which checkpoint goes to the full MuJoCo battery

Same network shape as lab 07 (2×64 tanh, linear output = wheel acceleration / 4 m/s²) so it can be compared on lab
07's battery (`eval_in_mujoco.py`) and later go to the ESP32-C3 like v3.
"""
import glob
import math
import os
import sys
import time

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
N_ENVS = 8192                 # measured: 4096 → 362k steps/s, 8192 → 537k (env only)
STEPS_PER_ENV = 24            # one PPO iteration = 8192 × 24 ≈ 197k steps (0.12 s of each robot's life)
D_MAX = 0.8                   # same curriculum as lab 07 v5/v6: 0.2 → 0.8 over the first 60 %
RESIDUAL = 0.25               # v7: correction up to ±0.25 × 4 = ±1 m/s² on top of the LQR


def runner_cfg(iterations, name="sbr_v11", residual=True):
    from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg
    cfg = RslRlOnPolicyRunnerCfg(
        num_steps_per_env=STEPS_PER_ENV, max_iterations=iterations, save_interval=max(1, iterations // 30),   # v11: dense — the best ones are mid-training
        experiment_name=name,
        actor=RslRlMLPModelCfg(hidden_dims=[64, 64], activation="tanh", obs_normalization=False,
                               distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=0.5 if residual else 1.0)),
        critic=RslRlMLPModelCfg(hidden_dims=[64, 64], activation="tanh", obs_normalization=False),
        algorithm=RslRlPpoAlgorithmCfg(
            value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.005,
            num_learning_epochs=5, num_mini_batches=4, learning_rate=1e-3, schedule="adaptive",
            gamma=0.995, lam=0.95, desired_kl=0.01, max_grad_norm=1.0)).to_dict()
    cfg["obs_groups"] = {"actor": ["policy"], "critic": ["policy", "critic"] if residual else ["policy"]}
    for m in ("actor", "critic"):          # Isaac Lab's cfg carries deprecated keys rsl_rl 5's MLPModel rejects
        cfg[m] = {k: v for k, v in cfg[m].items() if k in ("class_name", "hidden_dims", "activation",
                                                           "obs_normalization", "distribution_cfg")}
    return cfg


def export(pt, out, residual=0.0):
    """Actor MLP weights → npz (W0, b0, W1, b1, W2, b2): the deterministic policy is tanh(tanh(oW0+b0)W1+b1)W2+b2."""
    sd = torch.load(pt, map_location="cpu", weights_only=False)
    actor = next(v for k, v in sd.items() if "actor" in k and isinstance(v, dict))
    lin = [(k, v) for k, v in actor.items() if k.endswith(".weight") and v.dim() == 2]
    arrays = {}
    for i, (k, w) in enumerate(lin):
        arrays[f"W{i}"] = w.numpy().T
        arrays[f"b{i}"] = actor[k[:-len("weight")] + "bias"].numpy()
    assert [a.shape for a in arrays.values()][::2] == [(24, 64), (64, 64), (64, 1)], [a.shape for a in arrays.values()]
    import sbr_env as S
    np.savez(out, residual=np.float32(residual), hp_tau=np.float32(S.HP_TAU if residual else 0), **arrays)   # residual > 0: action = LQR + residual × network
    return out


def train(steps, w_th=None, residual=RESIDUAL):
    import sbr_env as S
    from rsl_rl.runners import OnPolicyRunner
    name = "sbr_v11" if residual else "sbr_v6"
    iterations = int(steps / (N_ENVS * STEPS_PER_ENV))
    env = S.SbrEnv(num_envs=N_ENVS, curriculum_ticks=int(0.6 * steps / N_ENVS), d_max=D_MAX, residual=residual,
                   hp_tau=S.HP_TAU if residual else 0.0, push_n=S.PUSH_N if residual else 0.0,
                   **({"w_th": w_th} if w_th else {}))
    log_dir = os.path.join(HERE, "logs", name, time.strftime("%Y-%m-%d_%H-%M-%S"))
    runner = OnPolicyRunner(env, runner_cfg(iterations, name, bool(residual)), log_dir=log_dir, device="cuda:0")
    t0 = time.time()
    runner.learn(num_learning_iterations=iterations, init_at_random_ep_len=True)
    print(f"{steps / 1e6:.0f}M steps in {(time.time() - t0) / 60:.1f} min")
    os.makedirs(os.path.join(HERE, "models"), exist_ok=True)
    out = []
    for pt in sorted(glob.glob(os.path.join(log_dir, "model_*.pt")), key=lambda f: int(os.path.basename(f)[6:-3])):
        it = int(os.path.basename(pt)[6:-3])
        out.append(export(pt, os.path.join(HERE, "models", f"ppo_{name[4:]}_isaac_{it}.npz"), residual))
        print("exported", out[-1])
    del runner, env
    torch.cuda.empty_cache()
    score(out)


def score(paths, n=4096, d=0.8, seconds=10.0, seed=123):
    """Short GPU battery: the same 4096 random robots (difficulty 0.8: setpoint error, pushes, grout, slopes, all
    scenarios) for each policy and for the plain LQR. Survival first, then tilt and shake decide."""
    import sbr_env as S
    rows = [("LQR", None)] + [(os.path.basename(p), dict(np.load(p))) for p in paths]
    for label, w in rows:
        res = float(w["residual"]) if w is not None and "residual" in w else 0.0
        tau = float(w["hp_tau"]) if w is not None and "hp_tau" in w else 0.0
        env = S.SbrEnv(num_envs=n, difficulty=d, seed=seed, residual=res, hp_tau=tau)
        W = {k: torch.tensor(v, device=env.device) for k, v in (w or {}).items() if k[0] in "Wb"}

        def act(env):
            if not W:
                return S.lqr_actions(env)
            o = env.get_observations()["policy"]
            h = torch.tanh(torch.tanh(o @ W["W0"] + W["b0"]) @ W["W1"] + W["b1"])
            return (h @ W["W2"] + W["b2"]).clamp(-1, 1)
        alive = torch.ones(n, dtype=torch.bool, device=env.device)
        tilt, jerk, rew, drift = 0.0, 0.0, 0.0, 0.0
        ticks = int(seconds / S.DT)
        for _ in range(ticks):
            _, r, done, ex = env.step(act(env))
            alive &= ~(done & ~ex["time_outs"])
            th = env.true_pitch() - torch.rad2deg(env.ar / S.G)
            tilt += (th[alive] ** 2).mean().item()
            jerk += (((env.action - env.prev_action) * S.A_MAX)[alive] ** 2).mean().item()
            rew += r.mean().item()
            drift += ((env.x - env.xfree)[alive] ** 2).mean().item()
        x0 = None                              # the line v7/v8 failed: standing still with a +2.75° setpoint error
        env = S.SbrEnv(num_envs=256, difficulty=0.0, seed=seed, residual=res, hp_tau=tau)
        env.bias[:], env.bias_drift[:], env.scenario[:] = 2.75, 0.0, 0
        for k in range(int(14 / S.DT)):        # < 15 s episode: no timeout reset
            env.step(act(env))
            if k == int(3 / S.DT):
                x0 = env.x.clone()
        creep = (env.x - x0).abs().median().item() * 100
        pushes = []                            # battery's push ladder, 256 tries per rung (one try is a coin flip)
        for N in (6.0, 6.5, 7.0):
            env = S.SbrEnv(num_envs=256, difficulty=0.0, seed=seed, residual=res, hp_tau=tau)
            env.scenario[:], env.push_f[:] = 0, N
            ok = torch.ones(256, dtype=torch.bool, device=env.device)
            for k in range(int(4 / S.DT)):
                if k == int(1 / S.DT):
                    env.push_until[:] = env.episode_length_buf * S.DT + 0.1   # N newtons for 0.1 s, as sim.push
                _, _, done, ex = env.step(act(env))
                ok &= ~(done & ~ex["time_outs"])
            pushes.append(f"{N:g}N {ok.float().mean().item() * 100:.0f}%")
        print(f"{label:28} survive {alive.float().mean().item() * 100:5.1f}%  tilt rms {math.sqrt(tilt / ticks):.2f}°  "
              f"jerk rms {math.sqrt(jerk / ticks):.2f} m/s²/tick  drift rms {math.sqrt(drift / ticks) * 100:.1f} cm  sp+2.75° creep {creep:.1f} cm  push {' '.join(pushes)}", flush=True)
        del env


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if a != "--v6"]
    if args and args[0] == "export":
        print(export(args[1], args[1].replace(".pt", ".npz"), 0.0 if "--v6" in sys.argv else RESIDUAL))
    elif args and args[0] == "score":
        score(args[1:])
    else:
        w = tuple(float(x) for x in args[1].split(",")) if len(args) > 1 else None   # e.g. 4,4 = v5 tilt
        train(int(float(args[0])) if args else 300_000_000, w, 0.0 if "--v6" in sys.argv else RESIDUAL)
