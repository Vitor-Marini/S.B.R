#!/usr/bin/env python3
"""Train v6 on the GPU with Isaac Lab's PPO (rsl_rl), thousands of robots at once.

    python labs/08-isaac-lab/train.py [steps]            → trains (default 300M), checkpoints in logs/sbr_v6/,
                                                           exports models/ppo_v6_isaac_<it>.npz for each checkpoint
    python labs/08-isaac-lab/train.py export model.pt    → .pt checkpoint → .npz (weights for lab 07's battery and
                                                           export.py)

Same network shape as lab 07 (2×64 tanh, linear output = wheel acceleration / 4 m/s²) so it can be compared on lab
07's battery (`eval_in_mujoco.py`) and later go to the ESP32-C3 like v3.
"""
import glob
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


def runner_cfg(iterations):
    from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg
    cfg = RslRlOnPolicyRunnerCfg(
        num_steps_per_env=STEPS_PER_ENV, max_iterations=iterations, save_interval=max(1, iterations // 6),
        experiment_name="sbr_v6",
        actor=RslRlMLPModelCfg(hidden_dims=[64, 64], activation="tanh", obs_normalization=False,
                               distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0)),
        critic=RslRlMLPModelCfg(hidden_dims=[64, 64], activation="tanh", obs_normalization=False),
        algorithm=RslRlPpoAlgorithmCfg(
            value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.005,
            num_learning_epochs=5, num_mini_batches=4, learning_rate=1e-3, schedule="adaptive",
            gamma=0.995, lam=0.95, desired_kl=0.01, max_grad_norm=1.0)).to_dict()
    cfg["obs_groups"] = {"actor": ["policy"], "critic": ["policy"]}
    for m in ("actor", "critic"):          # Isaac Lab's cfg carries deprecated keys rsl_rl 5's MLPModel rejects
        cfg[m] = {k: v for k, v in cfg[m].items() if k in ("class_name", "hidden_dims", "activation",
                                                           "obs_normalization", "distribution_cfg")}
    return cfg


def export(pt, out):
    """Actor MLP weights → npz (W0, b0, W1, b1, W2, b2): the deterministic policy is tanh(tanh(oW0+b0)W1+b1)W2+b2."""
    sd = torch.load(pt, map_location="cpu", weights_only=False)
    actor = next(v for k, v in sd.items() if "actor" in k and isinstance(v, dict))
    lin = [(k, v) for k, v in actor.items() if k.endswith(".weight") and v.dim() == 2]
    arrays = {}
    for i, (k, w) in enumerate(lin):
        arrays[f"W{i}"] = w.numpy().T
        arrays[f"b{i}"] = actor[k[:-len("weight")] + "bias"].numpy()
    assert [a.shape for a in arrays.values()][::2] == [(24, 64), (64, 64), (64, 1)], [a.shape for a in arrays.values()]
    np.savez(out, **arrays)
    return out


def train(steps, w_th=None):
    import sbr_env as S
    from rsl_rl.runners import OnPolicyRunner
    iterations = int(steps / (N_ENVS * STEPS_PER_ENV))
    env = S.SbrEnv(num_envs=N_ENVS, curriculum_ticks=int(0.6 * steps / N_ENVS), d_max=D_MAX,
                   **({"w_th": w_th} if w_th else {}))
    log_dir = os.path.join(HERE, "logs", "sbr_v6", time.strftime("%Y-%m-%d_%H-%M-%S"))
    runner = OnPolicyRunner(env, runner_cfg(iterations), log_dir=log_dir, device="cuda:0")
    t0 = time.time()
    runner.learn(num_learning_iterations=iterations, init_at_random_ep_len=True)
    print(f"{steps / 1e6:.0f}M steps in {(time.time() - t0) / 60:.1f} min")
    os.makedirs(os.path.join(HERE, "models"), exist_ok=True)
    for pt in sorted(glob.glob(os.path.join(log_dir, "model_*.pt"))):
        it = int(os.path.basename(pt)[6:-3])
        print("exported", export(pt, os.path.join(HERE, "models", f"ppo_v6_isaac_{it}.npz")))


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "export":
        print(export(sys.argv[2], sys.argv[2].replace(".pt", ".npz")))
    else:
        w = tuple(float(x) for x in sys.argv[2].split(",")) if len(sys.argv) > 2 else None   # e.g. 4,4 = v5 tilt
        train(int(float(sys.argv[1])) if len(sys.argv) > 1 else 300_000_000, w)
