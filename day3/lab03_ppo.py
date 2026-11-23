"""CPU PPO experiment on the official MuJoCo InvertedPendulum-v5 environment."""
from pathlib import Path
import argparse
import csv
import json
import platform
import time

import gymnasium as gym
import mujoco
import numpy as np
import stable_baselines3
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.logger import configure
from pendulum_env import make_env

ROOT = Path(__file__).resolve().parent
ENV_ID = "InvertedPendulum-v5"
EVAL_SEEDS = list(range(9000, 9020))


def evaluate(policy):
    env = make_env()
    episodes = []
    for seed in EVAL_SEEDS:
        obs, _ = env.reset(seed=seed)
        total, length = 0.0, 0
        while True:
            action = np.zeros(1, dtype=np.float32) if policy is None else policy.predict(obs, deterministic=True)[0]
            obs, reward, terminated, truncated, _ = env.step(action)
            assert np.isfinite(obs).all() and np.isfinite(reward)
            total += float(reward)
            length += 1
            if terminated or truncated:
                break
        episodes.append({"reset_seed": seed, "return": total, "steps": length,
                         "terminated": bool(terminated), "truncated": bool(truncated)})
    env.close()
    return {"mean_return": float(np.mean([row["return"] for row in episodes])),
            "mean_steps": float(np.mean([row["steps"] for row in episodes])),
            "episodes": episodes}


def contract_check():
    env = make_env()
    check_env(env.unwrapped, warn=True)
    obs1, _ = env.reset(seed=1234)
    obs2, _ = env.reset(seed=1234)
    assert np.array_equal(obs1, obs2)
    assert env.observation_space.shape == (4,) and env.action_space.shape == (1,)
    base = env.unwrapped
    result = {"env": ENV_ID, "observation_order": ["cart_m", "pole_rad", "cart_m_s", "pole_rad_s"],
              "observation_shape": list(env.observation_space.shape),
              "action_shape": list(env.action_space.shape),
              "action_low": env.action_space.low.tolist(), "action_high": env.action_space.high.tolist(),
              "nq": base.model.nq, "nv": base.model.nv, "nu": base.model.nu,
              "physics_dt": float(base.model.opt.timestep), "frame_skip": base.frame_skip,
              "control_dt": float(base.dt), "episode_limit": env.spec.max_episode_steps,
              "seeded_reset_equal": True}
    result["environment_class"] = type(base).__name__
    env.close()
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["smoke", "train", "evaluate"])
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--steps", type=int, default=65536)
    parser.add_argument("--output-root", type=Path, default=ROOT / "results")
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error("--steps must be positive")
    torch.set_num_threads(1)
    out = args.output_root / ("smoke" if args.mode == "smoke" else f"ppo_seed{args.seed}")
    out.mkdir(parents=True, exist_ok=True)
    contract = contract_check()
    if args.mode == "evaluate":
        model = PPO.load(out / "policy.zip", device="cpu")
        result = evaluate(model)
        (out / "reevaluation.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps({"status": "PASS", "mean_return": result["mean_return"], "episodes": len(EVAL_SEEDS)}))
        return
    env = Monitor(make_env(), filename=str(out / "monitor.csv"))
    model = PPO("MlpPolicy", env, learning_rate=3e-4, n_steps=1024,
                batch_size=64, n_epochs=10, gamma=0.99, gae_lambda=0.95,
                clip_range=0.2, policy_kwargs={"net_arch": [64, 64]},
                seed=args.seed, device="cpu", verbose=0)
    model.set_logger(configure(str(out / "training"), ["csv"]))
    before = torch.cat([p.detach().flatten().clone() for p in model.policy.parameters()])
    baseline = evaluate(None)
    untrained = evaluate(model)
    started = time.perf_counter()
    model.learn(total_timesteps=1024 if args.mode == "smoke" else args.steps)
    elapsed = time.perf_counter() - started
    after = torch.cat([p.detach().flatten() for p in model.policy.parameters()])
    delta = float(torch.linalg.vector_norm(after - before))
    assert delta > 0
    trained = evaluate(model)
    model.save(out / "policy.zip")
    loaded = PPO.load(out / "policy.zip", device="cpu")
    repeated = evaluate(loaded)
    assert repeated == trained, "Saved policy evaluation differs from in-memory policy"
    improvement = trained["mean_return"] > baseline["mean_return"] * 2
    result = {"status": "PASS" if args.mode == "smoke" or improvement else "ISSUES", "scope": "pipeline_smoke_only" if args.mode == "smoke" else "actual_PPO_train_and_heldout_eval",
              "training_seed": args.seed, "requested_steps": 1024 if args.mode == "smoke" else args.steps,
              "actual_steps": model.num_timesteps, "elapsed_training_s": elapsed,
              "device": "cpu", "torch_threads": 1, "contract": contract,
              "versions": {"python": platform.python_version(), "platform": platform.platform(),
                           "gymnasium": gym.__version__, "mujoco": mujoco.__version__,
                           "stable_baselines3": stable_baselines3.__version__, "torch": torch.__version__, "numpy": np.__version__},
              "baseline_zero": baseline, "untrained_policy": untrained, "trained_policy": trained,
              "parameter_change_l2": delta, "save_reload_eval_exact": repeated == trained,
              "greater_than_twice_zero_baseline": improvement,
              "limits": "Only MuJoCo inverted pendulum. No Microduck/Sapiens locomotion training or hardware validation."}
    (out / "report.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    env.close()
    print(json.dumps({k: result[k] for k in ["status", "scope", "training_seed", "actual_steps", "elapsed_training_s"]}))
    print(json.dumps({"zero": baseline["mean_return"], "untrained": untrained["mean_return"], "trained": trained["mean_return"]}))
    if args.mode == "train":
        assert improvement, "Learning not demonstrated: report saved; inspect and extend training"


if __name__ == "__main__":
    main()
