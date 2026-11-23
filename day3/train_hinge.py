"""Train a small PPO on the Day 2 hinge with the same physical action limits."""
import argparse
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO
from hinge_env import HingeTarget


def evaluate(model):
    errors = []
    with HingeTarget() as env:
        for seed in range(100, 110):
            obs, _ = env.reset(seed=seed)
            while True:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = env.step(action)
                errors.append(info["error_rad"]**2)
                if terminated or truncated:
                    break
    return float(np.sqrt(np.mean(errors)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=4096)
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error("--steps must be positive")
    torch.set_num_threads(1)
    agent = PPO("MlpPolicy", HingeTarget(), n_steps=128, batch_size=64,
                policy_kwargs={"net_arch": [32, 32]}, seed=7, device="cpu")
    print("before_rmse_rad", evaluate(agent))
    agent.learn(total_timesteps=args.steps)
    path = Path(__file__).resolve().parent / f"results/hinge_steps{agent.num_timesteps}/policy.zip"
    path.parent.mkdir(parents=True, exist_ok=True)
    agent.save(path)
    reloaded = PPO.load(path, device="cpu")
    after = evaluate(agent)
    assert np.isclose(after, evaluate(reloaded))
    print("after_rmse_rad", after, "steps", agent.num_timesteps, "saved", path)
