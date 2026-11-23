"""Train, save, reload, and compare three policies on paired target episodes."""
import argparse
import json
from pathlib import Path
import numpy as np
import torch
from stable_baselines3 import PPO
from head_env import HeadTarget

def evaluate(env, agent):
    errors, returns, falls = [], [], 0
    for seed in range(100, 110):
        obs, _ = env.reset(seed=seed)
        total = 0.
        while True:
            action = agent(obs)
            obs, reward, fallen, timeout, info = env.step(action)
            errors.append(info["error_rad"]**2)
            total += reward
            if fallen or timeout:
                falls += int(fallen)
                returns.append(total)
                break
    return dict(rmse_rad=float(np.sqrt(np.mean(errors))), falls=falls,
                mean_return=float(np.mean(returns)))

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--steps", type=int, default=4096)
    p.add_argument("--smooth", type=float, default=0.02)
    args = p.parse_args()
    torch.set_num_threads(1)
    env = HeadTarget(args.smooth)
    agent = PPO("MlpPolicy", env, seed=7, device="cpu", n_steps=128, batch_size=64,
                learning_rate=0.001, policy_kwargs={"net_arch":[32, 32]})
    out = Path("results") / f"steps{args.steps}_smooth{args.smooth}"
    out.mkdir(parents=True, exist_ok=True)
    agent.learn(total_timesteps=args.steps)
    agent.save(out / "ppo")
    agent = PPO.load(out / "ppo", device="cpu")
    policies = {"zero": lambda obs: np.zeros(1), "target": lambda obs: obs[:1]/0.35,
                "ppo": lambda obs: agent.predict(obs, deterministic=True)[0]}
    results = {name: evaluate(env, policy) for name, policy in policies.items()}
    report = {"training_steps": agent.num_timesteps, "training_seed": 7,
              "evaluation_seeds": [100,109], "smooth": args.smooth, "results": results}
    (out/"metrics.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
