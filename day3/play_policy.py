"""Replay a checkpoint in MuJoCo, or save a headless rendered frame."""
from pathlib import Path
import argparse
import gymnasium as gym
from stable_baselines3 import PPO
from PIL import Image
from pendulum_env import make_env

parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, default=11)
parser.add_argument("--snapshot", action="store_true")
args = parser.parse_args()
root = Path(__file__).resolve().parent
env = make_env(render_mode="rgb_array" if args.snapshot else "human", width=960, height=720)
policy = PPO.load(root / "results" / f"ppo_seed{args.seed}" / "policy.zip", device="cpu")
obs, _ = env.reset(seed=9000)
for step in range(1000):
    action, _ = policy.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, _ = env.step(action)
    if args.snapshot and step == 100:
        Image.fromarray(env.render()).save(root / "results" / "ppo_simulation.png")
        break
    if terminated or truncated:
        obs, _ = env.reset(seed=9000)
env.close()
