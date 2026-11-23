"""Apply zero action and the saved policy to the same initial observation."""
import argparse
from pathlib import Path
import numpy as np
from stable_baselines3 import PPO
from pendulum_env import make_env


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", type=Path, default=Path(__file__).parent / "results/ppo_seed11/policy.zip")
    parser.add_argument("--seed", type=int, default=9000)
    args = parser.parse_args()
    policy = PPO.load(args.policy, device="cpu")
    with make_env() as env:
        initial, _ = env.reset(seed=args.seed)
        predicted, _ = policy.predict(initial, deterministic=True)
        for name, action in [("zero", np.zeros_like(predicted)), ("trained", predicted)]:
            obs, _ = env.reset(seed=args.seed)
            np.testing.assert_array_equal(obs, initial)
            after, reward, term, trunc, _ = env.step(action)
            print(name, "obs", obs.round(5), "action", action.round(5), "next", after.round(5),
                  "reward", reward, "terminated", term, "truncated", trunc)


if __name__ == "__main__":
    main()
