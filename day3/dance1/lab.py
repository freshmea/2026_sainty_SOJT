"""CLI for the Sainty K1 Dance1 classroom experiment."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from dance_sim import ASSETS, LAB, DanceSimulation


def train(args):
    import gymnasium as gym
    import torch
    from stable_baselines3 import PPO

    class ResidualEnv(gym.Env):
        def __init__(self):
            self.sim = DanceSimulation()
            self.action_space = gym.spaces.Box(-1., 1., (23,), dtype=np.float32)
            self.observation_space = gym.spaces.Box(-np.inf, np.inf, (124,), dtype=np.float32)

        def reset(self, *, seed=None, options=None):
            super().reset(seed=seed)
            # Mix full starts with intermediate frames to expose later choreography.
            frame = 0 if self.np_random.random() < 0.5 else int(self.np_random.integers(0, 250))
            return self.sim.reset(frame, self.np_random.uniform(-2., 2.)), {}

        def step(self, action):
            obs, reward, fallen, last_frame, info = self.sim.advance(action)
            return obs, reward, fallen, last_frame and not fallen, info

    torch.set_num_threads(1)
    env = ResidualEnv()
    policy = PPO("MlpPolicy", env, n_steps=256, batch_size=64, n_epochs=5, seed=args.seed,
                 device="cpu", policy_kwargs={"net_arch": [64, 64]}, verbose=1)
    policy.learn(total_timesteps=args.steps, log_interval=8)
    args.checkpoint.parent.mkdir(parents=True, exist_ok=True)
    policy.save(args.checkpoint)
    print(f"Saved {policy.num_timesteps} steps to {args.checkpoint}", flush=True)


def controller(mode, checkpoint):
    if mode == "official":
        import onnxruntime as ort
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 1
        session = ort.InferenceSession(str(ASSETS / "policy.onnx"), opts, providers=["CPUExecutionProvider"])
        name = session.get_inputs()[0].name
        return lambda obs: session.run(None, {name: obs[None, :]})[0][0]
    if mode == "ppo":
        from stable_baselines3 import PPO
        policy = PPO.load(checkpoint, device="cpu")
        return lambda obs: policy.predict(obs, deterministic=True)[0]
    return lambda obs: np.zeros(23)


def evaluate(args):
    sim = DanceSimulation()
    obs = sim.reset(roll_deg=args.roll)
    act = controller(args.mode, args.checkpoint)
    poses = [sim.data.qpos.copy()]
    rows = []
    for tick in range(1, sim.frames):
        obs, _, fallen, last, info = sim.advance(act(obs), official=args.mode == "official")
        poses.append(sim.data.qpos.copy())
        rows.append((tick, tick * sim.dt, info["mse"] ** 0.5, info["height"]))
        if fallen or last:
            break
    args.out.mkdir(parents=True, exist_ok=True)
    stem = args.out / f"{args.mode}_roll{args.roll:g}"
    with Path(str(stem) + ".csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "time_s", "joint_rmse_rad", "pelvis_height_m"])
        w.writerows(rows)
    np.savez_compressed(str(stem) + ".npz", poses=poses, ref=sim.ref[:len(poses)], dt=sim.dt)
    result = dict(mode=args.mode, roll_deg=args.roll, frames_reached=len(rows), planned=sim.frames-1,
                  elapsed_s=round(len(rows)*sim.dt, 2), completed=bool(last and not fallen),
                  joint_rmse_rad=float(np.sqrt(np.mean(np.square(np.asarray(rows)[:, 2])))))
    Path(str(stem) + ".json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result))


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(required=True)
    t = sub.add_parser("train")
    t.add_argument("--steps", type=int, default=16384)
    t.add_argument("--seed", type=int, default=11)
    t.add_argument("--checkpoint", type=Path, default=LAB/"outputs/ppo.zip")
    t.set_defaults(fn=train)
    e = sub.add_parser("evaluate")
    e.add_argument("mode", choices=["pd", "ppo", "official"])
    e.add_argument("--roll", type=float, default=0)
    e.add_argument("--out", type=Path, default=LAB/"outputs")
    e.add_argument("--checkpoint", type=Path, default=LAB/"outputs/ppo.zip")
    e.set_defaults(fn=evaluate)
    args = p.parse_args()
    if hasattr(args, "steps") and args.steps <= 0:
        p.error("--steps must be positive")
    args.fn(args)


if __name__ == "__main__":
    main()
