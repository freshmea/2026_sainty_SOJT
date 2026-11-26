"""Train bounded residual PPO policies on the classroom WF MuJoCo simulation."""

import argparse
import json
import math
from pathlib import Path
import sys

import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from torch import nn

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "tron2_sdk"))
from train_tron2_wf import WheelDrive, session
from wf_task_runtime import Recovery, place_fallen


class ActionMap(nn.Module):
    """Differentiable reference actions and bounded, learned feedback corrections."""

    def __init__(self, task, crouch):
        super().__init__()
        self.task, self.crouch = task, crouch
        if task == "recovery":
            poses = json.loads((HERE / "wf_recovery_priors.json").read_text())
            self.register_buffer("poses", torch.tensor([poses["back"], poses["front"]]))

    def forward(self, x, residual):
        r = torch.clamp(residual, -1.0, 1.0)
        if self.task == "recovery":
            pose = self.poses[(x[..., 40] > 0).long(), x[..., 41].long().clamp(0, 2)]
            hip, knee, wheel, spread = pose.unbind(-1)
            hip = 4 * hip + 0.045 * r[..., 0]
            knee = 4 * knee + 0.045 * r[..., 1]
            wheel = wheel + 0.6 * r[..., 2]
            spread = 4 * spread + 0.025 * r[..., 3]
            zero = torch.zeros_like(hip)
            return torch.stack(
                [hip, spread, zero, knee, wheel, hip, -spread, zero, knee, wheel], -1
            )
        target, yaw = x[..., 37], x[..., 39]
        left, right = 20 * x[..., 21], 20 * x[..., 26]
        error = target - 0.05 * (left + right)
        # Only flex during a longitudinal speed mismatch. A wheel radius is 0.10 m.
        bend = (
            self.crouch
            * (1 + 0.15 * r[..., 0])
            * (error.abs() / 2).clamp(0, 1)
            * (target.abs() / 2).clamp(0, 1)
        )
        pitch_ref = (0.060 * error).clamp(-0.22, 0.22)
        trim = 0.013 * target.clamp(-1, 1) + 0.004 * r[..., 1]
        wheel = 62 * (x[..., 6] - pitch_ref + trim) + 52 * x[..., 4] + 0.35 * r[..., 2]
        turn = 2 * (yaw - 4 * x[..., 5]) + 0.03 * r[..., 3]
        zero = torch.zeros_like(target)
        return torch.stack(
            [
                2 * bend,
                zero,
                zero,
                -4 * bend,
                left + wheel - turn,
                2 * bend,
                zero,
                zero,
                -4 * bend,
                right + wheel + turn,
            ],
            -1,
        )


class TrainingEnv(gym.Env):
    def __init__(self, task, crouch, target):
        self.task, self.max_speed = task, target
        self.sim = WheelDrive()
        self.base_id = self.sim.model.body("base_Link").id
        self.reference = ActionMap(task, crouch)
        self.stand = session("policy.onnx")
        self.stand_input = self.stand.get_inputs()[0].name
        self.action_space = gym.spaces.Box(-1.0, 1.0, (4,), np.float32)
        self.observation_space = gym.spaces.Box(
            -np.inf, np.inf, (42 if task == "recovery" else 40,), np.float32
        )

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if self.task == "recovery":
            side = self.np_random.choice(["front", "back"])
            self.obs = place_fallen(
                self.sim,
                side,
                self.np_random.uniform(-0.04, 0.04),
                self.np_random.uniform(-0.4, 0.4),
            )
            self.recovery = Recovery(self.obs)
        else:
            self.goal = self.np_random.uniform(0.5, self.max_speed)
            self.ramp = self.np_random.uniform(1.5, 3.0)
            self.obs, _ = self.sim.reset(options={"speed": 0.0})
        return self.input(), {}

    def input(self):
        if self.task == "recovery":
            return self.recovery.input(self.obs)
        self.sim.speed = self.goal * np.clip(
            (self.sim.data.time - 0.5) / self.ramp, 0, 1
        )
        self.obs[-3:] = [self.sim.speed, 0, 0]
        return self.obs.copy()

    def step(self, residual):
        x = self.input()
        if self.task == "recovery" and self.recovery.balance:
            action = self.stand.run(None, {self.stand_input: self.obs})[0].ravel()
        else:
            with torch.no_grad():
                action = self.reference(
                    torch.from_numpy(x), torch.as_tensor(residual)
                ).numpy()
        self.obs, _, fallen, _, _ = self.sim.step(action)
        d = self.sim.data
        up = d.xmat[self.base_id, 8]
        if self.task == "recovery":
            status = self.recovery.update(self.sim, fallen)
            reward = (
                0.5 * up
                + d.subtree_com[self.base_id, 2]
                + (15 if status == "recovered" else -5 if status == "failed" else 0)
            )
            done, timeout = status != "running", False
        else:
            forward = float(d.xmat[self.base_id].reshape(3, 3)[:, 0] @ d.qvel[:3])
            stage = min(abs(self.sim.speed - forward) / 2, 1) * min(
                abs(self.sim.speed) / 2, 1
            )
            zref = 0.628 - 0.029 * stage * (self.reference.crouch / 0.35)
            reward = (
                math.exp(-(((forward - self.sim.speed) / 0.9) ** 2))
                + 0.5 * up
                - 5 * abs(d.subtree_com[self.base_id, 2] - zref)
                - 0.015 * np.sum(residual**2)
                - 10 * float(fallen)
            )
            done, timeout = fallen, self.sim.ticks >= 550
        return self.input(), float(reward), done, timeout, {}

    def close(self):
        self.sim.close()


class ExportActor(nn.Module):
    def __init__(self, agent, reference):
        super().__init__()
        self.net = nn.Sequential(
            agent.policy.mlp_extractor.policy_net, agent.policy.action_net
        )
        self.reference = reference

    def forward(self, obs):
        return self.reference(obs, self.net(obs))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", choices=["recovery", "high_speed"], required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=32768)
    parser.add_argument("--target", type=float, default=5.0)
    parser.add_argument("--crouch", type=float, default=0.35)
    parser.add_argument("--seed", type=int, default=73)
    parser.add_argument("--resume", type=Path)
    args = parser.parse_args()
    if (
        args.steps < 1
        or not math.isfinite(args.target)
        or args.target < 0.5
        or not 0 <= args.crouch <= 1
    ):
        parser.error(
            "steps must be positive, target must be at least 0.5 m/s, and crouch must be in [0,1] rad"
        )
    args.out.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(1)
    env = TrainingEnv(args.task, args.crouch, args.target)
    try:
        agent = (
            PPO.load(args.resume, env=env, device="cpu")
            if args.resume
            else PPO(
                "MlpPolicy",
                env,
                seed=args.seed,
                learning_rate=2e-4,
                n_steps=256,
                batch_size=64,
                n_epochs=4,
                policy_kwargs=dict(net_arch=[48, 48], log_std_init=-0.7),
                device="cpu",
            )
        )
        agent.learn(
            args.steps,
            reset_num_timesteps=not bool(args.resume),
            callback=CheckpointCallback(
                save_freq=8192, save_path=str(args.out / "checkpoints")
            ),
        )
        agent.save(args.out / "ppo")
        model = ExportActor(agent, env.reference).eval()
        example = torch.from_numpy(env.reset(seed=400)[0])
        path = args.out / "policy.onnx"
        torch.onnx.export(
            model,
            example,
            str(path),
            input_names=["obs"],
            output_names=["actions"],
            opset_version=17,
        )
        ort = session(path.resolve())
        rng = np.random.default_rng(104)
        for _ in range(32):
            x = example + torch.tensor(
                rng.normal(0, 0.1, example.shape), dtype=torch.float32
            )
            np.testing.assert_allclose(
                model(x).detach().numpy(),
                ort.run(None, {"obs": x.numpy()})[0],
                atol=3e-5,
                rtol=1e-5,
            )
        metadata = dict(
            task=args.task,
            steps=agent.num_timesteps,
            seed=args.seed,
            target=args.target,
            crouch_rad=args.crouch,
            torch_onnx_checks=32,
            input_size=example.numel(),
        )
        (args.out / "training.json").write_text(json.dumps(metadata, indent=2) + "\n")
        print(metadata, flush=True)
    finally:
        env.close()


if __name__ == "__main__":
    main()
