"""Small CPU velocity-tracking experiment on the official SF and WF MuJoCo models."""
import argparse
import json
from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np
from stable_baselines3 import PPO
import torch

ROOT = Path(__file__).resolve().parent
MODELS = ROOT.parent / "tron2_sdk/vendor/tron2_mujoco_sim/robot-description/tron2a"


class VelocityTask(gym.Env):
    def __init__(self, variant):
        model = mujoco.MjModel.from_xml_path(str(MODELS / variant / "xml/robot.xml"))
        self.model, self.data = model, mujoco.MjData(model)
        ids = model.actuator_trnid[:, 0]
        self.q, self.v = model.jnt_qposadr[ids], model.jnt_dofadr[ids]
        self.wheel = np.array(["wheel" in model.joint(int(j)).name.lower() for j in ids])
        self.action_space = gym.spaces.Box(-1., 1., (model.nu,), np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (4 + 2*model.nu,), np.float32)
        self.base = int(model.jnt_bodyid[0])

    def observe(self):
        d = self.data
        return np.r_[self.goal, d.qvel[0], d.xpos[self.base, 2], d.xmat[self.base, 8],
                     d.qpos[self.q]-self.rest, d.qvel[self.v]].astype(np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        m, d = self.model, self.data
        if m.nkey:
            mujoco.mj_resetDataKeyframe(m, d, 0)
        else:
            mujoco.mj_resetData(m, d)
        self.rest = d.qpos[self.q].copy()
        self.goal = float(self.np_random.choice([0.10, 0.20]))
        self.steps = 0
        mujoco.mj_forward(m, d)
        self.start_z = d.xpos[self.base, 2]
        return self.observe(), {}

    def step(self, action):
        m, d = self.model, self.data
        action = np.clip(action, -1., 1.)
        for _ in range(20):
            torque = 35*(self.rest + 0.15*action-d.qpos[self.q])-1.5*d.qvel[self.v]
            torque[self.wheel] = 1.5*(4*action[self.wheel]-d.qvel[self.v[self.wheel]])
            d.ctrl[:] = np.clip(torque, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1])
            mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)
        error = float(self.goal - d.qvel[0])
        tilt = float(d.xmat[self.base, 8])
        fallen = bool(tilt < 0.65 or d.xpos[self.base, 2] < 0.6*self.start_z)
        reward = float(1 - 2*abs(error) - 0.02*np.mean(action**2) - 3*fallen)
        self.steps += 1
        return self.observe(), reward, fallen, self.steps >= 100, {"error": error}


def evaluate(env, agent):
    rows = []
    for seed in (101, 102, 103):
        obs, _ = env.reset(seed=seed)
        errors, total = [], 0.
        while True:
            action = np.zeros(env.action_space.shape) if agent is None else agent.predict(obs, deterministic=True)[0]
            obs, reward, term, trunc, info = env.step(action)
            errors.append(info["error"])
            total += reward
            if term or trunc:
                break
        rows.append({"seed": seed, "goal_m_s": env.goal, "steps": len(errors), "fall": term,
                     "return": total, "vx_rmse_m_s": float(np.sqrt(np.mean(np.square(errors))))})
    return rows


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("variant", choices=["SF_TRON2A","WF_TRON2A"])
    p.add_argument("--steps", type=int, default=1024)
    p.add_argument("--width", type=int, choices=(16, 32, 64, 128), default=32,
                   help="units in each of the two actor and critic hidden layers")
    p.add_argument("--out", type=Path, required=True)
    args = p.parse_args()
    if args.steps < 1:
        p.error("positive steps required")
    args.out.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(1)
    env = VelocityTask(args.variant)
    agent = PPO("MlpPolicy", env, n_steps=128, batch_size=64, n_epochs=5,
                policy_kwargs={"net_arch": [args.width, args.width]}, seed=7, device="cpu")
    report = {"hidden_width": args.width, "variant": args.variant, "wheel_actuators": int(env.wheel.sum()),
              "obs_dim": env.observation_space.shape[0], "dt": env.model.opt.timestep,
              "zero": evaluate(env, None), "before": evaluate(env, agent)}
    agent.learn(total_timesteps=args.steps)
    agent.save(args.out/"policy.zip")
    report["after"] = evaluate(env, agent)
    report["actual_steps"] = agent.num_timesteps
    assert evaluate(env, PPO.load(args.out/"policy.zip", device="cpu")) == report["after"]
    (args.out/"evaluation.json").write_text(json.dumps(report, indent=2))
    print(report["after"])
