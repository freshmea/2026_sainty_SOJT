"""A minimal Gymnasium task on the same hinge used in the PD lab."""
from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np


class HingeTarget(gym.Env):
    def __init__(self):
        xml = Path(__file__).resolve().parent / "models/teaching_hinge.xml"
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data = mujoco.MjData(self.model)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (1,), np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (3,), np.float32)

    def observe(self):
        return np.array([self.goal, self.data.qpos[0], self.data.qvel[0]], dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        self.goal = float(self.np_random.choice([-0.3, 0.3]))
        self.steps = 0
        return self.observe(), {}

    def step(self, action):
        torque = 2.0 * float(np.clip(action[0], -1.0, 1.0))
        self.data.ctrl[0] = torque
        mujoco.mj_step(self.model, self.data, nstep=5)
        self.steps += 1
        error = self.goal - float(self.data.qpos[0])
        reward = -error**2
        return self.observe(), reward, False, self.steps >= 100, {"error_rad": error}


if __name__ == "__main__":
    from gymnasium.utils.env_checker import check_env

    env = HingeTarget()
    check_env(env, skip_render_check=True)
    for name in ("zero", "pd"):
        obs, _ = env.reset(seed=7)
        squared_errors = []
        for step in range(100):
            goal, angle, speed = obs
            torque = 0.0 if name == "zero" else 12 * (goal - angle) - 0.8 * speed
            action = np.array([np.clip(torque / 2, -1, 1)], dtype=np.float32)
            obs, reward, terminated, truncated, info = env.step(action)
            squared_errors.append(info["error_rad"]**2)
        assert truncated and not terminated
        print(name, "steps", step + 1, "rmse_rad", float(np.sqrt(np.mean(squared_errors))))
