"""Classroom task: track a yaw target while the remaining servos hold STAND."""
import gymnasium as gym
import mujoco
import numpy as np
from inspect_model import standing_model

class HeadTarget(gym.Env):
    def __init__(self, smooth=0.02):
        self.model, self.data = standing_model()
        self.smooth = smooth
        self.action_space = gym.spaces.Box(-1., 1., (1,), np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (4,), np.float32)

    def observe(self):
        joint = self.data.joint("head_yaw")
        return np.array([self.goal, joint.qpos[0], joint.qvel[0], self.previous], np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("STAND").id)
        mujoco.mj_forward(self.model, self.data)
        self.goal = self.np_random.choice([-0.25, 0.25])
        self.previous = 0.
        self.steps = 0
        return self.observe(), {}

    def step(self, action):
        command = float(np.clip(action[0], -1., 1.))
        self.data.actuator("head_yaw").ctrl[0] = 0.35 * command
        mujoco.mj_step(self.model, self.data, nstep=10)
        mujoco.mj_forward(self.model, self.data)
        error = float(self.goal - self.data.joint("head_yaw").qpos[0])
        change = (command - self.previous) ** 2
        self.previous = command
        self.steps += 1
        fallen = bool(self.data.body("trunk_base").xmat.reshape(3, 3)[2, 2] < 0.5)
        reward = -(error / 0.25)**2 - self.smooth * change - 5. * fallen
        return self.observe(), reward, fallen, self.steps >= 50, {"error_rad": error}

if __name__ == "__main__":
    from stable_baselines3.common.env_checker import check_env
    env = HeadTarget()
    check_env(env)
    print(env.reset(seed=7), env.step(np.array([0.5], np.float32)))
