"""선택 실습: 작은 MuJoCo 모델을 Gymnasium 계약으로 감싼다. 학습은 하지 않는다."""
import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces
from gymnasium.utils.env_checker import check_env
from common import load_model, save_json


class ArmEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self):
        self.model, self.data = load_model("arm.xml")
        self.action_space = spaces.Box(-1.0, 1.0, shape=(1,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(3,), dtype=np.float32)
        self.target = np.pi / 4
        self.steps = 0

    def _obs(self):
        return np.array([self.data.joint("hinge").qpos[0],
                         self.data.joint("hinge").qvel[0], self.target], dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.joint("hinge").qpos[0] = self.np_random.uniform(-0.1, 0.1)
        mujoco.mj_forward(self.model, self.data)
        self.steps = 0
        return self._obs(), {}

    def step(self, action):
        torque = 2.0 * float(np.clip(action[0], -1, 1))
        self.data.actuator("motor").ctrl[0] = torque
        # 0.002초 x 10 = 0.020초마다 새 행동을 계산한다.
        for _ in range(10):
            mujoco.mj_step(self.model, self.data)
        self.steps += 1
        obs = self._obs()
        error = float(self.target - obs[0])
        reward = -(error**2 + 0.01 * float(obs[1])**2 + 0.001 * torque**2)
        terminated = bool(abs(obs[0]) > np.pi)
        truncated = bool(self.steps >= 200)
        return obs, float(reward), terminated, truncated, {"error_rad": error}


def evaluate(mode, seeds):
    results = []
    env = ArmEnv()
    try:
        for seed in seeds:
            obs, _ = env.reset(seed=seed)
            total = 0.0
            for _ in range(200):
                if mode == "PD":
                    torque = 4.0 * (float(obs[2])-float(obs[0])) - 0.6 * float(obs[1])
                    action = np.array([np.clip(torque/2.0, -1, 1)], dtype=np.float32)
                else:
                    action = np.zeros(1, dtype=np.float32)
                obs, reward, terminated, truncated, info = env.step(action)
                total += reward
                if terminated or truncated:
                    break
            results.append({"seed": seed, "return": total, "final_error_rad": info["error_rad"],
                            "terminated": terminated, "truncated": truncated})
    finally:
        env.close()
    return results


if __name__ == "__main__":
    env = ArmEnv()
    check_env(env, skip_render_check=True)
    env.close()
    results = {mode: evaluate(mode, [11, 22, 33, 44, 55]) for mode in ["zero", "PD"]}
    save_json("07_gym_metrics.json", results)
    for mode, episodes in results.items():
        print(mode, "mean return:", np.mean([e["return"] for e in episodes]))
    print("PASS: Gymnasium contract checked; controllers evaluated, no RL training.")
