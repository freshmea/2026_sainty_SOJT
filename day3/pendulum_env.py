"""Keep the official task while loading its XML safely from Korean Windows paths."""
from pathlib import Path
import os
import gymnasium as gym
from gymnasium.envs.registration import EnvSpec
from gymnasium.envs.mujoco.inverted_pendulum_v5 import InvertedPendulumEnv
import mujoco


class UnicodePathPendulum(InvertedPendulumEnv):
    def _initialize_simulation(self):
        model = mujoco.MjModel.from_xml_string(Path(self.fullpath).read_text(encoding="utf-8"))
        model.vis.global_.offwidth = self.width
        model.vis.global_.offheight = self.height
        return model, mujoco.MjData(model)


def make_env(**kwargs):
    if os.name == "nt":
        spec = EnvSpec(id="InvertedPendulum-v5", entry_point=UnicodePathPendulum,
                       max_episode_steps=1000)
        return gym.make(spec, **kwargs)
    return gym.make("InvertedPendulum-v5", **kwargs)
