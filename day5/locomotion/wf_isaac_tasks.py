"""Classroom variants of the official Isaac WF task. Imported after Kit starts."""

import json
from pathlib import Path
import torch
from isaaclab.managers import RewardTermCfg
from isaaclab.utils import configclass
from bipedal_locomotion.tasks.locomotion.robots.limx_wheelfoot_tron2a_env_cfg import (
    WF_TRON2A_BlindFlatEnvCfg,
    WF_TRON2A_BlindFlatEnvCfg_PLAY,
)


def set_forward_command(cfg, speed_range):
    cmd = cfg.commands.base_velocity
    cmd.heading_command = False
    cmd.rel_heading_envs = 0.0
    cmd.rel_standing_envs = 0.0
    cmd.ranges.lin_vel_x = speed_range
    cmd.ranges.ang_vel_z = (0.0, 0.0)


def center_height_error(env):
    robot = env.scene["robot"]
    mass = robot.root_physx_view.get_masses().to(env.device)
    z = (robot.data.body_com_pos_w[..., 2] * mass).sum(dim=1) / mass.sum(dim=1)
    command = env.command_manager.get_command("base_velocity")[:, 0]
    error = (command - robot.data.root_lin_vel_b[:, 0]).abs()
    flex = (error / 2).clamp(max=1) * (command.abs() / 2).clamp(max=1)
    return (z - env.scene.env_origins[:, 2] - (0.628 - 0.029 * flex)).abs()


def high_speed(cfg):
    set_forward_command(cfg, (0.5, 5.0))
    cfg.rewards.base_height = RewardTermCfg(func=center_height_error, weight=-5.0)


def reset_fallen(env, env_ids):
    # Initialize from physical, torque-free falls. The complete recovery rollout then runs in Isaac.
    robot = env.scene["robot"]
    if not hasattr(env, "_wf_initial_states"):
        data = json.loads(
            Path(__file__).with_name("wf_fallen_samples.json").read_text()
        )
        names = data["joint_names"]
        joint_ids = [robot.joint_names.index(name) for name in names]
        samples = torch.tensor(
            [
                [
                    r["height"],
                    *r["wxyz"],
                    *r["linear"],
                    *r["angular"],
                    *r["joint_pos"],
                    *r["joint_vel"],
                ]
                for r in data["states"]
            ],
            device=env.device,
        )
        env._wf_initial_states = (joint_ids, samples)
    joint_ids, samples = env._wf_initial_states
    state = samples[torch.randint(len(samples), (len(env_ids),), device=env.device)]
    position = env.scene.env_origins[env_ids].clone()
    position[:, 2] += state[:, 0]
    robot.write_root_link_pose_to_sim(
        torch.cat((position, state[:, 1:5]), dim=1), env_ids=env_ids
    )
    robot.write_root_link_velocity_to_sim(state[:, 5:11], env_ids=env_ids)
    robot.write_joint_state_to_sim(
        state[:, 11:21], state[:, 21:31], joint_ids=joint_ids, env_ids=env_ids
    )


def upright_reward(env):
    return -env.scene["robot"].data.projected_gravity_b[:, 2]


def recovery(cfg):
    set_forward_command(cfg, (0.0, 0.0))
    cfg.events.reset_robot_base.func = reset_fallen
    cfg.events.reset_robot_base.params = {}
    cfg.events.reset_robot_joints = None
    cfg.terminations.base_contact = None
    cfg.terminations.bad_orientation = None
    cfg.rewards.undesired_contacts = None
    cfg.rewards.stand_still = None
    cfg.rewards.flat_orientation_l2 = RewardTermCfg(func=upright_reward, weight=4.0)
    cfg.episode_length_s = 4.6


@configclass
class HighSpeed(WF_TRON2A_BlindFlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        high_speed(self)


@configclass
class HighSpeedPlay(WF_TRON2A_BlindFlatEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()
        high_speed(self)


@configclass
class Recovery(WF_TRON2A_BlindFlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        recovery(self)


@configclass
class RecoveryPlay(WF_TRON2A_BlindFlatEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()
        recovery(self)
