"""MuJoCo-only reset and recovery hand-off shared by training, evaluation and ROS."""

import math
import mujoco
import numpy as np


def place_fallen(env, side, angle=0.0, yaw=0.0):
    env.reset(options={"speed": 0.0})
    d = env.data
    pitch = {"front": math.pi / 2, "back": -math.pi / 2}[side] + angle
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    d.qpos[3:7] = [cy * cp, -sy * sp, cy * sp, sy * cp]
    d.qpos[2] = 0.48
    mujoco.mj_forward(env.model, d)
    d.ctrl[:] = 0
    mujoco.mj_step(env.model, d, nstep=round(1 / env.model.opt.timestep))
    mujoco.mj_forward(env.model, d)
    env.last.fill(0)
    return env.observe(first=True)


class RecoveryPreparation:
    """Settle a real simulated fall; after a failed attempt also reposition the legs."""

    def __init__(self, env, *, retry):
        dt = env.model.opt.timestep * env.substeps
        self.settle_end = round(1.0 / dt)
        self.fold_end = self.settle_end + (round(0.9 / dt) if retry else 0)
        self.end = self.fold_end + (round(0.3 / dt) if retry else 0)
        self.ticks = 0
        self.fold = np.zeros_like(env.last)
        if retry:
            # WheelDrive maps normalized actions to rest + pos_scale * action.
            for index, actuator in enumerate(env.act):
                name = env.model.actuator(int(actuator)).name
                if name in ("proximal_pitch_L", "proximal_pitch_R"):
                    self.fold[index] = (1.0 - env.rest[index]) / env.pos_scale
                elif name in ("knee_L", "knee_R"):
                    self.fold[index] = (-1.2 - env.rest[index]) / env.pos_scale

    def step(self, env):
        env.speed = 0.0
        folding = self.settle_end <= self.ticks < self.fold_end
        self.ticks += 1
        ready = self.ticks == self.end
        if folding:
            obs, *_ = env.step(self.fold)
        else:
            env.data.ctrl.fill(0)
            mujoco.mj_step(env.model, env.data, nstep=env.substeps)
            mujoco.mj_forward(env.model, env.data)
            env.last.fill(0)
            obs = env.observe(first=ready)
        return obs, ready


class Recovery:
    def __init__(self, obs):
        self.side = 1.0 if obs[6] >= 0 else -1.0
        self.ticks = 0
        self.balance = False
        self.held = 0

    def input(self, obs):
        return np.r_[obs, self.side, min(self.ticks // 17, 2)].astype(np.float32)

    def update(self, env, fallen):
        self.ticks += 1
        base = env.data.body("base_Link")
        up, z = base.xmat[8], base.xpos[2]
        if self.balance:
            self.held = self.held + 1 if up > 0.93 and z > 0.58 else 0
            if self.held >= 100:
                return "recovered"
            if fallen:
                return "failed"
        elif up > 0.74 and z > 0.51:
            self.balance = True
        if self.ticks >= 230 or (not self.balance and self.ticks >= 81):
            return "failed"
        return "running"
