"""Dance1 reference sampling and 50 Hz closed-loop simulation."""
import json
from pathlib import Path

import mujoco
import numpy as np

LAB = Path(__file__).resolve().parent
ASSETS = LAB / "assets"
SCENE = LAB.parents[1] / "day1/K1/sources/ai_sapiens/ai_sapiens_description/mujoco/k1/scene.xml"


class DanceSimulation:
    def __init__(self):
        cfg = json.loads((ASSETS / "dance1.json").read_text())
        self.dt = cfg["step_dt"]
        self.model = mujoco.MjModel.from_xml_path(str(SCENE))
        self.data = mujoco.MjData(self.model)
        self.substeps = round(self.dt / self.model.opt.timestep)
        assert np.isclose(self.substeps * self.model.opt.timestep, self.dt)
        names = cfg["policy_joints"]
        self.jpos = np.array([self.model.joint(n).qposadr.item() for n in names])
        self.jvel = np.array([self.model.joint(n).dofadr.item() for n in names])
        self.motors = np.array([self.model.actuator(n + "_motor").id for n in names])
        self.limits = self.model.actuator_ctrlrange[self.motors]
        self.torso = self.model.body("torso_link").id
        data = np.loadtxt(ASSETS / "dance1.csv", delimiter=",")
        self.frames = len(data)
        self.ref = np.tile(self.model.qpos0, (self.frames, 1))
        self.ref[:, :3] = data[:, :3]
        self.ref[:, 3:7] = data[:, [6, 3, 4, 5]]
        self.ref[:, self.jpos] = data[:, [7 + cfg["csv_joints"].index(n) for n in names]]
        self.refvel = np.empty((self.frames, self.model.nv))
        self.rotation = []
        tmp = mujoco.MjData(self.model)
        for frame, pose in enumerate(self.ref):
            tmp.qpos[:] = pose
            mujoco.mj_forward(self.model, tmp)
            self.rotation.append(tmp.xmat[self.torso].reshape(3, 3).copy())
            nxt = min(frame + 1, self.frames - 1)
            mujoco.mj_differentiatePos(self.model, self.refvel[frame], self.dt, pose, self.ref[nxt])
        self.refvel[-1] = self.refvel[-2]
        props = cfg["joint_properties"]
        self.kp = np.array([props[n]["stiffness"] for n in names])
        self.kd = np.array([props[n]["damping"] for n in names])
        self.offset = np.array(cfg["actions"]["joint_pos"]["offset"])
        self.scale = np.array(cfg["actions"]["joint_pos"]["scale"])
        self.reset()

    def reset(self, frame=0, roll_deg=0.0):
        mujoco.mj_resetData(self.model, self.data)
        self.frame = frame
        self.data.qpos[:] = self.ref[frame]
        angle = np.deg2rad(roll_deg) / 2
        quat = np.zeros(4)
        mujoco.mju_mulQuat(quat, self.data.qpos[3:7], np.array([np.cos(angle), np.sin(angle), 0., 0.]))
        self.data.qpos[3:7] = quat
        self.data.qvel[:] = self.refvel[frame]
        self.last = np.zeros(23)
        mujoco.mj_forward(self.model, self.data)
        return self.observe()

    def observe(self):
        i = self.frame
        relative = self.data.xmat[self.torso].reshape(3, 3).T @ self.rotation[i]
        return np.concatenate((self.ref[i, self.jpos], self.refvel[i, self.jvel],
                               relative[:, :2].ravel(), self.data.sensor("imu_gyro").data,
                               self.data.qpos[self.jpos] - self.offset,
                               self.data.qvel[self.jvel], self.last)).astype(np.float32)

    def advance(self, action, official=False):
        """Official actions specify a target relative to neutral, classroom actions correct the reference."""
        action = np.asarray(action).reshape(23)
        self.last = action.copy()
        nxt = min(self.frame + 1, self.frames - 1)
        target = self.offset + self.scale * action if official else self.ref[nxt, self.jpos] + 0.15 * action
        for _ in range(self.substeps):
            torque = self.kp * (target - self.data.qpos[self.jpos]) - self.kd * self.data.qvel[self.jvel]
            self.data.ctrl[self.motors] = np.clip(torque, self.limits[:, 0], self.limits[:, 1])
            mujoco.mj_step(self.model, self.data)
        self.frame = nxt
        mujoco.mj_forward(self.model, self.data)
        error = self.data.qpos[self.jpos] - self.ref[nxt, self.jpos]
        mse = float(np.mean(error ** 2))
        height = float(self.data.qpos[2])
        upright = float(self.data.xmat[self.torso, 8])
        fallen = height < 0.45 or upright < 0.5
        reward = float(np.exp(-12 * mse) + 0.5 * upright - 0.02 * np.mean(action ** 2) - 5 * fallen)
        return self.observe(), reward, fallen, nxt == self.frames - 1, {"mse": mse, "height": height}
