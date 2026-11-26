"""Fine-tune the released WF actor in MuJoCo, using its frozen history encoder."""
import argparse
import json
from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np
import onnxruntime as ort
import yaml

ROOT = Path(__file__).resolve().parent
WEIGHTS = ROOT / "vendor/tron2_rl_deploy_python/controllers/model/WF_TRON2A"
XML = ROOT / "vendor/tron2_mujoco_sim/robot-description/tron2a/WF_TRON2A/xml/robot.xml"


def session(name):
    opts = ort.SessionOptions()
    opts.intra_op_num_threads = opts.inter_op_num_threads = 1
    return ort.InferenceSession(str(WEIGHTS / name), opts, providers=["CPUExecutionProvider"])


class WheelDrive(gym.Env):
    """Flat-ground forward velocity tracking, 50 Hz policy and 1 kHz physics."""

    def __init__(self):
        cfg = yaml.safe_load((WEIGHTS / "params.yaml").read_text())["PointfootCfg"]
        control = cfg["control"]
        self.model = mujoco.MjModel.from_xml_path(str(XML))
        self.data = mujoco.MjData(self.model)
        names = cfg["init_state"]["joint_names"]
        self.act = np.array([self.model.actuator(n).id for n in names])
        joints = self.model.actuator_trnid[self.act, 0]
        self.q = self.model.jnt_qposadr[joints]
        self.v = self.model.jnt_dofadr[joints]
        self.rest = np.array([cfg["init_state"]["default_joint_angle"][n] for n in names])
        self.wheel = np.array(control["joint_control_modes"]) == "velocity"
        self.kp, self.kd, self.limit = np.array([
            (0., control["wheel_joint_damping"], control["wheel_joint_torque_limit"])
            if wheel else tuple(control[prefix + k] for k in ("_stiffness", "_damping", "_torque_limit"))
            for wheel, prefix in zip(self.wheel, ["light_joint" if "yaw" in n else "leg_joint" for n in names])
        ]).T
        self.pos_scale = control["action_scale_pos"]
        self.vel_scale = control["action_scale_vel"]
        self.substeps = round(control["decimation"] / cfg["loop_frequency"] / self.model.opt.timestep)
        self.encoder = session("encoder.onnx")
        self.encoder_input = self.encoder.get_inputs()[0].name
        self.history = np.zeros((10, 34), dtype=np.float32)
        self.last = np.zeros(10, dtype=np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (40,), np.float32)
        self.action_space = gym.spaces.Box(-100., 100., (10,), np.float32)
        self.floor = self.model.geom("ground").id
        self.wheel_bodies = self.model.jnt_bodyid[joints[self.wheel]]

    def observe(self, first=False):
        d = self.data
        gravity = -d.site("base_imu").xmat.reshape(3, 3)[2]
        raw = np.r_[0.25*d.sensor("base_imu_gyro").data, gravity,
                    (d.qpos[self.q]-self.rest)[~self.wheel], 0.05*d.qvel[self.v], self.last]
        if first:
            self.history[:] = raw
        else:
            self.history[:-1] = self.history[1:]
            self.history[-1] = raw
        latent = self.encoder.run(None, {self.encoder_input: self.history.ravel()})[0]
        return np.r_[latent.ravel(), raw.clip(-100, 100), self.speed, 0, 0].astype(np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.speed = float((options or {}).get("speed", self.np_random.uniform(.2, .6)))
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.q] = self.rest
        mujoco.mj_forward(self.model, self.data)
        # Start the upright wheels on the floor instead of dropping the XML spawn pose.
        wheel_id = self.model.body("wheel_L_Link").id
        geoms = np.flatnonzero((self.model.geom_bodyid == wheel_id) &
                               (self.model.geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER))
        g = geoms.item()
        self.data.qpos[2] -= self.data.geom_xpos[g, 2] - self.model.geom_size[g, 0]
        mujoco.mj_forward(self.model, self.data)
        self.last.fill(0)
        self.ticks = 0
        return self.observe(first=True), {}

    def step(self, action):
        d = self.data
        self.last[:] = np.clip(action, -100, 100)
        pos = self.rest + self.pos_scale * self.last
        vel = np.where(self.wheel, self.vel_scale * self.last, 0)
        for _ in range(self.substeps):
            torque = self.kp*(pos-d.qpos[self.q]) + self.kd*(vel-d.qvel[self.v])
            d.ctrl[self.act] = np.clip(torque, -self.limit, self.limit)
            mujoco.mj_step(self.model, d)
        mujoco.mj_forward(self.model, d)
        self.ticks += 1
        upright = d.body("base_Link").xmat[8]
        fell = bool(upright < .65 or d.qpos[2] < .40)
        reward = np.exp(-((d.qvel[0]-self.speed)/.35)**2) + .25*upright - 5*float(fell)
        return self.observe(), float(reward), fell, self.ticks >= 400, {}


def from_official(env, seed):
    import onnx
    import torch
    from stable_baselines3 import PPO
    from torch import nn

    graph = onnx.load(WEIGHTS / "policy.onnx").graph
    weights = {p.name: onnx.numpy_helper.to_array(p) for p in graph.initializer}
    nodes = list(graph.node)
    if [n.op_type for n in nodes] != ["MatMul", "Add", "Elu"]*3 + ["MatMul", "Add"]:
        raise ValueError("Unexpected official WF actor architecture")
    agent = PPO("MlpPolicy", env, learning_rate=1e-5, n_steps=256, batch_size=64, n_epochs=3,
                policy_kwargs=dict(net_arch=dict(pi=[512, 256, 128], vf=[128, 128]),
                                   activation_fn=nn.ELU, log_std_init=-1.5),
                seed=seed, device="cpu")
    layers = [m for m in agent.policy.mlp_extractor.policy_net if isinstance(m, nn.Linear)]
    with torch.no_grad():
        for index, layer in enumerate([*layers, agent.policy.action_net]):
            mul, add = nodes[3*index:3*index+2]
            bias = next(n for n in add.input if n in weights)
            layer.weight.copy_(torch.tensor(weights[mul.input[1]].T))
            layer.bias.copy_(torch.tensor(weights[bias]))
    official = session("policy.onnx")
    rng = np.random.default_rng(85)
    for _ in range(16):
        obs = rng.normal(size=40).astype(np.float32)
        expected = official.run(None, {official.get_inputs()[0].name: obs})[0]
        with torch.no_grad():
            actual = agent.policy.get_distribution(torch.from_numpy(obs[None])).mode()[0].numpy()
        np.testing.assert_allclose(actual, expected.ravel(), atol=1e-4, rtol=1e-5)
    return agent


def evaluate(agent, env, speed):
    obs, _ = env.reset(seed=101, options={"speed": speed})
    d = env.data
    start = d.qpos.copy()
    samples = []
    wheel_contact_ticks = np.zeros(2, dtype=int)
    while True:
        action, _ = agent.predict(obs, deterministic=True)
        obs, _, fallen, done, _ = env.step(action)
        samples.append(float(d.qvel[0]))
        pairs = d.contact.geom
        ground_geoms = pairs[pairs[:, 0] == env.floor, 1]
        wheel_contact_ticks += np.isin(env.wheel_bodies, env.model.geom_bodyid[ground_geoms])
        if done or fallen:
            break
    angles = d.qpos[env.q[env.wheel]] - start[env.q[env.wheel]]
    return dict(command_m_s=speed, seconds=round(d.time, 3), fallen=fallen,
                forward_m=float(d.qpos[0]-start[0]), lateral_m=float(d.qpos[1]-start[1]),
                wheel_turns=(angles/(2*np.pi)).tolist(),
                wheel_ground_fraction=(wheel_contact_ticks / len(samples)).tolist(),
                speed_rmse=float(np.sqrt(np.mean((np.array(samples)-speed)**2))))


def main():
    import torch
    from stable_baselines3 import PPO

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True, help="new directory")
    parser.add_argument("--steps", type=int, default=4096)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--resume", type=Path, help="continue a saved SB3 PPO ZIP")
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("--steps must be positive")
    torch.set_num_threads(1)
    args.out.mkdir(parents=True, exist_ok=False)
    env = WheelDrive()
    agent = PPO.load(args.resume, env=env, device="cpu") if args.resume else from_official(env, args.seed)
    speeds = (0., .3, .5)
    before = [evaluate(agent, env, speed) for speed in speeds]
    print("Before PPO:", before, flush=True)
    agent.learn(args.steps, reset_num_timesteps=not bool(args.resume))
    agent.save(args.out / "wf_ppo")
    restored = PPO.load(args.out / "wf_ppo.zip", device="cpu")
    after = [evaluate(restored, env, speed) for speed in speeds]
    report = dict(training_steps=agent.num_timesteps, before=before, after=after)
    (args.out / "evaluation.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
