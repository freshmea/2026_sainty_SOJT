# -*- coding: utf-8 -*-
"""CPU MuJoCo model, ONNX contract, and educational policy rollout checks."""
import argparse
from collections import deque
import csv
import json
from pathlib import Path
import platform
import sys

import mujoco
import numpy as np
import onnxruntime as ort
from PIL import Image
import yaml

parser = argparse.ArgumentParser()
parser.add_argument('--seconds', type=float, default=6.0)
parser.add_argument('--no-render', action='store_true')
parser.add_argument('--ros2', action='store_true', help='publish simulated joint and IMU samples')
parser.add_argument('--output-dir', type=Path, help='write simulation measurements here')
args = parser.parse_args()
if not 0 < args.seconds <= 60:
    parser.error('--seconds must be in (0, 60]')
root = Path(__file__).resolve().parent
source = root / 'sources' / 'ai_sapiens'
description = source / 'ai_sapiens_description'
scene = description / 'mujoco' / 'k1' / 'scene.xml'
policy_dir = source / 'ai_sapiens_sim2real/assets/k1/locomotion/velocity/walk_default'
cfg = yaml.safe_load((policy_dir / 'params/sim2real.yaml').read_text(encoding='utf-8'))
assets = {p.name: p.read_bytes() for p in description.rglob('*') if p.suffix in ('.stl', '.xml')}
model = mujoco.MjModel.from_xml_string(scene.read_text(encoding='utf-8'), assets=assets)
data = mujoco.MjData(model)
out = args.output_dir or root / 'validation'
out.mkdir(parents=True, exist_ok=True)
joints = cfg['policy_joints']
joint_ids = np.array([model.joint(n).id for n in joints])
qpos_ids = model.jnt_qposadr[joint_ids]
dof_ids = model.jnt_dofadr[joint_ids]
actuator_ids = np.array([model.actuator(n + '_motor').id for n in joints])
default = np.array([cfg['joint_properties'][n]['default_position'] for n in joints])
kp = np.array([cfg['joint_properties'][n]['stiffness'] for n in joints])
kd = np.array([cfg['joint_properties'][n]['damping'] for n in joints])
session = ort.InferenceSession((policy_dir / 'exported/policy.onnx').read_bytes(), providers=['CPUExecutionProvider'])
input_info = session.get_inputs()[0]
output_info = session.get_outputs()[0]
obs_dim = sum(len(v['scale']) * v['history_length'] for v in cfg['observations'].values())
assert input_info.shape[-1] == obs_dim, (input_info.shape, obs_dim)
zero_output = session.run(None, {input_info.name: np.zeros((1, obs_dim), dtype=np.float32)})[0]
assert zero_output.shape == (1, len(joints)) and np.isfinite(zero_output).all()
invalid_dimension_rejected = False
try:
    session.run(None, {input_info.name: np.zeros((1, obs_dim - 1), dtype=np.float32)})
except ort.capi.onnxruntime_pybind11_state.InvalidArgument:
    invalid_dimension_rejected = True
assert invalid_dimension_rejected
mujoco.mj_resetDataKeyframe(model, data, 0)
for _ in range(100):
    mujoco.mj_step(model, data)
assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
mujoco.mj_resetDataKeyframe(model, data, 0)
data.qpos[qpos_ids] = default
mujoco.mj_forward(model, data)
if not args.no_render:
    model.vis.global_.offwidth = 960
    model.vis.global_.offheight = 720
    renderer = mujoco.Renderer(model, height=720, width=960)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, 0.7]
    camera.distance = 2.8
    camera.azimuth = 135
    camera.elevation = -15
    renderer.update_scene(data, camera=camera)
    Image.fromarray(renderer.render()).save(out / 'k1_initial.png')
last_action = np.zeros(len(joints))
histories = {}
decimation = round(cfg['step_dt'] / model.opt.timestep)
assert np.isclose(decimation * model.opt.timestep, cfg['step_dt'])
if args.ros2:
    import time
    timing = []
    import rclpy
    sys.path.insert(0, str(root.parent / 'ros_lab'))
    from telemetry import Telemetry
    rclpy.init()
    from geometry_msgs.msg import Twist
    telemetry = Telemetry('k1')
    latest_cmd = [0.0, -float('inf')]
    def accept_twist(msg):
        if np.isfinite(msg.linear.x):
            latest_cmd[:] = [float(np.clip(msg.linear.x, -0.3, 0.3)), time.monotonic()]
    telemetry.create_subscription(Twist, '/k1/cmd_vel', accept_twist, 10)
rows = []
for step in range(round(args.seconds / cfg['step_dt'])):
    command = np.array([0.0 if data.time < 2.0 else 0.3, 0.0, 0.0])
    if args.ros2:
        rclpy.spin_once(telemetry, timeout_sec=0)
        command[0] = latest_cmd[0] if time.monotonic() - latest_cmd[1] < 0.5 else 0.0
    rotation = data.xmat[model.body('pelvis').id].reshape(3, 3)
    terms = {
        'base_ang_vel': data.sensor('imu_gyro').data.copy(),
        'projected_gravity': rotation.T @ np.array([0.0, 0.0, -1.0]),
        'velocity_commands': command,
        'joint_pos_rel': data.qpos[qpos_ids] - default,
        'joint_vel_rel': data.qvel[dof_ids].copy(),
        'last_action': last_action.copy(),
    }
    for name, spec in cfg['observations'].items():
        value = terms[name] * np.asarray(spec['scale'])
        if name not in histories:
            histories[name] = deque([value.copy() for _ in range(spec['history_length'])], maxlen=spec['history_length'])
        else:
            histories[name].append(value)
    observation = np.concatenate([np.concatenate(histories[name]) for name in cfg['observations']]).astype(np.float32)[None, :]
    if args.ros2:
        infer_start = time.perf_counter_ns()
    action = session.run(None, {input_info.name: observation})[0][0]
    if args.ros2:
        infer_end = time.perf_counter_ns()
    assert np.isfinite(action).all()
    action_cfg = cfg['actions']['joint_pos']
    action = np.clip(action, np.array(action_cfg['raw_clip'])[:, 0], np.array(action_cfg['raw_clip'])[:, 1])
    target = action * np.array(action_cfg['scale']) + np.array(action_cfg['offset'])
    for _ in range(decimation):
        torque = kp * (target - data.qpos[qpos_ids]) - kd * data.qvel[dof_ids]
        limits = model.actuator_ctrlrange[actuator_ids]
        data.ctrl[actuator_ids] = np.clip(torque, limits[:, 0], limits[:, 1])
        mujoco.mj_step(model, data)
    if args.ros2:
        mujoco.mj_forward(model, data)
        stamp = round(data.time * 1e9)
        publish_start = time.perf_counter_ns()
        telemetry.joint_sample(stamp, joints, data.qpos[qpos_ids], data.qvel[dof_ids], data.actuator_force[actuator_ids])
        publish_end = time.perf_counter_ns()
        timing.append((step, infer_end - infer_start, publish_start - infer_end,
                       publish_end - publish_start, publish_end - infer_start))
        telemetry.imu_sample(stamp, 'imu', data.sensor('imu_quat').data,
                             data.sensor('imu_gyro').data, data.sensor('imu_acc').data)
        time.sleep(cfg['step_dt'])
    last_action = action
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    rows.append(dict(time=float(data.time), base_height=float(data.qpos[2]), x=float(data.qpos[0]), command_x=float(command[0]), velocity_x=float(data.qvel[0]), contacts=int(data.ncon)))
if args.ros2:
    with (out / 'joint_publish_timing.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(('tick', 'onnx_ns', 'target_pd_physics_forward_ns',
                         'joint_message_publish_ns', 'onnx_to_publish_return_ns'))
        writer.writerows(timing)
    telemetry.destroy_node()
    rclpy.shutdown()
with (out / 'policy_rollout.csv').open('w', newline='', encoding='utf-8') as handle:
    writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)
if not args.no_render:
    camera.lookat[:] = [data.qpos[0], data.qpos[1], max(data.qpos[2], 0.5)]
    renderer.update_scene(data, camera=camera)
    Image.fromarray(renderer.render()).save(out / 'k1_after_rollout.png')
    renderer.close()
summary = dict(
    status='PASS', scope='model loading, finite integration, ONNX contract, educational CPU rollout; not training convergence or official ROS deployment',
    python=sys.version, platform=platform.platform(), mujoco=mujoco.__version__, numpy=np.__version__, onnxruntime=ort.__version__,
    model=dict(nq=model.nq, nv=model.nv, nu=model.nu, nbody=model.nbody, nmesh=model.nmesh, physics_dt=model.opt.timestep),
    policy=dict(input_name=input_info.name, input_shape=input_info.shape, output_name=output_info.name, output_shape=output_info.shape, observation_dimension=obs_dim, action_dimension=len(joints), control_dt=cfg['step_dt'], decimation=decimation, invalid_dimension_rejected=invalid_dimension_rejected),
    rollout=dict(steps=len(rows), seconds=float(data.time), min_base_height=min(r['base_height'] for r in rows), final_base_height=float(data.qpos[2]), displacement_x=float(data.qpos[0]), all_finite=True, upright_height_above_0_55_all_steps=bool(min(r['base_height'] for r in rows)>0.55)),
    source_commit='937c67d66db1015a73a359f2c742943810a74e3a',
)
(out / 'validation.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(summary, ensure_ascii=False, indent=2))
