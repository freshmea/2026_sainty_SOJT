# -*- coding: utf-8 -*-
"""Educational legacy-position-actuator rollout, not the official BAM rehearsal."""
import argparse
import ast
import csv
import hashlib
import json
import platform
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort
from PIL import Image

ROOT = Path(__file__).resolve().parent
MICRO = ROOT.parent
SOURCE = MICRO / "sources/microduck_rl"
MODEL = SOURCE / "src/mjlab_microduck/robot/microduck"
INFERENCE = SOURCE / "scripts/infer_policy.py"


def source_pose():
    tree = ast.parse(INFERENCE.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign):
            if any(isinstance(t, ast.Name) and t.id == "DEFAULT_POSE" for t in node.targets):
                return np.asarray(ast.literal_eval(node.value.args[0]), dtype=np.float32)
    raise ValueError("Pinned official DEFAULT_POSE not found")


def observation(model, data, body_id, gyro_adr, position_indices,
                velocity_indices, pose, previous_action, command):
    quat = data.xquat[body_id].astype(np.float32)
    gravity = np.asarray([0, 0, -1], dtype=np.float32)
    cross = np.cross(quat[1:], gravity) * 2
    body_gravity = gravity - quat[0] * cross + np.cross(quat[1:], cross)
    values = np.concatenate([
        data.sensordata[gyro_adr:gyro_adr + 3],
        body_gravity,
        data.qpos[position_indices] - pose,
        data.qvel[velocity_indices],
        previous_action,
        command,
    ]).astype(np.float32)
    assert values.shape == (61,) and np.isfinite(values).all()
    return values


def run(no_render):
    results = ROOT / "results"
    results.mkdir(exist_ok=True)
    assets = {p.relative_to(MODEL).as_posix(): p.read_bytes()
              for p in MODEL.rglob("*") if p.is_file()}
    model = mujoco.MjModel.from_xml_string(
        (MODEL / "scene_walk.xml").read_text(encoding="utf-8"), assets=assets)
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "STAND")
    assert key >= 0
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)
    pose = source_pose()
    joint_ids = model.actuator_trnid[:, 0]
    position_indices = model.jnt_qposadr[joint_ids]
    velocity_indices = model.jnt_dofadr[joint_ids]
    joint_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(j)) for j in joint_ids]
    expected_names = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
                      "neck_pitch", "head_pitch", "head_yaw", "head_roll", "right_hip_yaw",
                      "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle"]
    assert joint_names == expected_names
    assert (model.nq, model.nv, model.nu) == (21, 20, 14)
    assert np.all(model.actuator_gainprm[:, 0] > 0)
    assert np.allclose(model.actuator_biasprm[:, 1], -model.actuator_gainprm[:, 0])
    assert np.isclose(model.opt.timestep, 0.002)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "trunk_base")
    gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_ang_vel")
    assert body_id >= 0 and gyro_id >= 0
    gyro_adr = int(model.sensor_adr[gyro_id])
    session = ort.InferenceSession((MICRO / "policies/alpha_walking.onnx").read_bytes(),
                                   providers=["CPUExecutionProvider"])
    input_meta, output_meta = session.get_inputs()[0], session.get_outputs()[0]
    assert input_meta.shape == [1, 61] and output_meta.shape == [1, 14]
    action = np.zeros(14, dtype=np.float32)
    command = np.zeros(13, dtype=np.float32)
    rows = []
    initial_xyz = data.xpos[body_id].copy()
    for control_step in range(300):
        command[0] = 0.0 if control_step < 100 else 0.1
        obs = observation(model, data, body_id, gyro_adr, position_indices,
                          velocity_indices, pose, action, command)
        action = session.run([output_meta.name], {input_meta.name: obs[None, :]})[0][0]
        assert action.shape == (14,) and np.isfinite(action).all()
        data.ctrl[:] = pose + action
        for _ in range(10):
            mujoco.mj_step(model, data)
            assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
        mujoco.mj_forward(model, data)
        rows.append({"time_s": float(data.time), "command_vx_m_s": float(command[0]),
                     "trunk_x_m": float(data.xpos[body_id, 0]),
                     "trunk_y_m": float(data.xpos[body_id, 1]),
                     "trunk_height_m": float(data.xpos[body_id, 2]),
                     "upright_cosine": float(data.xmat[body_id].reshape(3, 3)[2, 2]),
                     "vx_world_m_s": float(data.qvel[0]),
                     "action_abs_max": float(np.max(np.abs(action))),
                     "contacts": int(data.ncon)})
    with (results / "rollout.csv").open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    if not no_render:
        model.vis.global_.offwidth, model.vis.global_.offheight = 960, 720
        camera = mujoco.MjvCamera()
        camera.lookat[:] = data.xpos[body_id]
        camera.distance, camera.azimuth, camera.elevation = 0.65, 130, -18
        with mujoco.Renderer(model, height=720, width=960) as renderer:
            renderer.update_scene(data, camera=camera)
            Image.fromarray(renderer.render()).save(results / "microduck_after_6s.png")
    min_upright = min(r["upright_cosine"] for r in rows)
    min_height = min(r["trunk_height_m"] for r in rows)
    result = {
        "pipeline_status": "PASS",
        "scope": "educational actual-state ONNX legacy-position-actuator closed loop",
        "official_BAM_reproduction": False,
        "walking_performance_certified": False,
        "ppo_training_performed": False,
        "python": platform.python_version(), "mujoco": mujoco.__version__,
        "onnxruntime": ort.__version__, "numpy": np.__version__,
        "source_sha": "0daab160d3fb1c2cbaba16a4a19052f36709dd9e",
        "policy_revision": "236d1835b03ac7b45d962efbf2fe3a6940206112",
        "inference_source_sha256": hashlib.sha256(INFERENCE.read_bytes()).hexdigest(),
        "joint_names": joint_names, "default_pose": pose.tolist(),
        "physics_steps": 3000, "policy_calls": 300, "simulation_time_s": float(data.time),
        "physics_dt_s": 0.002, "control_dt_s": 0.02,
        "observation_size": 61, "action_size": 14, "action_scale": 1.0,
        "command_schedule": "0 m/s for 2s, then +0.1 m/s for 4s",
        "initial_trunk_xyz_m": initial_xyz.tolist(),
        "final_trunk_xyz_m": data.xpos[body_id].tolist(),
        "x_displacement_m": float(data.xpos[body_id, 0] - initial_xyz[0]),
        "min_trunk_height_m": min_height, "min_upright_cosine": min_upright,
        "commanded_interval_vx_rmse_m_s": float(np.sqrt(np.mean([
            (r["vx_world_m_s"] - r["command_vx_m_s"]) ** 2 for r in rows[100:]
        ]))),
        "commanded_interval_mean_vx_m_s": float(np.mean([r["vx_world_m_s"] for r in rows[100:]])),
        "tilt_beyond_60deg_observed": min_upright < 0.5,
        "all_state_values_finite": True,
        "actuator_model": "unchanged scene_walk legacy position actuators, not BAM M6",
        "limitations": ["single deterministic rollout", "no perturbation evaluation",
                        "alpha policy training lineage unverified", "different actuator model from current training"],
    }
    (results / "validation.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-render", action="store_true")
    run(parser.parse_args().no_render)
