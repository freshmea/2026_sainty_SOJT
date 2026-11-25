"""Compare forward kinematics with a short pulse of motor torque."""
import argparse
import json
import time
from pathlib import Path

import mujoco
import numpy as np

MODEL = Path(__file__).with_name("reacher.xml")


POSES = [(0, 0), (0, 90), (90, 0), (30, -60)]
PULSE_STEPS = 5
SHOULDER_CTRL = 0.2


def set_pose(model, data, shoulder, wrist):
    data.joint("shoulder").qpos[0] = np.deg2rad(shoulder)
    data.joint("wrist").qpos[0] = np.deg2rad(wrist)
    mujoco.mj_forward(model, data)


def experiment():
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    poses = []
    for shoulder, wrist in POSES:
        a, b = np.deg2rad([shoulder, wrist])
        set_pose(model, data, shoulder, wrist)
        expected = [.12 * (np.cos(a) + np.cos(a+b)),
                    .12 * (np.sin(a) + np.sin(a+b)), .01]
        actual = data.geom("finger").xpos.copy()
        np.testing.assert_allclose(actual, expected, atol=1e-8, rtol=0)
        poses.append({"degrees": [shoulder, wrist], "finger_m": actual.tolist()})
    assert data.time == 0.0
    controls = {}
    for name in ("shoulder", "wrist"):
        motor = model.actuator(name).id
        joint = model.joint(name).id
        assert model.actuator_trnid[motor, 0] == joint
        controls[name] = [int(model.jnt_qposadr[joint]), int(model.jnt_dofadr[joint]), motor]
    mujoco.mj_resetData(model, data)
    data.actuator("shoulder").ctrl[0] = SHOULDER_CTRL
    mujoco.mj_step(model, data, nstep=PULSE_STEPS)
    assert np.isfinite(data.qvel).all() and np.linalg.norm(data.qvel) > 0
    np.testing.assert_allclose(data.time, PULSE_STEPS * model.opt.timestep)
    assert (model.nq, model.nv, model.nu) == (2, 2, 2)
    return {"shape": [model.nq, model.nv, model.nu], "addresses_q_v_u": controls,
            "poses": poses, "pulse_seconds": data.time, "pulse_qvel": data.qvel.tolist()}


def show_motion():
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    dt = model.opt.timestep
    with mujoco.viewer.launch_passive(model, data) as viewer:
        with viewer.lock():
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = model.camera("fixed").id

        while viewer.is_running():
            mujoco.mj_resetData(model, data)
            for shoulder, wrist in POSES:
                set_pose(model, data, shoulder, wrist)
                until = time.monotonic() + 1.0
                while time.monotonic() < until:
                    if not viewer.is_running():
                        return
                    viewer.sync()
                    time.sleep(dt)

            mujoco.mj_resetData(model, data)
            deadline = time.monotonic()
            for step in range(round(2.0 / dt)):
                if not viewer.is_running():
                    return
                data.actuator("shoulder").ctrl[0] = SHOULDER_CTRL if step < PULSE_STEPS else 0
                mujoco.mj_step(model, data)
                viewer.sync()
                deadline += dt
                time.sleep(max(0.0, deadline - time.monotonic()))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-viewer", action="store_true", help="run numerical checks only")
    args = parser.parse_args()
    print(json.dumps(experiment(), indent=2))
    if not args.no_viewer:
        show_motion()
