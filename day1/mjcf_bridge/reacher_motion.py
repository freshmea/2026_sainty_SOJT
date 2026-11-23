"""Compare forward kinematics with a short pulse of motor torque."""
from pathlib import Path

import mujoco
import numpy as np

MODEL = Path(__file__).with_name("reacher.xml")


def experiment():
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    poses = []
    for shoulder, wrist in [(0, 0), (0, 90), (90, 0), (30, -60)]:
        a, b = np.deg2rad([shoulder, wrist])
        data.joint("shoulder").qpos[0] = a
        data.joint("wrist").qpos[0] = b
        mujoco.mj_forward(model, data)
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
    data.actuator("shoulder").ctrl[0] = 0.2
    mujoco.mj_step(model, data, nstep=5)
    assert np.isfinite(data.qvel).all() and np.linalg.norm(data.qvel) > 0
    np.testing.assert_allclose(data.time, 5 * model.opt.timestep)
    assert (model.nq, model.nv, model.nu) == (2, 2, 2)
    return {"shape": [model.nq, model.nv, model.nu], "addresses_q_v_u": controls,
            "poses": poses, "pulse_seconds": data.time, "pulse_qvel": data.qvel.tolist()}


if __name__ == "__main__":
    import json
    print(json.dumps(experiment(), indent=2))
