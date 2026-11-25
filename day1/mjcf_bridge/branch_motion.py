"""Separate a change of pose from passive motion in a branched pendulum."""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np


MODEL = Path(__file__).with_name("chaotic_pendulum.xml")


def experiment():
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    ids = [model.body(str(i)).id for i in (1, 2, 3)]
    parent = model.body("0").id
    assert all(model.body_parentid[i] == parent for i in ids)
    geoms = model.body_geomadr[ids]
    mujoco.mj_forward(model, data)
    before = data.geom_xpos[geoms].copy()
    child_joint = model.body_jntadr[ids[0]]
    data.qpos[model.jnt_qposadr[child_joint]] = 0.25
    mujoco.mj_forward(model, data)
    shifts = np.linalg.norm(data.geom_xpos[geoms] - before, axis=1)
    assert shifts[0] > 0.01
    np.testing.assert_allclose(shifts[1:], 0, atol=1e-12)
    assert data.time == 0.0
    mujoco.mj_resetData(model, data)
    data.joint("root").qpos[0] = 0.25
    mujoco.mj_forward(model, data)
    start = data.qpos.copy()
    mujoco.mj_step(model, data, nstep=100)
    assert (model.nq, model.nv, model.nu) == (4, 4, 0)
    assert np.isfinite(data.qpos).all() and np.all(np.abs(data.qpos - start) > 1e-5)
    np.testing.assert_allclose(data.time, 0.1)
    return {"shape": [model.nq, model.nv, model.nu], "children": [1, 2, 3],
            "child_geom_shifts_m": shifts.tolist(), "initial_rad": start.tolist(),
            "after_100_steps_rad": data.qpos.tolist(), "seconds": data.time}


def show_motion():
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    # Release the tilted main bar from rest. Gravity drives all four hinges.
    data.joint("root").qpos[0] = np.deg2rad(40)
    mujoco.mj_forward(model, data)
    model.vis.global_.azimuth = 90
    model.vis.global_.elevation = 0
    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-viewer", action="store_true", help="run numerical checks only")
    args = parser.parse_args()
    print(json.dumps(experiment(), indent=2))
    if not args.no_viewer:
        show_motion()
