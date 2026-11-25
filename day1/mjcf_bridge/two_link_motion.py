"""Give the two-link planar arm an initial push and watch its free motion."""
from math import radians
from pathlib import Path

import mujoco
import mujoco.viewer


def show_motion():
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name("two_link_minimal.xml")))
    data = mujoco.MjData(model)

    # Both hinge axes are vertical. An initial angular velocity sets the arm moving.
    data.joint("wrist").qpos[0] = radians(65)
    data.joint("shoulder").qvel[0] = radians(60)
    data.joint("wrist").qvel[0] = radians(-90)
    mujoco.mj_forward(model, data)

    # Center the top view on the shoulder so the tip stays visible through a full turn.
    model.stat.center[:] = data.body("upper").xpos
    model.stat.extent = 0.48
    model.vis.global_.azimuth = 90
    model.vis.global_.elevation = -90
    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    show_motion()
