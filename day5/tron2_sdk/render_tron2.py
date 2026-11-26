"""Render the four classroom TRON2A models in their initial MJCF poses."""
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent
MODELS = ROOT / "vendor/tron2_mujoco_sim/robot-description/tron2a"
OUTPUT = ROOT / "results/renders"
VARIANTS = {
    "SF": "robot.xml",
    "WF": "robot.xml",
    "DA": "robot.xml",
    "DACH": "robot_grasper.xml",
}


def render_variant(variant, xml):
    name = f"{variant}_TRON2A"
    model = mujoco.MjModel.from_xml_path(str(MODELS / name / "xml" / xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # Frame only the visual links. The MJCF also contains a 200 m ground plane.
    visual = model.geom_group == 1
    rotation = data.geom_xmat[visual].reshape(-1, 3, 3)
    boxes = model.geom_aabb[visual]
    centers = data.geom_xpos[visual] + (rotation @ boxes[:, :3, None]).squeeze(-1)
    radii = (np.abs(rotation) @ boxes[:, 3:, None]).squeeze(-1)
    low = (centers - radii).min(axis=0)
    high = (centers + radii).max(axis=0)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = (low + high) / 2
    camera.azimuth = 145
    camera.elevation = -12
    camera.distance = 0.66 * np.linalg.norm(high - low) / np.tan(np.deg2rad(model.vis.global_.fovy / 2))

    # Group 0 also contains collision proxies. Show only the floor and visual meshes.
    model.geom_group[model.geom("ground").id] = 2
    options = mujoco.MjvOption()
    options.geomgroup[:] = 0
    options.geomgroup[1:3] = 1
    options.sitegroup[:] = 0
    width, height = 1024, 1024
    model.vis.global_.offwidth = width
    model.vis.global_.offheight = height
    with mujoco.Renderer(model, width=width, height=height) as renderer:
        renderer.update_scene(data, camera=camera, scene_option=options)
        output = OUTPUT / f"{name}.png"
        Image.fromarray(renderer.render()).save(output)
    print(output, flush=True)


if __name__ == "__main__":
    OUTPUT.mkdir(parents=True, exist_ok=True)
    for variant, xml in VARIANTS.items():
        render_variant(variant, xml)
