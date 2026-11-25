"""Save a PNG of each standalone classroom model, or of one selected XML."""
import argparse
import os
from pathlib import Path

# EGL can render offscreen even when WSL has no desktop session.
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
from PIL import Image

HERE = Path(__file__).resolve().parent


def render(model_path, output_dir):
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    fixed = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "fixed")
    camera = mujoco.MjvCamera()
    if fixed < 0:
        mujoco.mjv_defaultFreeCamera(model, camera)
        camera.elevation = -90  # Look down on the planar two-link arm.

    width, height = 800, 600
    model.vis.global_.offwidth = width
    model.vis.global_.offheight = height
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / model_path.with_suffix(".png").name
    with mujoco.Renderer(model, height=height, width=width) as renderer:
        renderer.update_scene(data, camera=fixed if fixed >= 0 else camera)
        Image.fromarray(renderer.render()).save(path)
    return path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path, nargs="?", help="path to a standalone MJCF XML")
    parser.add_argument("--output-dir", type=Path, default=HERE / "results")
    args = parser.parse_args()
    models = [args.model] if args.model else sorted(HERE.glob("*.xml"))
    for model_path in models:
        print(render(model_path.resolve(), args.output_dir))
