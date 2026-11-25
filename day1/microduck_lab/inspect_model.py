"""Print the joint/actuator worksheet for the pinned scene, with STAND applied."""
import argparse
from pathlib import Path
import time
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
SCENE = ROOT / "sources/microduck_rl/src/mjlab_microduck/robot/microduck/scene_walk.xml"

def standing_model(*, full_body_contact=False):
    if full_body_contact:
        # The walking scene only collides with the ground through the soles.
        spec = mujoco.MjSpec.from_file(str(SCENE.with_name("scene_allcollisions.xml")))
        spec.option.timestep = 0.001  # Resolve impacts at 1 ms during falls.
        floor = spec.geom("floor")
        floor.conaffinity |= 2  # Include the trunk's power support.
        floor.priority = 1
        floor.solref = [0.004, 1.0]
        floor.margin = 0.002  # Begin contact 2 mm before the visible surface.
        model = spec.compile()
    else:
        model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("STAND").id)
    mujoco.mj_forward(model, data)
    return model, data

def print_worksheet(model, data):
    print(f"nq={model.nq} nv={model.nv} nu={model.nu} dt={model.opt.timestep}")
    print("STAND root xyz + wxyz:", data.qpos[:7])
    print("ctrl  actuator -> joint             qpos qvel   angle(rad) target(rad)")
    for i in range(model.nu):
        joint = model.actuator_trnid[i, 0]
        q = model.jnt_qposadr[joint]
        v = model.jnt_dofadr[joint]
        print(i, model.actuator(i).name, "->", model.joint(joint).name,
              q, v, round(data.qpos[q], 6), round(data.ctrl[i], 6))
    for i in range(model.nsensor):
        print("sensor", model.sensor(i).name, "address", model.sensor_adr[i],
              "width", model.sensor_dim[i])


def configure_camera(camera):
    camera.lookat[:] = (0.01, 0, 0.12)
    camera.distance = 0.52
    camera.azimuth = 152
    camera.elevation = -18


def render_poses(model):
    from PIL import Image

    output = Path(__file__).with_name("results")
    output.mkdir(exist_ok=True)
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    configure_camera(camera)
    width, height = 960, 720
    model.vis.global_.offwidth = width
    model.vis.global_.offheight = height
    with mujoco.Renderer(model, width=width, height=height) as renderer:
        for name in ("STAND", "INIT"):
            mujoco.mj_resetDataKeyframe(model, data, model.key(name).id)
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera)
            path = output / f"microduck_{name.lower()}.png"
            Image.fromarray(renderer.render()).save(path)
            print(path, flush=True)


def show_motion(model, data):
    """Drive the position servos with random targets while MuJoCo simulates contact."""
    import mujoco.viewer

    rng = np.random.default_rng()
    joint_ids = model.actuator_trnid[:, 0]
    lower, upper = model.jnt_range[joint_ids].T
    next_change = data.time
    steps_per_frame = max(1, round(1 / (60 * model.opt.timestep)))
    frame_period = steps_per_frame * model.opt.timestep

    with mujoco.viewer.launch_passive(model, data) as viewer:
        with viewer.lock():
            configure_camera(viewer.cam)
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = model.body("trunk_base").id

        deadline = time.monotonic()
        while viewer.is_running():
            if data.time >= next_change:
                data.ctrl[:] = rng.uniform(lower, upper)
                next_change = data.time + 0.8
            mujoco.mj_step(model, data, nstep=steps_per_frame)
            viewer.sync()
            deadline = max(deadline + frame_period, time.monotonic())
            time.sleep(max(0.0, deadline - time.monotonic()))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--render", action="store_true", help="save STAND and INIT as PNGs")
    parser.add_argument("--viewer", action="store_true", help="simulate random joint motion in the GUI")
    args = parser.parse_args()
    model, data = standing_model(full_body_contact=args.viewer)
    print_worksheet(model, data)
    if args.render:
        render_poses(model)
    if args.viewer:
        show_motion(model, data)
