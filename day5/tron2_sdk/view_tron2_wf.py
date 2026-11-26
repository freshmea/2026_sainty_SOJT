"""Drive WF_TRON2A in MuJoCo with the downloaded LimX encoder and actor ONNX files."""
import argparse
from contextlib import contextmanager
import time

import mujoco
import mujoco.viewer
import numpy as np

from train_tron2_wf import WheelDrive, session


@contextmanager
def wf_viewer(env, enabled=True):
    if not enabled:
        yield None
        return
    env.model.geom_group[env.floor] = 2
    with mujoco.viewer.launch_passive(env.model, env.data, show_left_ui=False,
                                      show_right_ui=False) as viewer:
        with viewer.lock():
            viewer.opt.geomgroup[0] = 0
            viewer.opt.sitegroup[:] = 0
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = env.model.body("base_Link").id
            viewer.cam.lookat[:] = env.data.body("base_Link").xpos
            viewer.cam.distance = 2.5
            viewer.cam.elevation = -12
            viewer.cam.azimuth = 112
        yield viewer


def display_trial(viewer, title, status, period):
    if viewer is None:
        return
    if not viewer.is_running():
        raise KeyboardInterrupt("Viewer closed")
    viewer.set_texts((None, None, title, status))
    viewer.sync()
    time.sleep(period)


def play(speed):
    actor = session("policy.onnx")
    actor_input = actor.get_inputs()[0].name
    with WheelDrive() as env:
        obs, _ = env.reset(options={"speed": speed})
        period = env.substeps * env.model.opt.timestep
        wheels = env.q[env.wheel]
        start_x = env.data.qpos[0]
        start_angles = env.data.qpos[wheels].copy()
        with wf_viewer(env) as viewer:
            deadline = time.monotonic()
            while viewer.is_running():
                action = actor.run(None, {actor_input: obs})[0].ravel()
                obs, _, fallen, finished, _ = env.step(action)
                x = env.data.qpos[0] - start_x
                left, right = (env.data.qpos[wheels] - start_angles) / (2 * np.pi)
                viewer.set_texts((None, None,
                                  "LimX WF ONNX\nTime (s)\nCommand / actual vx (m/s)\nForward (m)\nWheel turns L / R",
                                  f"Episode\n{env.data.time:.1f} / 8.0\n{speed:.2f} / {env.data.qvel[0]:.2f}"
                                  f"\n{x:.2f}\n{left:.2f} / {right:.2f}"))
                viewer.sync()
                deadline = max(deadline + period, time.monotonic())
                time.sleep(max(0, deadline - time.monotonic()))
                if finished or fallen:
                    print(f"t={env.data.time:.2f}s, x={x:.3f}m, turns L/R={left:.2f}/{right:.2f}, "
                          f"fallen={fallen}", flush=True)
                    obs, _ = env.reset(options={"speed": speed})
                    start_x = env.data.qpos[0]
                    start_angles = env.data.qpos[wheels].copy()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speed", type=float, default=0.5, help="forward command (m/s)")
    args = parser.parse_args()
    if not 0 <= args.speed <= 0.7:
        parser.error("--speed must be between 0 and 0.7 m/s")
    play(args.speed)
