"""Watch one Dance1 episode in a WSLg GLFW window."""
import argparse
import time
import glfw
import mujoco
from dance_sim import DanceSimulation, LAB
from lab import controller


def main():
    p = argparse.ArgumentParser()
    p.add_argument("mode", choices=["reference", "pd", "official", "ppo"])
    p.add_argument("--roll", type=float, default=0.)
    p.add_argument("--checkpoint", default=LAB/"outputs/ppo.zip")
    args = p.parse_args()
    sim = DanceSimulation()
    obs = sim.reset(roll_deg=args.roll)
    act = controller(args.mode, args.checkpoint)
    if not glfw.init():
        p.exit(2, "WSLg display unavailable. Run an evaluation and use the video command.\n")
    try:
        window = glfw.create_window(760, 500, f"K1 Dance1 | {args.mode}", None, None)
        if not window:
            p.exit(2, "Cannot open a GLFW window. Use the video command.\n")
        glfw.make_context_current(window)
        glfw.swap_interval(0)
        ctx = mujoco.MjrContext(sim.model, mujoco.mjtFontScale.mjFONTSCALE_100)
        scene = mujoco.MjvScene(sim.model, maxgeom=2000)
        opt, camera = mujoco.MjvOption(), mujoco.MjvCamera()
        camera.lookat[:] = [*sim.ref[0, :2], .65]
        camera.azimuth, camera.elevation, camera.distance = -110, -9, 2.1
        for frame in range(sim.frames):
            if glfw.window_should_close(window):
                break
            tick = time.monotonic()
            done = False
            if args.mode == "reference":
                sim.data.qpos[:] = sim.ref[frame]
                mujoco.mj_forward(sim.model, sim.data)
            elif frame:
                obs, _, done, _, _ = sim.advance(act(obs), official=args.mode == "official")
            mujoco.mjv_updateScene(sim.model, sim.data, opt, None, camera,
                                    mujoco.mjtCatBit.mjCAT_ALL, scene)
            w, h = glfw.get_framebuffer_size(window)
            mujoco.mjr_render(mujoco.MjrRect(0, 0, w, h), scene, ctx)
            glfw.swap_buffers(window)
            glfw.poll_events()
            time.sleep(max(0., sim.dt - (time.monotonic() - tick)))
            if done:
                print(f"Stop threshold reached at {frame * sim.dt:.2f} s")
                break
        print(f"Last displayed frame: {frame}")
        ctx.free()
        glfw.destroy_window(window)
    finally:
        glfw.terminate()


if __name__ == "__main__":
    main()
