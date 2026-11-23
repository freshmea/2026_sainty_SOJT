"""Side by side replay from an evaluation trace. Requires ffmpeg and EGL or OSMesa."""
import argparse
import shutil
import subprocess
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from dance_sim import DanceSimulation


def main():
    p = argparse.ArgumentParser()
    p.add_argument("trace", type=Path, help="NPZ written by official, pd or ppo")
    p.add_argument("--out", type=Path)
    args = p.parse_args()
    output = args.out or args.trace.with_suffix(".mp4")
    trace = np.load(args.trace)
    sim = DanceSimulation()
    if not shutil.which("ffmpeg"):
        p.error("Install ffmpeg: sudo apt install ffmpeg")
    renderer = mujoco.Renderer(sim.model, height=300, width=400)
    camera = mujoco.MjvCamera()
    camera.azimuth, camera.elevation, camera.distance = -110, -9, 2.1
    camera.lookat[:] = [*trace["ref"][0, :2], .65]
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 17)
    cmd = ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
           "-s", "800x344", "-r", "25", "-i", "-", "-c:v", "libx264", "-crf", "20",
           "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output)]
    with subprocess.Popen(cmd, stdin=subprocess.PIPE) as ff:
        refs, poses = trace["ref"], trace["poses"]
        for i in list(range(0, len(poses)-1, 2)) + [len(poses)-1]:
            canvas = Image.new("RGB", (800, 344), "#112d38")
            for col, pose in enumerate((refs[i], poses[i])):
                sim.data.qpos[:] = pose
                mujoco.mj_forward(sim.model, sim.data)
                renderer.update_scene(sim.data, camera=camera)
                canvas.paste(Image.fromarray(renderer.render()), (col*400, 44))
            draw = ImageDraw.Draw(canvas)
            draw.text((16, 11), f"Reference  /  {i*sim.dt:0.2f} s", font=font, fill="white")
            draw.text((416, 11), f"MuJoCo  /  {args.trace.stem}", font=font, fill="white")
            ff.stdin.write(canvas.tobytes())
        ff.stdin.close()
        if ff.wait():
            raise RuntimeError("ffmpeg failed")
    renderer.close()
    print(output)


if __name__ == "__main__":
    main()
