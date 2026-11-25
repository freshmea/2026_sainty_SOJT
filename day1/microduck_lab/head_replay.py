"""저장된 SB3 머리 정책을 실제 HeadTarget 환경에서 다시 실행한다."""
import math
from pathlib import Path
import subprocess
import time

import mujoco
from PIL import Image, ImageDraw, ImageFont
from stable_baselines3 import PPO

from head_env import HeadTarget

LABELS = {"early": "초기", "middle": "중기", "late": "말기"}


def camera_pose(camera):
    camera.lookat[:] = (0.01, 0, 0.13)
    camera.distance, camera.azimuth, camera.elevation = 0.52, 152, -18


def episode(env, agent, seed):
    """영상과 GUI가 동일한 관측 → 결정적 행동 → 물리 스텝 경로를 사용한다."""
    obs, _ = env.reset(seed=seed)
    yield obs, 0.0, "시작"
    while True:
        action, _ = agent.predict(obs, deterministic=True)
        obs, reward, fallen, timeout, _ = env.step(action)
        status = "넘어짐 종료" if fallen else ("시간 종료" if timeout else "추종 중")
        yield obs, reward, status
        if fallen or timeout:
            return


def show(env, agent, seeds):
    import mujoco.viewer
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        camera_pose(viewer.cam)
        for seed in seeds:
            for obs, _, status in episode(env, agent, seed):
                if not viewer.is_running():
                    return
                viewer.sync()
                if status == "시작" or status.endswith("종료"):
                    print(f"seed {seed} | {status} | 목표 {math.degrees(obs[0]):+.1f}° | "
                          f"실제 {math.degrees(obs[1]):+.1f}°", flush=True)
                time.sleep(0.08)  # 0.02초 물리 구간을 4배 느리게 관찰한다.
            time.sleep(0.6)


def record_stage(out, stage, env, agent, seeds):
    path = out / f"{stage}.mp4"
    # 완성된 영상만 원래 이름으로 교체한다. FFmpeg에는 RGB 픽셀을 직접 전송한다.
    temp = path.with_suffix(".partial.mp4")
    args = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo",
            "-pix_fmt", "rgb24", "-s", "960x540", "-r", "25", "-i", "-", "-an",
            "-c:v", "libx264", "-preset", "veryfast", "-crf", "18", "-threads", "2",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(temp)]
    camera = mujoco.MjvCamera()
    camera_pose(camera)
    font_dir = Path("/usr/share/fonts/truetype/nanum")
    title = ImageFont.truetype(str(font_dir / "NanumGothicBold.ttf"), 27)
    text = ImageFont.truetype(str(font_dir / "NanumGothic.ttf"), 19)
    small = ImageFont.truetype(str(font_dir / "NanumGothic.ttf"), 15)
    with mujoco.Renderer(env.model, width=640, height=480) as renderer:
        with subprocess.Popen(args, stdin=subprocess.PIPE) as proc:
            try:
                for seed in seeds:
                    angles = []
                    total = 0.
                    for obs, reward, status in episode(env, agent, seed):
                        target, angle = map(math.degrees, obs[:2])
                        angles.append(angle)
                        total += reward
                        renderer.update_scene(env.data, camera=camera)
                        frame = Image.new("RGB", (960, 540), "#f2f6f6")
                        frame.paste(Image.fromarray(renderer.render()), (0, 60))
                        d = ImageDraw.Draw(frame)
                        d.rectangle((0, 0, 960, 59), fill="#183846")
                        d.text((18, 12), f"Microduck  /  머리 방향 추종    {LABELS[stage]} · {agent.num_timesteps:,}스텝",
                               font=title, fill="white")
                        for i, line in enumerate([
                            f"평가 시드 {seed}   ·   0.25배속",
                            f"목표각      {target:+.2f}°",
                            f"실제각      {angle:+.2f}°",
                            f"오차          {target-angle:+.2f}°",
                            f"제어 명령  {float(obs[3]):+.3f}",
                            f"누적 보상  {total:.2f}",
                        ]):
                            d.text((660, 76+32*i), line, font=text, fill="#183846")
                        d.text((660, 280), "머리 yaw  /  목표와 실제", font=small, fill="#375666")
                        def xy(i, deg):
                            return 672+i*5.2, 357-deg*2.6
                        d.line((672,357,932,357), fill="#bcc9cc")
                        d.line((*xy(0,target), *xy(50,target)), fill="#d7464b", width=3)
                        if len(angles)>1:
                            d.line([xy(i,a) for i,a in enumerate(angles)], fill="#00796e", width=4)
                        d.text((660, 420), "빨강  목표각     초록  실제각", font=small, fill="#375666")
                        d.text((660, 464), f"{env.steps*0.02:.2f}초   {status}", font=text,
                               fill="#ac393e" if status=="넘어짐 종료" else "#134b58")
                        # 25fps에서 두 프레임 = 0.08초. 끝 자세는 판독을 위해 잠시 유지한다.
                        count = 16 if status.endswith("종료") else 2
                        proc.stdin.write(frame.tobytes()*count)
            finally:
                proc.stdin.close()
            if proc.wait() != 0:
                raise RuntimeError(f"영상 인코딩 실패: {path}")
    temp.replace(path)
    print(path, flush=True)


def replay(out, stages, smooth, seeds, record):
    with HeadTarget(smooth) as env:
        for stage in stages:
            agent = PPO.load(out / f"{stage}.zip", device="cpu")
            if record:
                record_stage(out, stage, env, agent, seeds)
            else:
                show(env, agent, seeds)
