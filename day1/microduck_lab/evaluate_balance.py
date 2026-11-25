"""같은 제자리 머리 명령으로 공식 ONNX 및 학습 체크포인트의 기립·각도 오차를 측정한다."""
import argparse
from contextlib import ExitStack
import csv
import json
import math
from pathlib import Path
import random
import sys
import time

import mujoco
import numpy as np

# 50 Hz에서 2초씩 다섯 구간. 홀수 시드는 좌우 반전하여 방향 편향을 드러낸다.
TARGETS = (0, 20, -25, 30, -15)
HZ = 50
HOLD = 100
HORIZON = HOLD * len(TARGETS)
FALL_DEG = 60
# 수업 판정 기준. 전체 구간(목표가 바뀐 직후 포함)의 관절각 RMSE.
PASS_RMSE_DEG = 10
ROOT = Path(__file__).resolve().parent
DEFAULT_REPO = ROOT.parents[1] / "day4/microduck_rl_training"


def record_frame(renderer, camera, data, goal, actual, tilt):
    from PIL import Image, ImageDraw, ImageFont
    renderer.update_scene(data, camera=camera)
    image = Image.fromarray(renderer.render())
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype("DejaVuSans.ttf", 16)
    draw.rectangle((0, 0, image.width, 63), fill="#14242c")
    draw.text((10, 4), f"{data.time:4.2f} s    target {goal:+.0f} deg    yaw {actual:+.1f} deg    tilt {tilt:.1f} deg",
              fill="white", font=font)
    # 공통 각도 눈금(±40도) 위에 목표는 빨간 선, 측정은 흰 원으로 표시한다.
    center, scale = image.width // 2, image.width / 100
    draw.line((center-40*scale, 44, center+40*scale, 44), fill="#87a7b2", width=2)
    x = center + scale * goal
    draw.line((x, 32, x, 56), fill="#ff605d", width=4)
    x = center + scale * np.clip(actual, -40, 40)
    draw.ellipse((x-4, 40, x+4, 48), fill="white")
    return np.asarray(image)


def episode(official, repo, onnx, seed, rows, movie=None, viewer=False):
    # 공식 CPU 재생 경로의 전압·BAM·4개 물리 서브스텝을 그대로 쓴다.
    scene = repo / "src/mjlab_microduck/robot/microduck/scene_walk.xml"
    motor = official.load_bam_model(official.BAM_KP_FW, 7.4, None)
    model, data, bam, _ = official.load_mujoco_with_bam(str(scene), motor, .005, .1, official.BAM_VIN_MIN)
    policy = official.PolicyInference(model, data, walking_onnx_path=str(onnx), bam_ctrl=bam,
                                      use_projected_gravity=True, new_cmd_obs=True)
    if policy.ort_session.get_inputs()[0].shape[-1] != 61 or policy.n_joints != 14:
        raise ValueError("이 평가는 61차원 명령 관측과 14개 관절 모델을 사용합니다.")
    pitch = random.Random(seed).uniform(-3, 3)
    half = math.radians(pitch) / 2
    data.qpos[:7] = (0, 0, .125, math.cos(half), 0, math.sin(half), 0)
    data.qpos[policy.joint_qpos_indices] = policy.default_pose
    policy.set_position_targets(policy.default_pose)
    mujoco.mj_forward(model, data)
    errors = []
    peak_drift, peak_tilt = 0., 0.
    fallen = False
    with ExitStack() as resources:
        if movie:
            import imageio_ffmpeg
            renderer = resources.enter_context(mujoco.Renderer(model, height=384, width=576))
            encoder = imageio_ffmpeg.write_frames(str(movie), (576, 384), fps=HZ, codec="libx264")
            encoder.send(None)
            resources.callback(encoder.close)
        if viewer:
            from mujoco.viewer import launch_passive
            window = resources.enter_context(launch_passive(model, data))
        camera = mujoco.MjvCamera()
        camera.distance, camera.azimuth, camera.elevation = .58, 125, -19
        camera.lookat[:] = (0, 0, .13)
        if viewer:
            window.cam.distance, window.cam.azimuth, window.cam.elevation = camera.distance, camera.azimuth, camera.elevation
            window.cam.lookat[:] = camera.lookat
        for tick in range(HORIZON):
            start = time.monotonic()
            goal = TARGETS[tick // HOLD] * (-1 if seed % 2 else 1)
            # 13개 명령 슬롯 중 head_yaw만 변경한다. 모터 목표는 오직 actor가 결정한다.
            if tick % HOLD == 0:
                policy.head_offset[2] = math.radians(goal)
                policy.set_vel_cmd(0, 0, 0)
            policy.apply_action(policy.infer())
            for _ in range(4):
                bam.update()
                mujoco.mj_step(model, data)
                mujoco.mj_forward(model, data)
                upright = data.body("trunk_base").xmat[8]
                tilt = math.degrees(math.acos(np.clip(upright, -1, 1)))
                fallen = not np.isfinite(data.qpos).all() or not np.isfinite(tilt) or tilt >= FALL_DEG
                if fallen:
                    break
            actual = math.degrees(data.joint("head_yaw").qpos[0])
            drift = float(np.linalg.norm(data.qpos[:2]))
            peak_drift, peak_tilt = max(peak_drift, drift), max(peak_tilt, tilt)
            rows.writerow((seed, round(data.time, 4), goal, actual, tilt, drift, int(fallen)))
            # 넘어진 이후나 리셋 상태의 각도를 더해 오차가 작아지는 일을 피한다.
            if not fallen:
                errors.append((actual - goal) ** 2)
            if movie:
                encoder.send(record_frame(renderer, camera, data, goal, actual, tilt))
            if viewer:
                if not window.is_running():
                    raise KeyboardInterrupt("뷰어를 닫아 평가를 중단했습니다.")
                window.set_texts((None, None, "Target / yaw / tilt (deg)",
                                  f"{goal:+.0f} / {actual:+.1f} / {tilt:.1f}"))
                window.sync()
                time.sleep(max(0, 1/HZ - (time.monotonic()-start)))
            if fallen:
                break
    rmse = math.sqrt(sum(errors) / len(errors)) if errors else None
    return dict(seed=seed, initial_pitch_deg=pitch, upright=not fallen, seconds=round(data.time, 4),
                rmse_deg=rmse, samples=len(errors), max_tilt_deg=peak_tilt, max_drift_m=peak_drift,
                passed=not fallen and rmse is not None and rmse <= PASS_RMSE_DEG)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=DEFAULT_REPO)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--seed", type=int, default=100)
    parser.add_argument("--record", action="store_true", help="첫 평가 시드를 MP4로 저장")
    parser.add_argument("--viewer", action="store_true", help="첫 평가 시드를 실시간 재생")
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error("episodes >= 1")
    sys.path.insert(0, str(args.repo / "scripts"))
    import infer_policy as official
    args.out.mkdir(parents=True, exist_ok=True)
    trials = []
    with (args.out / "trace.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("seed","time_s","target_deg","head_yaw_deg","tilt_deg","drift_m","fall"))
        for i in range(args.episodes):
            movie = args.out / "replay.mp4" if args.record and i == 0 else None
            trials.append(episode(official, args.repo, args.policy, args.seed+i, writer, movie, args.viewer and i==0))
    n = sum(t["samples"] for t in trials)
    sse = sum(t["samples"] * t["rmse_deg"]**2 for t in trials if t["samples"])
    import hashlib
    summary = dict(policy=args.policy.name, sha256=hashlib.sha256(args.policy.read_bytes()).hexdigest(),
                   horizon_s=HORIZON/HZ, targets_deg=TARGETS, fall_deg=FALL_DEG, pass_rmse_deg=PASS_RMSE_DEG,
                   upright_rate=sum(t["upright"] for t in trials)/len(trials), rmse_deg=math.sqrt(sse/n) if n else None,
                   joint_pass_rate=sum(t["passed"] for t in trials)/len(trials), trials=trials)
    (args.out / "metrics.json").write_text(json.dumps(summary, indent=2, allow_nan=False))
    print({k:v for k,v in summary.items() if k != "trials"})


if __name__ == "__main__":
    main()
