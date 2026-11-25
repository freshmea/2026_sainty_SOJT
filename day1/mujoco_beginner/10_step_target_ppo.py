"""실습 10: 4개의 목표각을 매 학습 에피소드에서 차례로 추종한다."""
import argparse
import csv
import json
import math
from importlib import import_module

import torch
from stable_baselines3 import PPO

lab = import_module("09_arm_policy")
OUT = lab.RESULTS / "10_step_targets"
TARGETS = (45, 15, 60, -15)
LABELS = ("기본 PD", "08번 PD", "고정 목표 PPO", "가변 목표 PPO")
NAMES = ("pd", "pd_grid", "ppo_fixed", "ppo_step")


class StepTargets(lab.ArmEnv):
    def reset(self, **kwargs):
        self.target = math.radians(TARGETS[0])
        return super().reset(**kwargs)

    def step(self, action):
        # 이 행동의 보상은 행동을 선택할 때 알려 준 목표에 대해 계산한다.
        obs, reward, failed, timeout, info = super().step(action)
        if self.steps % 50 == 0 and self.steps < 200:
            # 다음 구간의 첫 행동을 계산하기 전에 새 목표를 관측에 담는다.
            self.target = math.radians(TARGETS[self.steps // 50])
            obs = self._obs()
        return obs, reward, failed, timeout, info


def train():
    # 09번과 같은 망·보상·학습 시드. 4,096스텝에서도 저장해 동일 예산 비교를 남긴다.
    with StepTargets() as env:
        model = PPO("MlpPolicy", env, seed=7, device="cpu", n_steps=128, batch_size=64,
                    n_epochs=10, learning_rate=0.001, clip_range=0.2,
                    policy_kwargs=dict(net_arch=dict(pi=[32, 32], vf=[32, 32])))
        for budget in (4096, 32768):
            model.learn(total_timesteps=budget-model.num_timesteps,
                        reset_num_timesteps=(budget == 4096))
            model.save(OUT / f"ppo_{budget}.zip")
            print(f"저장: {model.num_timesteps}스텝", flush=True)
        actor = lab.OnnxActor(model.policy).eval()
        torch.onnx.export(actor, torch.zeros(1, 3), str(OUT / "ppo_step.onnx"),
                          input_names=["observation"], output_names=["action"], opset_version=17)


def controllers():
    def pd(kp, kd):
        def command(obs):
            q, v, target = map(float, obs)
            return [max(-1., min(1., (kp * (target-q) - kd*v) / 2))]
        return command

    def agent(path):
        model = PPO.load(path, device="cpu")
        return lambda obs: model.predict(obs, deterministic=True)[0]

    return dict(pd=pd(4, 0.6), pd_grid=pd(14, 1.6), ppo_fixed=agent(lab.CHECKPOINT),
                ppo_step=agent(OUT / "ppo_32768.zip"), ppo_step_4096=agent(OUT / "ppo_4096.zip"))


def evaluate():
    rows, summaries = [], {}
    with StepTargets() as env:
        for name, act in controllers().items():
            episodes = []
            for seed in lab.EVAL_SEEDS:
                obs, _ = env.reset(seed=seed)
                episode = []
                for step in range(200):
                    target = TARGETS[step // 50]
                    obs, reward, failed, timeout, info = env.step(act(obs))
                    row = dict(controller=name, seed=seed, segment=step // 50 + 1,
                               time_s=(step+1)/50, target_deg=target,
                               angle_deg=math.degrees(float(obs[0])),
                               error_deg=math.degrees(info["error_rad"]), reward=reward)
                    episode.append(row)
                    if failed or timeout:
                        break
                episodes.append(episode)
                rows.extend(episode)
            # 정착: 구간 끝까지 ±2° 안에 머물고, 그 마지막 연속 체류가 0.20초 이상인 경우.
            segments = []
            for i, target in enumerate(TARGETS):
                settled_times, ends = [], []
                for ep in episodes:
                    part = ep[i*50:(i+1)*50]
                    if len(part) != 50:
                        continue
                    ends.append(part[-1]["angle_deg"])
                    outside = [j for j,x in enumerate(part) if abs(x["error_deg"]) > 2]
                    first = outside[-1] + 1 if outside else 0
                    if first < 50 and (49-first)/50 >= 0.2:
                        settled_times.append((first+1)/50)
                segments.append(dict(target_deg=target, settled=len(settled_times),
                                     settle_times_s=settled_times, end_angles_deg=ends))
            samples = [x for ep in episodes for x in ep]
            summaries[name] = dict(mean_return=sum(x["reward"] for x in samples)/len(episodes),
                rmse_deg=math.sqrt(sum(x["error_deg"]**2 for x in samples)/len(samples)),
                lengths=[len(ep) for ep in episodes], segments=segments)
    with (OUT / "trajectories.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    (OUT / "scores.json").write_text(json.dumps(summaries, indent=2), encoding="utf-8")
    for name,s in summaries.items():
        print(name, round(s["mean_return"],4), round(s["rmse_deg"],3),
              [x["settled"] for x in s["segments"]])



def target_line(scene, target_deg):
    """MuJoCo 장면에 관절 축에서 목표각 방향으로 뻗는 빨간 선을 추가한다."""
    import mujoco
    import numpy as np

    angle = math.radians(target_deg)
    geom = scene.geoms[scene.ngeom]
    scene.ngeom += 1
    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3),
                        np.zeros(3), np.eye(3).ravel(), np.array([1., 0.025, 0.04, 1.]))
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.004,
                         [0., 0., 0.538], [0.49*math.cos(angle), 0.49*math.sin(angle), 0.538])


def top_view(camera):
    camera.lookat[:] = [0.16, 0.13, 0.5]
    camera.azimuth, camera.elevation, camera.distance = 90, -89.9, 0.86


def viewer():
    """WSLg의 대화형 MuJoCo 뷰어에서 재학습 PPO의 한 에피소드를 느리게 재생한다."""
    import time
    import mujoco
    import mujoco.viewer

    agent = PPO.load(OUT / "ppo_32768.zip", device="cpu")
    with StepTargets() as env, mujoco.viewer.launch_passive(env.model, env.data) as window:
        top_view(window.cam)
        obs, _ = env.reset(seed=lab.EVAL_SEEDS[0])
        while window.is_running():
            with window.lock():
                window.user_scn.ngeom = 0
                target_line(window.user_scn, math.degrees(env.target))
            window.sync()
            time.sleep(0.08)
            action = agent.predict(obs, deterministic=True)[0]
            obs, _, done, timeout, _ = env.step(action)
            if done or timeout:
                break


def record():
    """평가 CSV의 관절 상태를 MuJoCo 장면으로 재생해 4분할 영상과 개별 영상을 만든다."""
    import subprocess
    from contextlib import ExitStack
    from pathlib import Path
    import mujoco
    from PIL import Image, ImageDraw, ImageFont

    seed = lab.EVAL_SEEDS[0]
    with (OUT / "trajectories.csv").open() as f:
        rows = list(csv.DictReader(f))
    traces = {}
    with StepTargets() as env:
        initial_obs, _ = env.reset(seed=seed)
        for name in NAMES:
            angles = [float(row["angle_deg"]) for row in rows
                      if row["controller"] == name and int(row["seed"]) == seed]
            if len(angles) != 200:
                raise ValueError(f"{name}: 전체 4초 기록이 필요합니다.")
            traces[name] = [math.degrees(float(initial_obs[0]))] + angles

        camera = mujoco.MjvCamera()
        top_view(camera)
        font_dir = Path("/usr/share/fonts/truetype/nanum")
        def font(size, bold=False):
            face = "NanumGothicBold.ttf" if bold else "NanumGothic.ttf"
            return ImageFont.truetype(str(font_dir / face), size)
        title_font, body_font, small = font(26, True), font(19, True), font(16)
        bg, ink, teal, red = "#f6f7f6", "#173747", "#087d7c", "#df2539"
        width, height, header = 640, 412, 84

        def encode(stack, name, size):
            proc = stack.enter_context(subprocess.Popen([
                "ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                "-s", size, "-r", "25", "-i", "-", "-an", "-c:v", "libx264",
                "-threads", "2", "-preset", "veryfast", "-crf", "19", "-pix_fmt", "yuv420p",
                "-movflags", "+faststart", str(OUT / f"{name}.mp4")], stdin=subprocess.PIPE))
            return proc

        def curve_xy(index, degrees):
            return (48 + index*568/200, 359 - degrees*0.67)

        def panel(renderer, name, label, k, target):
            current = traces[name][k]
            env.data.joint("hinge").qpos[0] = math.radians(current)
            mujoco.mj_forward(env.model, env.data)
            renderer.update_scene(env.data, camera=camera)
            target_line(renderer.scene, target)
            pane = Image.new("RGB", (width, height), bg)
            pane.paste(Image.fromarray(renderer.render()), (0, 52))
            d = ImageDraw.Draw(pane)
            d.text((14, 12), label, font=title_font, fill=ink)
            d.text((280, 17), f"목표 {target:+d}°   실제 {current:+.1f}°   오차 {target-current:+.1f}°",
                   font=body_font, fill=ink)
            # 같은 축에 계단 목표(빨강)와 시각 k까지의 실제 각도(청록)를 겹쳐 그린다.
            for deg in (-15, 15, 45, 60):
                y = curve_xy(0, deg)[1]
                d.line((48, y, 616, y), fill="#d6dfe2")
            d.text((5, 312), "60°", font=small, fill=ink)
            d.text((4, 359), "-15°", font=small, fill=ink)
            for i, deg in enumerate(TARGETS):
                x0,y = curve_xy(i*50,deg)
                x1,_ = curve_xy((i+1)*50,deg)
                d.line((x0,y,x1,y), fill=red, width=3)
                if i:
                    d.line((x0,curve_xy(0,TARGETS[i-1])[1],x0,y), fill=red, width=2)
            pts = [curve_xy(i,a) for i,a in enumerate(traces[name][:k+1])]
            if len(pts)>1:
                d.line(pts, fill=teal, width=3)
            x,y = pts[-1]
            d.ellipse((x-4,y-4,x+4,y+4), fill=teal)
            d.text((14,383), f"시뮬레이션 {k/50:.2f}초  /  4.00초", font=small, fill=ink)
            state = "±2° 이내" if abs(target-current) <= 2 else "±2° 밖"
            d.text((494,383), state, font=small, fill=teal if abs(target-current)<=2 else ink)
            return pane

        with ExitStack() as stack:
            renderer = stack.enter_context(mujoco.Renderer(env.model, height=250, width=width))
            videos = {n: encode(stack, n, f"{width}x{height}") for n in NAMES}
            comp = encode(stack, "comparison", f"{width*2}x{height*2+header}")
            for k in range(201):
                target = TARGETS[min(k//50, 3)]
                image = Image.new("RGB", (width*2,height*2+header), "#142f3d")
                d = ImageDraw.Draw(image)
                d.text((20,8), "목표각이 바뀔 때 로봇 팔은 얼마나 빨리 따라갈까?", font=title_font, fill="white")
                d.text((20,48), "빨간 선 = 목표각    청록색 팔·곡선 = 실제 각도     |     같은 초기 상태 · 0.25배속", font=body_font, fill="#cce0e7")
                for i,(n,label) in enumerate(zip(NAMES,LABELS)):
                    pane = panel(renderer, n, label, k, target)
                    image.paste(pane, (i%2*width,header+i//2*height))
                    videos[n].stdin.write(pane.tobytes()*2)
                comp.stdin.write(image.tobytes()*2)
            for proc in [*videos.values(), comp]:
                proc.stdin.close()
                if proc.wait():
                    raise RuntimeError("ffmpeg 인코딩 실패")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["train", "evaluate", "record", "viewer"])
    args = parser.parse_args()
    OUT.mkdir(exist_ok=True)
    torch.set_num_threads(1)
    if args.mode == "train":
        train()
        evaluate()
    elif args.mode == "evaluate":
        evaluate()
    elif args.mode == "record":
        record()
    else:
        viewer()
