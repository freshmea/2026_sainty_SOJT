"""실습 11: 빠른 계단 목표에서 PPO와 다시 조정한 PD를 같은 조건으로 비교한다."""
import argparse
import csv
import json
import math
from importlib import import_module
from itertools import product
from statistics import mean
from shutil import copyfile

import mujoco
import torch
from stable_baselines3 import PPO

from stable_baselines3.common.env_util import make_vec_env

from common import RESULTS

ArmEnv = import_module("07_gym_bridge").ArmEnv
EVAL_SEEDS = (11, 22, 33, 44, 55)
TARGETS = (45, 15, 60, -15)
HOLD_STEPS = 50            # 50 Hz × 1.00초 = 목표당 행동 50번
TORQUE_LIMIT = 8.0         # N·m. 07번의 4배. 각속도 자체를 강제로 설정하지 않는다.
OUT = RESULTS / f"11_cuda_hold{HOLD_STEPS}_torque{TORQUE_LIMIT:g}"
TUNE_SEEDS = (401, 402, 403)
TRAIN_SEEDS = (7, 17, 27)
STEPS = 65536
CHECK_EVERY = 8192


class FastTargets(ArmEnv):
    def __init__(self):
        super().__init__()
        self.model.actuator("motor").ctrlrange[:] = [-TORQUE_LIMIT, TORQUE_LIMIT]

    def reset(self, **kwargs):
        self.target = math.radians(TARGETS[0])
        return super().reset(**kwargs)

    def step(self, action):
        torque = TORQUE_LIMIT * max(-1., min(1., float(action[0])))
        self.data.actuator("motor").ctrl[0] = torque
        mujoco.mj_step(self.model, self.data, nstep=10)
        self.steps += 1
        # 이번 행동과 보상은 같은 목표를 기준으로 한다.
        obs = self._obs()
        error = self.target - float(obs[0])
        velocity = float(obs[1])
        reward = -(error**2 + 0.01*velocity**2 + 0.001*torque**2)
        failed = abs(float(obs[0])) > math.pi
        timeout = self.steps >= HOLD_STEPS * len(TARGETS)
        info = dict(error=error, torque=torque)
        if self.steps % HOLD_STEPS == 0 and not timeout:
            index = (self.steps // HOLD_STEPS) % len(TARGETS)
            self.target = math.radians(TARGETS[index])
            obs = self._obs()
        return obs, reward, failed, timeout, info


def pd_policy(kp, kd):
    def act(obs):
        angle, velocity, target = map(float, obs)
        return [(kp*(target-angle) - kd*velocity) / TORQUE_LIMIT]
    return act


def evaluate(env, act, seeds, trace=None):
    returns, errors, speeds, torques, end_hits = [], [], [], [], []
    failures = 0
    reach_pct = []
    for seed in seeds:
        obs, _ = env.reset(seed=seed)
        total = 0.0
        while True:
            goal = math.degrees(env.target)
            if env.steps % HOLD_STEPS == 0:
                start_distance = abs(goal - math.degrees(float(obs[0])))
            obs, reward, failed, timeout, info = env.step(act(obs))
            error_deg = math.degrees(info["error"])
            total += reward
            errors.append(error_deg)
            speeds.append(float(obs[1]))
            torques.append(info["torque"])
            if env.steps % HOLD_STEPS == 0:
                end_hits.append(abs(error_deg) <= 2)
                # 끝에 남은 거리를 시작 거리로 나눈다. 지나친 뒤 멀어져도 감점된다.
                reach_pct.append(100 * (1 - abs(error_deg) / max(start_distance, 1e-9)))
            if trace is not None:
                trace.append((seed, env.steps / 50, goal, math.degrees(float(obs[0])),
                              error_deg, math.degrees(float(obs[1])), info["torque"], reward))
            if failed or timeout:
                returns.append(total)
                failures += int(failed)
                break
    def rms(values):
        return math.sqrt(mean(x*x for x in values))
    return dict(mean_return=mean(returns), rmse_deg=rms(errors),
                rms_speed_deg_s=math.degrees(rms(speeds)), rms_torque_nm=rms(torques),
                end_hits=sum(end_hits), segments=len(end_hits), failures=failures,
                expected_segments=len(seeds)*len(TARGETS),
                reached_90=sum(x >= 90 for x in reach_pct),
                min_reach_pct=min(reach_pct, default=0.0), segment_reach_pct=reach_pct)


def tune_pd(env):
    # 튜닝용 시작 각도에서 평균 보상으로 선택한다. 평가용 시작 각도는 따로 둔다.
    kps = (0.25, 0.5, 1, 2, 4, 8, 14, 24, 40, 64)
    kds = (0.1, 0.2, 0.4, 0.6, 0.9, 1.2, 1.6, 2.5, 4, 6)
    scores = {}
    def scan(pairs):
        for kp, kd in pairs:
            if (kp, kd) not in scores:
                result = evaluate(env, pd_policy(kp, kd), TUNE_SEEDS)
                scores[kp, kd] = result["mean_return"]
    scan(product(kps, kds))
    kp, kd = max(scores, key=scores.get)
    # 넓게 찾은 후보 주변을 한 번 더 탐색해 성긴 간격의 영향을 줄인다.
    factors = (0.6, 0.8, 0.9, 1.0, 1.1, 1.25, 1.5)
    scan(product([kp*x for x in factors], [kd*x for x in factors]))
    kp, kd = max(scores, key=scores.get)
    return dict(kp=kp, kd=kd, candidates=len(scores), tuning_return=scores[kp,kd])


def train(seed):
    # CPU형 패키지 설치 또는 WSL GPU 연결 문제를 조용히 넘기지 않는다.
    if not torch.cuda.is_available():
        raise RuntimeError(f"CUDA를 사용할 수 없습니다. torch={torch.__version__}, "
                           f"CUDA runtime={torch.version.cuda}. uv sync --locked 후 확인하세요.")
    device = "cuda:0"
    print(f"seed={seed} | {torch.cuda.get_device_name(0)} | torch={torch.__version__} | "
          f"CUDA={torch.version.cuda}", flush=True)
    # MuJoCo 물리는 CPU에서 계산한다. 8개 환경의 관측을 묶어 GPU 정책으로 전달한다.
    envs = make_vec_env(FastTargets, n_envs=8, seed=seed)
    history = []
    best_key = None
    try:
        model = PPO("MlpPolicy", envs, seed=seed, device=device, n_steps=128,
                    batch_size=128, n_epochs=10, learning_rate=0.001, clip_range=0.2,
                    policy_kwargs=dict(net_arch=dict(pi=[32,32], vf=[32,32])))
        with FastTargets() as validation:
            while model.num_timesteps < STEPS:
                model.learn(total_timesteps=CHECK_EVERY, reset_num_timesteps=False)
                # 가중치와 Adam의 실제 모멘트 텐서가 모두 GPU에 있는지 업데이트 후 확인한다.
                param_device = str(next(model.policy.parameters()).device)
                adam_device = str(next(iter(model.policy.optimizer.state.values()))["exp_avg"].device)
                if param_device != device or adam_device != device:
                    raise RuntimeError(f"GPU 학습 장치 불일치: {param_device}, {adam_device}")
                result = evaluate(validation, lambda obs: model.predict(obs, deterministic=True)[0],
                                  TUNE_SEEDS)
                key = (result["reached_90"], result["min_reach_pct"], result["mean_return"])
                if best_key is None or key > best_key:
                    best_key, best_step = key, model.num_timesteps
                    model.save(OUT / f"ppo_seed{seed}.zip")
                history.append(dict(steps=model.num_timesteps, reached_90=result["reached_90"],
                                    min_reach_pct=result["min_reach_pct"]))
                print(f"seed={seed} | {model.num_timesteps}/{STEPS} | 검증 90% 도달 "
                      f"{result['reached_90']}/{result['expected_segments']} | "
                      f"최소 {result['min_reach_pct']:.1f}% | Adam {adam_device}", flush=True)
        record = dict(seed=seed, torch=torch.__version__, cuda_runtime=torch.version.cuda,
                      gpu=torch.cuda.get_device_name(0), policy_device=param_device,
                      adam_device=adam_device, selected_step=best_step, history=history)
        (OUT / f"training_seed{seed}.json").write_text(json.dumps(record, indent=2))
    finally:
        envs.close()



def refine_if_needed(seed):
    """검증 구간이 90% 경계에 걸리면 선택된 모델에서 낮은 학습률로 추가 학습한다."""
    path = OUT / f"training_seed{seed}.json"
    record = json.loads(path.read_text())
    envs = make_vec_env(FastTargets, n_envs=8, seed=seed)
    try:
        if not torch.cuda.is_available():
            raise RuntimeError("추가 학습에는 CUDA가 필요합니다.")
        model = PPO.load(OUT / f"ppo_seed{seed}.zip", env=envs, device="cuda:0",
                         learning_rate=0.0003)
        with FastTargets() as env:
            def check():
                score = evaluate(env, lambda obs: model.predict(obs, deterministic=True)[0], TUNE_SEEDS)
                return score["reached_90"], score["min_reach_pct"], score["mean_return"]
            best = check()
            extra = 0
            while (best[0] < len(TUNE_SEEDS)*len(TARGETS) or best[1] < 92) and extra < STEPS:
                model.learn(total_timesteps=CHECK_EVERY, reset_num_timesteps=False)
                extra += CHECK_EVERY
                key = check()
                if key > best:
                    best = key
                    record["selected_step"] = model.num_timesteps
                    model.save(OUT / f"ppo_seed{seed}.zip")
                record["history"].append(dict(stage="refine", steps=model.num_timesteps,
                                               reached_90=key[0], min_reach_pct=key[1]))
                print(f"seed={seed} | 추가 {extra}스텝 | 최고 검증 최소 도달률 {best[1]:.2f}%",
                      flush=True)
            record["extra_training_steps"] = extra
            record["total_training_steps"] = STEPS + extra
            path.write_text(json.dumps(record, indent=2))
    finally:
        envs.close()

def plot_trace(selected_seed):
    """첫 평가 시드의 전체 경로에서 모든 목표 전환과 응답을 읽는다."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with (OUT / "trajectories.csv").open() as f:
        rows = [row for row in csv.DictReader(f)
                if int(row["seed"]) == EVAL_SEEDS[0]]
    with FastTargets() as env:
        obs, _ = env.reset(seed=EVAL_SEEDS[0])
        initial_deg = math.degrees(float(obs[0]))
    fig, ax = plt.subplots(figsize=(10, 3.8), layout="constrained")
    # 계단 경계 시각의 목표는 새로 주는 명령이고, CSV는 직전 행동에 사용한 목표를 기록한다.
    hold_s = HOLD_STEPS / 50
    ax.stairs(TARGETS, [i * hold_s for i in range(len(TARGETS)+1)],
              baseline=None, color="#c6373d", linewidth=2, label="Target")
    for name, label, color in (("pd_retuned", "Retuned PD", "#155a85"),
                               (f"ppo_seed{selected_seed}", f"PPO (training seed {selected_seed})", "#16845e")):
        selected = [r for r in rows if r["policy"] == name]
        ax.plot([0]+[float(r["time_s"]) for r in selected],
                [initial_deg]+[float(r["angle_deg"]) for r in selected],
                linewidth=2, color=color, label=label)
    ax.set(xlim=(0, len(TARGETS)*hold_s), xlabel="Simulation time (s)", ylabel="Joint angle (degrees)",
           title=f"Target switches every {hold_s:.2f} s | evaluation seed 11")
    ax.grid(axis="y", alpha=0.2)
    ax.legend(loc="upper right", ncol=3, bbox_to_anchor=(1, 1.23), frameon=False)
    fig.savefig(OUT / "tracking.png", dpi=160)
    plt.close(fig)


def compare():
    with FastTargets() as env:
        gains = tune_pd(env)
        controllers = {"pd_previous": pd_policy(14, 1.6),
                       "pd_retuned": pd_policy(gains["kp"], gains["kd"])}
        for seed in TRAIN_SEEDS:
            agent = PPO.load(OUT / f"ppo_seed{seed}.zip", device="cuda:0")
            controllers[f"ppo_seed{seed}"] = lambda obs, m=agent: m.predict(obs, deterministic=True)[0]
        # 기본 배포 모델은 최종 평가 시드에 접근하기 전에 검증 결과만으로 고른다.
        validation = {s: evaluate(env, controllers[f"ppo_seed{s}"], TUNE_SEEDS) for s in TRAIN_SEEDS}
        def selection_key(seed):
            metrics = validation[seed]
            return metrics["reached_90"], metrics["min_reach_pct"], metrics["mean_return"]
        selected_seed = max(TRAIN_SEEDS, key=selection_key)
        copyfile(OUT / f"ppo_seed{selected_seed}.zip", OUT / "policy.zip")
        results = {}
        with (OUT / "trajectories.csv").open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["policy","seed","time_s","target_deg","angle_deg","error_deg",
                             "velocity_deg_s","torque_nm","reward"])
            for name, act in controllers.items():
                trace = []
                results[name] = evaluate(env, act, EVAL_SEEDS, trace)
                writer.writerows((name,*row) for row in trace)
    report = dict(targets_deg=TARGETS, hold_seconds=HOLD_STEPS/50,
                  torque_limit_nm=TORQUE_LIMIT, initial_training_steps=STEPS,
                  training_steps_per_seed={s: json.loads((OUT / f"training_seed{s}.json").read_text()).get("total_training_steps", STEPS) for s in TRAIN_SEEDS},
                  train_seeds=TRAIN_SEEDS, tune_seeds=TUNE_SEEDS, eval_seeds=EVAL_SEEDS,
                  tuned_pd=gains, selected_seed=selected_seed, selection_seeds=TUNE_SEEDS, results=results)
    (OUT / "comparison.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    plot_trace(selected_seed)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    training = sub.add_parser("train", help="빠른 경로에서 PPO를 처음부터 학습")
    training.add_argument("--seed", type=int, choices=TRAIN_SEEDS, help="생략하면 세 시드 모두 학습")
    sub.add_parser("compare", help="PD 게인 탐색 후 저장된 세 모델과 비교")
    args = parser.parse_args()
    torch.set_num_threads(1)
    OUT.mkdir(exist_ok=True)
    if args.mode == "train":
        for seed in TRAIN_SEEDS if args.seed is None else (args.seed,):
            train(seed)
            refine_if_needed(seed)
    else:
        compare()
