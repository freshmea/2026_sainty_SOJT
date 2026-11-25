"""이전 한 관절 비교 실습(SB3). 전신 균형·머리 추종 수업은 train_balance.py를 실행한다."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback

from head_env import HeadTarget

STAGES = ("early", "middle", "late")
ROLLOUT = 128


class StageCheckpoints(BaseCallback):
    """한 번의 learn()에서 PPO 업데이트가 끝난 시점의 정책을 보관한다."""

    def __init__(self, out, steps):
        super().__init__()
        batches = math.ceil(steps / ROLLOUT)
        self.out = out
        self.cutoffs = dict(early=ROLLOUT, middle=math.ceil(batches / 2)*ROLLOUT,
                            late=batches*ROLLOUT)

    def _save_due(self):
        for stage, tick in self.cutoffs.items():
            if self.num_timesteps == tick:
                self.model.save(self.out / stage)
                print(f"{stage}: {tick}스텝, {self.model.device}", flush=True)

    def _on_rollout_start(self):
        # 다음 수집의 시작은 직전 데이터에 대한 optimizer 갱신까지 마친 시점이다.
        self._save_due()

    def _on_step(self):
        return True

    def _on_training_end(self):
        # 마지막 갱신 뒤에는 다음 rollout이 없어 별도로 저장한다.
        self._save_due()


def evaluate(env, agent):
    errors, returns, falls = [], [], 0
    for seed in range(100, 110):
        obs, _ = env.reset(seed=seed)
        total = 0.0
        while True:
            obs, reward, fallen, timeout, info = env.step(agent(obs))
            total += reward
            errors.append(info["error_rad"]**2)
            if fallen or timeout:
                falls += int(fallen)
                returns.append(total)
                break
    return dict(rmse_rad=float(np.sqrt(np.mean(errors))), falls=falls,
                mean_return=float(np.mean(returns)))


def train(out, steps, smooth, device):
    out.mkdir(parents=True, exist_ok=True)
    with HeadTarget(smooth) as env:
        # actor가 연속 행동을 선택하고 critic이 상태 가치를 추정한다.
        model = PPO("MlpPolicy", env, seed=7, device=device, n_steps=ROLLOUT,
                    batch_size=64, learning_rate=0.001, policy_kwargs={"net_arch": [32, 32]})
        stages = StageCheckpoints(out, steps)
        model.learn(total_timesteps=steps, callback=stages)
        model.save(out / "ppo")  # 기존 실습의 최종 정책 파일명도 유지한다.

        results = {"zero": evaluate(env, lambda obs: np.zeros(1)),
                   "target": evaluate(env, lambda obs: obs[:1]/0.35)}
        checkpoints = {}
        for stage in STAGES:
            restored = PPO.load(out / stage, device="cpu")
            score = evaluate(env, lambda obs: restored.predict(obs, deterministic=True)[0])
            checkpoints[stage] = dict(step=restored.num_timesteps, **score)
        results["ppo"] = {k: checkpoints["late"][k] for k in results["zero"]}
        report = dict(training_steps=model.num_timesteps, training_seed=7, device=str(model.device),
                      evaluation_seeds=[100, 109], smooth=smooth, results=results,
                      checkpoints=checkpoints)
    (out / "metrics.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=4096)
    parser.add_argument("--smooth", type=float, default=0.02)
    parser.add_argument("--out", type=Path, help="학습 결과를 저장하거나 재생할 폴더")
    parser.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--viewer", choices=STAGES, help="지정한 정책으로 WSLg 실시간 추론")
    mode.add_argument("--record", action="store_true", help="세 체크포인트를 MP4로 녹화")
    parser.add_argument("--replay-seed", type=int, action="append",
                        help="재생 초기조건. 반복 지정 가능하며 기본값은 100, 101")
    args = parser.parse_args()
    out = args.out or Path("results") / f"steps{args.steps}_smooth{args.smooth}"
    torch.set_num_threads(1)
    if args.viewer or args.record:
        from head_replay import replay
        wanted = (args.viewer,) if args.viewer else STAGES
        if not all((out / f"{stage}.zip").is_file() for stage in wanted) or not (out / "metrics.json").is_file():
            parser.error(f"{out}에서 단계별 ZIP과 metrics.json을 찾을 수 없습니다. 같은 --out으로 먼저 학습하세요.")
        saved_smooth = json.loads((out / "metrics.json").read_text())["smooth"]
        replay(out, wanted, saved_smooth, args.replay_seed or [100, 101], record=args.record)
    else:
        if args.steps <= 0:
            parser.error("--steps는 양수여야 합니다.")
        if args.device == "cuda" and not torch.cuda.is_available():
            parser.error("이 가상환경에서 CUDA를 사용할 수 없습니다. 교안의 GPU 확인 절차를 실행하세요.")
        train(out, args.steps, args.smooth, args.device)
