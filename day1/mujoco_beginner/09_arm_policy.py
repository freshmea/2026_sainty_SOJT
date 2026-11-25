"""Day 1 실습 09. SB3 PPO로 한 관절 팔을 학습하고 ONNX로 추론한다."""
import argparse
from importlib import import_module
from math import sqrt
from statistics import mean

import onnxruntime as ort
import torch
from stable_baselines3 import PPO
from torch import nn

from common import RESULTS, save_json

ArmEnv = import_module("07_gym_bridge").ArmEnv
CHECKPOINT = RESULTS / "09_arm_ppo.zip"
MODEL_FILE = RESULTS / "09_arm_policy.onnx"
EVAL_SEEDS = (11, 22, 33, 44, 55)


class OnnxActor(nn.Module):
    """학습된 actor의 결정적 행동만 내보낸다. 입력과 출력은 각각 [1, 3], [1, 1]."""

    def __init__(self, policy):
        super().__init__()
        # 같은 PyTorch 층을 가져오므로 재로딩한 SB3 가중치가 그대로 쓰인다.
        self.features = policy.features_extractor
        self.hidden = policy.mlp_extractor.policy_net
        self.output = policy.action_net

    def forward(self, observation):
        # 이 Box 정책에서 predict(deterministic=True)는 정규분포 평균을 행동 범위로 자른다.
        mean_action = self.output(self.hidden(self.features(observation)))
        return mean_action.clamp(-1.0, 1.0)


def evaluate(env, act):
    """모든 제어기를 같은 시작 각도에서 실행해 보상과 추종 오차를 비교한다."""
    returns, final_errors = [], []
    error_squared, steps, failures = 0.0, 0, 0
    for seed in EVAL_SEEDS:
        obs, _ = env.reset(seed=seed)
        total = 0.0
        while True:
            obs, reward, terminated, truncated, info = env.step(act(obs))
            total += reward
            error = info["error_rad"]
            error_squared += error ** 2
            steps += 1
            if terminated or truncated:
                returns.append(total)
                final_errors.append(abs(error))
                failures += int(terminated)
                break
    return dict(mean_return=mean(returns), rmse_rad=sqrt(error_squared / steps),
                final_abs_error_rad=mean(final_errors), failures=failures, steps=steps)


def train(env, steps):
    # microduck_lab과 같은 CPU PPO 구성. actor(pi)와 critic(vf)은 별도의 은닉층을 학습한다.
    model = PPO("MlpPolicy", env, seed=7, device="cpu", n_steps=128, batch_size=64,
                n_epochs=10, learning_rate=0.001, clip_range=0.2,
                policy_kwargs=dict(net_arch=dict(pi=[32, 32], vf=[32, 32]),
                                   activation_fn=nn.Tanh, optimizer_class=torch.optim.Adam))

    # 평가 환경을 분리해 기준선 측정이 학습 환경의 초기 상태나 난수 흐름에 관여하지 않게 한다.
    with ArmEnv() as test_env:
        zero = evaluate(test_env, lambda obs: [0.0])
        before = evaluate(test_env, lambda obs: model.predict(obs, deterministic=True)[0])

    # learn은 128스텝 수집 → advantage 추정 → PPO 손실 계산 → 역전파와 Adam 갱신을 반복한다.
    model.learn(total_timesteps=steps)
    model.save(CHECKPOINT)
    restored = PPO.load(CHECKPOINT, device="cpu")
    with ArmEnv() as test_env:
        after = evaluate(test_env, lambda obs: restored.predict(obs, deterministic=True)[0])

    # critic과 탐색용 표준편차는 학습에 쓰인다. 배포용 그래프에는 actor 경로만 담는다.
    actor = OnnxActor(restored.policy).eval()
    torch.onnx.export(actor, torch.zeros(1, 3), str(MODEL_FILE), opset_version=17,
                      input_names=["observation"], output_names=["action"])
    metrics = dict(training_seed=7, requested_steps=steps, training_steps=model.num_timesteps,
                   evaluation_seeds=EVAL_SEEDS, zero=zero, before=before, sb3=after)
    print(f"실제 학습 {model.num_timesteps}스텝 | SB3 평균 보상 {after['mean_return']:.6f}")
    print(CHECKPOINT, MODEL_FILE, save_json("09_arm_train.json", metrics), sep="\n")


def infer(env):
    # Day 4와 같은 방식으로 세션을 한 번 열고, 매 제어 주기마다 관측 한 건을 보낸다.
    session = ort.InferenceSession(str(MODEL_FILE), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name

    def act(obs):
        batch = session.run([output_name], {input_name: obs.reshape(1, 3)})[0]
        return batch[0]  # [1, 1] → [1]. 07번 env.step이 행동 × 2를 토크로 적용한다.

    result = evaluate(env, act)
    print(f"ONNX 평균 보상 {result['mean_return']:.6f} | "
          f"최종 평균 절대 오차 {result['final_abs_error_rad']:.5f} rad")
    print(save_json("09_arm_infer.json", dict(evaluation_seeds=EVAL_SEEDS, onnx=result)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="mode", required=True)
    train_args = modes.add_parser("train", help="PPO 학습, ZIP 저장·복원, ONNX 내보내기")
    train_args.add_argument("--steps", type=int, default=4096, help="학습 환경 스텝 수")
    modes.add_parser("infer", help="ONNX Runtime으로 팔 제어 및 평가")
    args = parser.parse_args()
    if args.mode == "train" and args.steps < 1:
        parser.error("--steps는 양의 정수여야 합니다.")
    if args.mode == "infer" and not MODEL_FILE.is_file():
        parser.error("ONNX 파일이 없습니다. 먼저 train을 실행하세요.")
    torch.set_num_threads(1)
    with ArmEnv() as env:
        if args.mode == "train":
            train(env, args.steps)
        else:
            infer(env)

# checkpoint 를 여러개 비교해서 best case 를 따로 저장 하는것!
# loss function 의 값을 학습 step 마다 저장해서 그래프로 저장
# learning rate 수정해보고, activation 함수 relu 로 수정(다른거... )