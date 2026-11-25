"""공식 평지 속도 PPO를 제자리 머리 추종으로 미세조정한다 (WSL/CUDA)."""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path

import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
from mjlab.utils.os import dump_yaml
from mjlab.utils.torch import configure_torch_backends
from mjlab_microduck.tasks import MicroduckOnPolicyRunner

TASK = "Mjlab-Velocity-Flat-MicroDuck"
PUBLIC_SHA = "702c01bd02aeda1d7869390342e00298193e7071d475f3776e0a6eedfc5c8209"


def classroom_cfg(num_envs, seed):
    cfg = load_env_cfg(TASK)
    cfg.scene.num_envs = num_envs
    cfg.seed = seed
    cfg.episode_length_s = 10.0
    cfg.terminations["fell_over"].params["limit_angle"] = math.radians(60)
    # 이동·회전 목표와 제자리 회전 샘플러를 모두 끈다. 다리 행동은 계속 활성이다.
    twist = cfg.commands["twist"]
    twist.rel_standing_envs = 1.0
    twist.rel_forward_envs = 0.0
    twist.rel_heading_envs = twist.rel_turn_in_place_envs = 0.0
    twist.ranges.lin_vel_x = twist.ranges.lin_vel_y = twist.ranges.ang_vel_z = (0., 0.)
    # 4개 머리 명령 중 세 번째가 head_yaw이다. 각도는 HOME 기준 라디안.
    zero = (0., 0.)
    yaw = math.radians(30)
    cfg.commands["head_pose"].ranges = (zero, zero, (-yaw, yaw), zero)
    cfg.commands["head_pose"].resampling_time_range = (1.5, 2.5)
    cfg.commands["body_pose"].ranges = (zero,) * 6
    # 원본 커리큘럼이 위의 수업용 명령 분포를 다시 쓰지 않게 한다.
    for name in ("standing_envs", "head_pose_range", "body_pose_range"):
        del cfg.curriculum[name]
    return cfg


class ClassroomRunner(MicroduckOnPolicyRunner):
    @staticmethod
    def _get_export_paths(checkpoint_path):
        # 원본 runner의 ONNX 하나를 덮어쓰지 않고 각 .pt와 이름을 맞춘다.
        path = Path(checkpoint_path).with_suffix(".onnx")
        return path.parent, path.name, path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--warmstart", type=Path, help="공식 velocity2 3750 체크포인트")
    parser.add_argument("--envs", type=int, default=32)
    parser.add_argument("--iterations", type=int, default=25)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    if args.iterations < 3 or args.envs < 4:
        parser.error("iterations >= 3, envs >= 4")
    if not torch.cuda.is_available():
        parser.error("CUDA PyTorch 및 WSL NVIDIA 드라이버를 확인하세요.")
    if args.warmstart and hashlib.sha256(args.warmstart.read_bytes()).hexdigest() != PUBLIC_SHA:
        parser.error("공개 체크포인트 해시 불일치")
    if args.out.exists() and any(args.out.iterdir()):
        parser.error("기존 체크포인트 보호를 위해 비어 있는 출력 폴더를 지정하세요.")
    args.out.mkdir(parents=True, exist_ok=True)
    configure_torch_backends()
    cfg = classroom_cfg(args.envs, args.seed)
    agent = load_rl_cfg(TASK)
    agent.logger = "tensorboard"
    agent.upload_model = False
    agent.seed = args.seed
    agent.max_iterations = args.iterations
    agent.save_interval = (args.iterations - 1) // 2
    if args.warmstart:
        # 이미 서는 actor는 작은 고정 학습률로 조정하고 새 과제의 critic은 다시 학습한다.
        agent.algorithm.learning_rate = 1e-5
        agent.algorithm.schedule = "fixed"
    dump_yaml(args.out / "env.yaml", asdict(cfg))
    dump_yaml(args.out / "agent.yaml", asdict(agent))
    env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
    try:
        vec = RslRlVecEnvWrapper(env, clip_actions=agent.clip_actions)
        runner = ClassroomRunner(vec, asdict(agent), str(args.out), "cuda:0")
        if args.warmstart:
            runner.load(str(args.warmstart), load_cfg={"actor": True}, map_location="cuda:0")
            env.common_step_counter = 0
        # learn 내부에서 rollout → GAE → clipped PPO loss → 역전파와 Adam 업데이트.
        runner.learn(num_learning_iterations=args.iterations)
        stages = {"early": 0, "middle": agent.save_interval, "late": args.iterations - 1}
        for index in stages.values():
            for ext in ("pt", "onnx"):
                if not (args.out / f"model_{index}.{ext}").is_file():
                    raise RuntimeError(f"체크포인트/ONNX 내보내기 실패: model_{index}.{ext}")
        (args.out / "stages.json").write_text(json.dumps(stages, indent=2))
    finally:
        env.close()


if __name__ == "__main__":
    main()
