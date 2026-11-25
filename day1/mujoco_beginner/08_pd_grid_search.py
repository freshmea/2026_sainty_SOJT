"""07번과 같은 로봇 팔에서 PD1~PD100의 평균 누적 보상을 비교한다."""
from importlib import import_module

import numpy as np

from common import save_csv

# 숫자로 시작하는 07번 파일에서 시뮬레이션 환경을 그대로 가져온다.
ArmEnv = import_module("07_gym_bridge").ArmEnv

# 양 끝값을 바꾸면 탐색 범위가 달라진다. 기본 범위에는 기존 (4.0, 0.6)도 들어 있다.
KP_VALUES = np.linspace(2.0, 20.0, 10)
KD_VALUES = np.linspace(0.2, 2.0, 10)
SEEDS = [11, 22, 33, 44, 55]


def mean_return(env, kp, kd):
    returns = []
    for seed in SEEDS:
        obs, _ = env.reset(seed=seed)
        total = 0.0
        while True:
            angle, velocity, target = map(float, obs)
            torque = kp * (target - angle) - kd * velocity
            action = np.array([np.clip(torque / 2.0, -1.0, 1.0)], dtype=np.float32)
            obs, reward, terminated, truncated, _ = env.step(action)
            total += reward
            if terminated or truncated:
                break
        returns.append(total)
    return float(np.mean(returns))


def main():
    results = []
    env = ArmEnv()
    try:
        for kp in KP_VALUES:
            for kd in KD_VALUES:
                mode = f"PD{len(results) + 1}"
                score = mean_return(env, kp, kd)
                results.append((mode, kp, kd, score))
                print(f"{mode:5}  kp={kp:6.2f}  kd={kd:5.2f}  평균 누적 보상={score:.6f}")
    finally:
        env.close()

    # 07번의 보상은 오차, 속도, 토크에 대한 벌점이므로 평균 누적 보상이 클수록 좋다.
    results.sort(key=lambda row: row[3], reverse=True)
    path = save_csv("08_pd_grid_search.csv", ["mode", "kp", "kd", "mean_return"], results)
    mode, kp, kd, score = results[0]
    print(f"1위 {mode}: kp={kp:.2f}, kd={kd:.2f}, 평균 누적 보상={score:.6f}")
    print(f"전체 순위: {path}")


if __name__ == "__main__":
    main()
