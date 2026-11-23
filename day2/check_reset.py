"""같은 seed의 초기 상태, 기울기로 끝난 에피소드, 시간 제한을 따로 관찰한다."""
from pathlib import Path
import json

import gymnasium as gym
import numpy as np

def first_episode(limit):
    with gym.make("InvertedPendulum-v5", max_episode_steps=limit) as env:
        first, _ = env.reset(seed=42)
        repeated, _ = env.reset(seed=42)
        assert np.array_equal(first, repeated)
        action = np.zeros(env.action_space.shape, dtype=env.action_space.dtype)
        for step in range(1, limit + 1):
            obs, reward, terminated, truncated, _ = env.step(action)
            if not np.isfinite(obs).all():
                raise ValueError("관측값에 NaN 또는 무한대가 있다.")
            if terminated or truncated:
                return dict(limit=limit, seed=42, first=first.tolist(), steps=step,
                            final_angle_rad=float(obs[1]), reward=float(reward),
                            terminated=bool(terminated), truncated=bool(truncated))
    raise RuntimeError("설정한 시간 제한까지 종료 신호가 없었다.")

rows = [first_episode(1000), first_episode(1)]
assert rows[0]["terminated"] and rows[1]["truncated"]
out = Path(__file__).resolve().parent / "results/reset.json"
out.parent.mkdir(exist_ok=True)
out.write_text(json.dumps(rows, indent=2), encoding="utf-8")
print(out, [(r["steps"], r["terminated"], r["truncated"]) for r in rows])
