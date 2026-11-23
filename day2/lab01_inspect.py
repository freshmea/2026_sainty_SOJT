"""Inspect MJCF and the Gymnasium environment contract without a robot."""
from pathlib import Path
import json
import gymnasium as gym
import mujoco
import numpy as np
from pendulum_env import make_env

root = Path(__file__).resolve().parent
out = root / "results" / "inspect"
out.mkdir(parents=True, exist_ok=True)
env = make_env()
obs, info = env.reset(seed=42)
base = env.unwrapped
model = base.model
rows = []
for step in range(100):
    obs, reward, terminated, truncated, info = env.step(np.zeros(1, dtype=np.float32))
    rows.append({"step": step, "obs": obs.tolist(), "reward": float(reward),
                 "terminated": bool(terminated), "truncated": bool(truncated)})
    assert np.isfinite(obs).all()
    if terminated or truncated:
        env.reset(seed=42 + step)
result = {"status": "PASS", "env": "InvertedPendulum-v5", "steps": len(rows),
          "nq": model.nq, "nv": model.nv, "nu": model.nu,
          "joint_names": [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)],
          "physics_timestep": model.opt.timestep, "frame_skip": base.frame_skip,
          "observation_shape": list(env.observation_space.shape),
          "action_bounds": [env.action_space.low.tolist(), env.action_space.high.tolist()],
          "rows": rows}
(out / "inspection.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
xml = Path(base.fullpath)
(root / "models" / "inverted_pendulum.xml").write_text(xml.read_text(encoding="utf-8"), encoding="utf-8")
env.close()
print(json.dumps({k: v for k, v in result.items() if k != "rows"}))
