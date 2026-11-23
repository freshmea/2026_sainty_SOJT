# -*- coding: utf-8 -*-
"""Check the pinned Microduck model and pretrained ONNX input/output contract."""
import json
import platform
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort
from PIL import Image

ROOT = Path(__file__).resolve().parent
MODEL = ROOT / "sources/microduck_rl/src/mjlab_microduck/robot/microduck"
RESULTS = ROOT / "validation"
RESULTS.mkdir(exist_ok=True)

assets = {
    path.relative_to(MODEL).as_posix(): path.read_bytes()
    for path in MODEL.rglob("*") if path.is_file()
}
model = mujoco.MjModel.from_xml_string(
    (MODEL / "scene_walk.xml").read_text(encoding="utf-8"), assets=assets
)
data = mujoco.MjData(model)
stand = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "STAND")
assert stand >= 0
mujoco.mj_resetDataKeyframe(model, data, stand)
mujoco.mj_forward(model, data)
assert (model.nq, model.nv, model.nu) == (21, 20, 14)

model.vis.global_.offwidth = 960
model.vis.global_.offheight = 720
camera = mujoco.MjvCamera()
camera.lookat[:] = [0, 0, 0.12]
camera.distance = 0.65
camera.azimuth = 130
camera.elevation = -18
with mujoco.Renderer(model, height=720, width=960) as renderer:
    renderer.update_scene(data, camera=camera)
    Image.fromarray(renderer.render()).save(RESULTS / "microduck_initial.png")

for _ in range(100):
    mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all()
    assert np.isfinite(data.qvel).all()

session = ort.InferenceSession(
    (ROOT / "policies/alpha_walking.onnx").read_bytes(),
    providers=["CPUExecutionProvider"],
)
input_info = session.get_inputs()[0]
output_info = session.get_outputs()[0]
assert input_info.shape == [1, 61]
assert output_info.shape == [1, 14]
rng = np.random.default_rng(20261116)
outputs = []
for _ in range(100):
    obs = np.zeros((1, 61), dtype=np.float32)
    obs[0, 3:6] = [0, 0, -1]
    obs[0, 48:51] = rng.uniform([-0.1, -0.05, -0.2], [0.1, 0.05, 0.2])
    action = session.run([output_info.name], {input_info.name: obs})[0]
    assert action.shape == (1, 14) and np.isfinite(action).all()
    outputs.append(action[0])
wrong_dimension_rejected = False
try:
    session.run(None, {input_info.name: np.zeros((1, 51), dtype=np.float32)})
except Exception as error:
    wrong_dimension_rejected = "INVALID_ARGUMENT" in str(error)
assert wrong_dimension_rejected

result = {
    "status": "PASS",
    "scope": "model load + 100 physics steps + 100 static observation ONNX calls",
    "not_verified": ["closed-loop walking", "BAM reference rollout", "PPO training"],
    "python": platform.python_version(),
    "platform": platform.platform(),
    "mujoco": mujoco.__version__,
    "numpy": np.__version__,
    "onnxruntime": ort.__version__,
    "nq": model.nq,
    "nv": model.nv,
    "nu": model.nu,
    "physics_steps": 100,
    "physics_timestep_s": model.opt.timestep,
    "simulation_time_s": float(data.time),
    "final_base_xyz": data.qpos[:3].tolist(),
    "onnx_inputs": {input_info.name: input_info.shape},
    "onnx_outputs": {output_info.name: output_info.shape},
    "onnx_calls": 100,
    "observation_fixture_seed": 20261116,
    "action_abs_max": float(np.max(np.abs(outputs))),
    "wrong_51D_observation_rejected": wrong_dimension_rejected,
    "providers": session.get_providers(),
    "render": "microduck_initial.png",
}
(RESULTS / "validation.json").write_text(
    json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
)
print(json.dumps(result, ensure_ascii=False, indent=2))
