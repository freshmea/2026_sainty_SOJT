"""같은 관절과 목표에 한 조건씩 바꿔 오차와 토크 포화 시간을 비교한다."""
import csv
from pathlib import Path

import mujoco
import numpy as np

root = Path(__file__).resolve().parent
model = mujoco.MjModel.from_xml_path(str(root / "models/teaching_hinge.xml"))
out = root / "results/pd_comparison.csv"
out.parent.mkdir(exist_ok=True)

def measure(kp, kd, stride):
    data = mujoco.MjData(model)
    errors = []
    saturation_steps = 0
    for step in range(3000):
        target = 0.4 * np.sin(2 * np.pi * 0.25 * data.time)
        if step % stride == 0:
            torque = kp * (target - data.qpos[0]) - kd * data.qvel[0]
            data.ctrl[0] = np.clip(torque, -2, 2)
        if step >= 500:
            errors.append((target - data.qpos[0]) ** 2)
        saturation_steps += int(abs(data.ctrl[0]) >= 2)
        mujoco.mj_step(model, data)
    return float(np.sqrt(np.mean(errors))), saturation_steps * model.opt.timestep

with out.open("w", newline="") as stream:
    writer = csv.writer(stream)
    writer.writerow(["kp", "kd", "stride", "control_ms", "rmse_rad", "saturation_s"])
    for kp, kd, stride in [(12, .8, 5), (24, .8, 5), (12, 0, 5), (12, .8, 20)]:
        row = (kp, kd, stride, 1000 * stride * model.opt.timestep, *measure(kp, kd, stride))
        writer.writerow(row)
        print(*row)
