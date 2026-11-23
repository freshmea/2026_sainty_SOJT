"""MuJoCo simulation-only PD tracking on an explicitly educational hinge."""
from pathlib import Path
import csv
import json
import mujoco
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "results/lab02"
OUT.mkdir(parents=True, exist_ok=True)
model = mujoco.MjModel.from_xml_string((ROOT / "models/teaching_hinge.xml").read_text(encoding="utf-8"))
data = mujoco.MjData(model)
kp, kd, control_stride = 12.0, 0.8, 5
rows = []
target = 0.0
for step in range(3000):
    if step % control_stride == 0:
        target = 0.4 * np.sin(2 * np.pi * 0.25 * data.time)
        data.ctrl[0] = np.clip(kp * (target - data.qpos[0]) - kd * data.qvel[0], -2, 2)
    mujoco.mj_step(model, data)
    rows.append([data.time, target, data.qpos[0], data.qvel[0], data.ctrl[0], data.sensordata[0]])
values = np.asarray(rows)
rmse = float(np.sqrt(np.mean((values[500:, 1] - values[500:, 2]) ** 2)))
assert np.isfinite(values).all()
assert abs(data.time - 6.0) < 1e-8, data.time
assert rmse < 0.06, rmse
assert np.max(np.abs(values[:, 4])) <= 2.0
assert np.max(np.abs(values[:, 2])) < 1.0
mujoco.mj_resetData(model, data)
assert data.time == 0 and np.allclose(data.qpos, model.qpos0)
with (OUT / "tracking.csv").open("w", newline="", encoding="utf-8") as stream:
    writer = csv.writer(stream)
    writer.writerow(["time_s", "target_rad", "angle_rad", "velocity_rad_s", "torque_Nm", "sensor_angle_rad"])
    writer.writerows(rows)
fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
axes[0].plot(values[:, 0], values[:, 1], label="Target")
axes[0].plot(values[:, 0], values[:, 2], label="Measured angle")
axes[0].set_ylabel("Angle [rad]")
axes[0].legend()
axes[1].plot(values[:, 0], values[:, 4], color="#008c88")
axes[1].set_ylabel("Torque [N m]")
axes[1].set_xlabel("Simulation time [s]")
fig.suptitle("Educational hinge: 500 Hz physics / 100 Hz control")
fig.tight_layout()
fig.savefig(OUT / "tracking.png", dpi=160)
plt.close(fig)
result = {"status": "PASS", "model": "educational_hinge", "mujoco_version": mujoco.__version__,
          "physics_hz": 1 / model.opt.timestep, "control_hz": 1 / model.opt.timestep / control_stride,
          "simulated_seconds": 6.0, "steps": 3000, "tracking_rmse_rad_after_1s": rmse,
          "reset_verified": True, "limits_verified": True,
          "not_verified": ["robot-specific PD gains", "physical motor dynamics", "SDK bridge"]}
(OUT / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
print(json.dumps(result, indent=2))
