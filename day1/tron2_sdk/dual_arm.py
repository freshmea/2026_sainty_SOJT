"""Move two elbows and two gripper drive joints on a simulated fixed-base fixture."""
from pathlib import Path
import csv
import json
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
source = ROOT/"vendor/tron2_mujoco_sim/robot-description/tron2a/DACH_TRON2A/xml/robot_grasper.xml"
spec = mujoco.MjSpec.from_file(str(source))
base = next(j for j in spec.joints if j.type == mujoco.mjtJoint.mjJNT_FREE)
spec.delete(base)
model = spec.compile()
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
joint_ids = model.actuator_trnid[:, 0]
q, v = model.jnt_qposadr[joint_ids], model.jnt_dofadr[joint_ids]
rest = data.qpos[q].copy()
names = ["elbow_L", "elbow_R", "grasper_L_drive_Joint_ctrl", "grasper_R_drive_Joint_ctrl"]
indices = np.array([model.actuator(name).id for name in names])
offset = np.array([-0.18, -0.12, 0.25, 0.20])
goal = rest.copy()
goal[indices] += offset
limits = model.jnt_range[joint_ids[indices]]
assert np.all((goal[indices] >= limits[:,0]) & (goal[indices] <= limits[:,1]))
out = ROOT/"results/dual_arm"
out.mkdir(parents=True, exist_ok=True)
with (out/"tracking.csv").open("w",newline="") as stream:
    writer = csv.writer(stream)
    writer.writerow(["time_s","actuator","target_rad","measured_rad","torque_nm","ncon"])
    for i in range(1500):
        target = rest + min(i / 500, 1.) * (goal-rest)
        torque = 35*(target - data.qpos[q]) - 1.2*data.qvel[v] + data.qfrc_bias[v]
        data.ctrl[:] = np.clip(torque, model.actuator_ctrlrange[:,0], model.actuator_ctrlrange[:,1])
        mujoco.mj_step(model, data)
        if i % 20 == 0:
            for name, idx in zip(names, indices):
                writer.writerow([data.time,name,target[idx],data.qpos[q[idx]],data.ctrl[idx],data.ncon])
assert np.isfinite(data.qpos).all()
report = {"fixture":"fixed base", "names":names, "target_rad":goal[indices].tolist(),
          "initial_rad":rest[indices].tolist(), "final_rad":data.qpos[q[indices]].tolist(),
          "seconds":data.time}
report["absolute_error_rad"] = np.abs(goal[indices]-data.qpos[q[indices]]).tolist()
(out/"report.json").write_text(json.dumps(report, indent=2))
print(report)
