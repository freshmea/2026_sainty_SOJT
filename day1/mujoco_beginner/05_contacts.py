"""공-바닥 접촉과 접촉 좌표계의 힘. 센서는 forward 후 읽는다."""
import numpy as np
import mujoco
from common import load_model, save_csv, save_json

model, data = load_model("ball.xml")
rows = []
first_contact = None
wrench = np.zeros(6)
for _ in range(1500):
    mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    normal_force = 0.0
    for i in range(data.ncon):
        c = data.contact[i]
        names = {model.geom(c.geom1).name, model.geom(c.geom2).name}
        if names == {"floor", "ball_geom"}:
            mujoco.mj_contactForce(model, data, i, wrench)
            normal_force += wrench[0]
            if first_contact is None:
                first_contact = float(data.time)
    rows.append([data.time, data.qpos[2], data.ncon, normal_force])
save_csv("05_contact.csv", ["time_s", "center_z_m", "ncon", "normal_force_N"], rows)
report = {"first_contact_s": first_contact, "final_center_z_m": float(data.qpos[2]),
          "mean_last_100_force_N": float(np.mean(np.asarray(rows)[-100:, 3])),
          "expected_static_weight_N": 0.2 * 9.81}
save_json("05_contact_metrics.json", report)
assert first_contact is not None
assert abs(report["mean_last_100_force_N"] - 1.962) < 0.05
print("PASS:", report)
