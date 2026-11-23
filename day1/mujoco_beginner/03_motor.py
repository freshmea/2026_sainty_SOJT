"""일정 토크 입력: 목표각이 아니라 회전이 계속됨을 관찰한다."""
import mujoco
from common import load_model, save_csv

model, data = load_model("arm.xml")
rows = []
for _ in range(1000):
    data.actuator("motor").ctrl[0] = 0.1
    mujoco.mj_step(model, data)
    rows.append([data.time, data.joint("hinge").qpos[0], data.joint("hinge").qvel[0]])
path = save_csv("03_motor.csv", ["time_s", "angle_rad", "speed_rad_s"], rows)
print(f"2 s: angle={rows[-1][1]:.6f} rad; speed={rows[-1][2]:.6f} rad/s")
assert rows[-1][1] > 1.0
print("PASS: constant torque produced rotation.", path)
