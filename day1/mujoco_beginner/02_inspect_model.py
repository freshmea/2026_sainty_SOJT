"""이름, 주소, 단위와 forward/reset의 의미를 확인한다."""
import numpy as np
import mujoco
from common import load_model

for filename in ["ball.xml", "arm.xml"]:
    model, data = load_model(filename)
    print(f"{filename}: nq={model.nq}, nv={model.nv}, nu={model.nu}")
    for j in range(model.njnt):
        print("joint:", model.joint(j).name,
              "qpos address:", model.jnt_qposadr[j],
              "qvel address:", model.jnt_dofadr[j])

model, data = load_model("arm.xml")
data.joint("hinge").qpos[0] = np.deg2rad(45)
mujoco.mj_forward(model, data)
print("45 deg in rad:", data.joint("hinge").qpos.copy())
print("tip world position:", data.site("tip").xpos.copy())
print("angle sensor:", data.sensor("angle").data.copy())
print("time after forward:", data.time)
assert data.time == 0
np.testing.assert_allclose(data.site("tip").xpos, [0.4/np.sqrt(2), 0.4/np.sqrt(2), 0.5])
mujoco.mj_resetData(model, data)
mujoco.mj_forward(model, data)
assert data.joint("hinge").qpos[0] == 0
print("PASS: named access, geometry and reset.")
