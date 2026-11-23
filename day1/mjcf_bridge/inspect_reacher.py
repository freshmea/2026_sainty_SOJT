"""Inspect the two-link arm from the DeepMind Control Suite."""
from pathlib import Path

import mujoco
import numpy as np


model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name("reacher.xml")))
data = mujoco.MjData(model)
print(f"nq={model.nq} nv={model.nv} nu={model.nu}")
for name in ("shoulder", "wrist"):
    joint = model.joint(name)
    print(f"{name}: qpos[{joint.qposadr[0]}], qvel[{joint.dofadr[0]}], "
          f"ctrl[{model.actuator(name).id}]")

for shoulder, wrist in [(0, 0), (0, 90), (90, 0)]:
    data.joint("shoulder").qpos[0] = np.deg2rad(shoulder)
    data.joint("wrist").qpos[0] = np.deg2rad(wrist)
    mujoco.mj_forward(model, data)
    xyz = np.round(data.geom("finger").xpos, 3)
    print(f"shoulder={shoulder:2d}° wrist={wrist:2d}° -> finger xyz (m) = {xyz}")
