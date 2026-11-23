"""Print the joint/actuator worksheet for the pinned scene, with STAND applied."""
from pathlib import Path
import mujoco

ROOT = Path(__file__).resolve().parent.parent
SCENE = ROOT / "sources/microduck_rl/src/mjlab_microduck/robot/microduck/scene_walk.xml"

def standing_model():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("STAND").id)
    mujoco.mj_forward(model, data)
    return model, data

if __name__ == "__main__":
    model, data = standing_model()
    print(f"nq={model.nq} nv={model.nv} nu={model.nu} dt={model.opt.timestep}")
    print("STAND root xyz + wxyz:", data.qpos[:7])
    print("ctrl  actuator -> joint             qpos qvel   angle(rad) target(rad)")
    for i in range(model.nu):
        joint = model.actuator_trnid[i, 0]
        q = model.jnt_qposadr[joint]
        v = model.jnt_dofadr[joint]
        print(i, model.actuator(i).name, "->", model.joint(joint).name,
              q, v, round(data.qpos[q], 6), round(data.ctrl[i], 6))
    for i in range(model.nsensor):
        print("sensor", model.sensor(i).name, "address", model.sensor_adr[i],
              "width", model.sensor_dim[i])
