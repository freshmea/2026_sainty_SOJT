"""Print the WF_TRON2A SDK motor order, MuJoCo array indices, and sensor layout."""
from pathlib import Path
import sys

import mujoco

SIM = Path(__file__).resolve().parent / "vendor/tron2_mujoco_sim"
sys.path.insert(0, str(SIM))
from tron2_sim.channels import JointChannel
from tron2_sim.variants import wf


def main():
    spec = wf.build("WF_TRON2A", "tron2a", None)
    xml = SIM / "robot-description" / spec.family_dir / spec.robot_type / "xml" / spec.model_candidates[0]
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    channel = JointChannel(spec.channels[0]).resolve(model)

    print(f"{spec.robot_type}  njnt={model.njnt}  nq={model.nq}  nv={model.nv}  "
          f"nu={model.nu}  nsensor={model.nsensor}  nsensordata={model.nsensordata}  "
          f"dt={model.opt.timestep:g} s")
    for jid in range(model.njnt):
        if model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_FREE:
            q = model.jnt_qposadr[jid]
            v = model.jnt_dofadr[jid]
            print(f"Free base {model.body(model.jnt_bodyid[jid]).name}: joint ID {jid}, "
                  f"qpos[{q}:{q+7}] (xyz, wxyz) = {data.qpos[q:q+7]}, qvel[{v}:{v+6}]")

    print("\n SDK  joint  qpos  qvel  ctrl  joint name              actuator           q0(rad)  ctrl range(Nm)")
    for sdk, (name, (q, v, act)) in enumerate(zip(channel.spec.joints, channel.map, strict=True)):
        jid = model.joint(name).id
        if model.actuator_trnid[act, 0] != jid:
            raise ValueError(f"Actuator {act} does not drive {name}")
        lo, hi = model.actuator_ctrlrange[act]
        print(f"{sdk:4}  {jid:5}  {q:4}  {v:4}  {act:4}  {name:<22}  "
              f"{model.actuator(act).name:<17}  {data.qpos[q]:7.3f}  [{lo:g}, {hi:g}]")

    print("\n  ID  sensordata      sensor                       initial value")
    for sid in range(model.nsensor):
        start = model.sensor_adr[sid]
        stop = start + model.sensor_dim[sid]
        span = f"[{start}:{stop}]"
        print(f"{sid:4}  {span:<14}  {model.sensor(sid).name:<27}  {data.sensordata[start:stop]}")


if __name__ == "__main__":
    main()
