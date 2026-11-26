"""Write shared physically settled starting states for the Isaac recovery curriculum."""

import json
from pathlib import Path
import sys

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "tron2_sdk"))
from train_tron2_wf import WheelDrive
from wf_task_runtime import place_fallen


def main():
    rng = np.random.default_rng(73)
    samples = []
    with WheelDrive() as env:
        body = env.data.body("base_Link")
        names = [env.model.actuator(int(i)).name + "_Joint" for i in env.act]
        for side in ("front", "back"):
            for _ in range(16):
                place_fallen(
                    env,
                    side,
                    angle=rng.uniform(-0.04, 0.04),
                    yaw=rng.uniform(-0.4, 0.4),
                )
                velocity = np.empty(6)
                mujoco.mj_objectVelocity(
                    env.model, env.data, mujoco.mjtObj.mjOBJ_XBODY, body.id, velocity, 0
                )
                samples.append(
                    dict(
                        side=side,
                        height=float(body.xpos[2]),
                        wxyz=body.xquat.tolist(),
                        linear=velocity[3:].tolist(),
                        angular=velocity[:3].tolist(),
                        joint_pos=env.data.qpos[env.q].tolist(),
                        joint_vel=env.data.qvel[env.v].tolist(),
                    )
                )
    (HERE / "wf_fallen_samples.json").write_text(
        json.dumps({"joint_names": names, "states": samples}, indent=1) + "\n"
    )


if __name__ == "__main__":
    main()
