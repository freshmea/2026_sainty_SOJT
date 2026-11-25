"""Check signed straight-line velocities and yaw reversals in one continuous rollout."""
import argparse
import json
import math

import numpy as np

from train_tron2_wf import ROOT, WheelDrive, session
from view_tron2_wf import display_trial, wf_viewer


def run(env, actor, viewer, linear, angular):
    obs, _ = env.reset(seed=101, options={"speed": 0.})
    d = env.data
    period = env.model.opt.timestep * env.substeps
    input_name = actor.get_inputs()[0].name
    phases = [("forward", linear, 0.), ("reverse", -linear, 0.),
              ("left", 0., angular), ("right", 0., -angular)]
    rows = []
    for name, vx, wz in phases:
        env.speed = vx
        distances, turns = [], []
        wheel_start = d.qpos[env.q[env.wheel]].copy()
        for tick in range(round(2 / period)):
            # Replace only the three command slots; the observation history stays continuous.
            obs[-3:] = [vx, 0., wz]
            action = actor.run(None, {input_name: obs})[0].ravel()
            previous_xy = d.qpos[:2].copy()
            rotation = d.body("base_Link").xmat.reshape(3, 3)
            heading = math.atan2(rotation[1, 0], rotation[0, 0])
            obs, _, fallen, done, _ = env.step(action)
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
            delta = yaw - heading
            turns.append(math.atan2(math.sin(delta), math.cos(delta)))
            displacement = d.qpos[:2] - previous_xy
            distances.append(float(displacement @ [math.cos(heading), math.sin(heading)]))
            display_trial(viewer, "WF command\nTime\nTarget vx / yaw rate\nMeasured vx / yaw rate",
                          f"{name}\n{d.time:.2f} s\n{vx:+.2f} m/s, {wz:+.2f} rad/s\n"
                          f"{distances[-1]/period:+.2f} m/s, {turns[-1]/period:+.2f} rad/s", period)
            if fallen or done:
                break
        # Allow 0.6 s to reverse direction, then require >=50% of requested signed speed.
        offset = round(.6 / period)
        measurements = distances if vx else turns
        target = vx or wz
        steady = float(np.mean(measurements[offset:]) / period) if len(measurements) > offset else None
        passed = (not fallen and len(measurements) == round(2 / period) and steady is not None
                  and steady / target >= .5 and sum(measurements) / target > 0)
        row = dict(phase=name, vx_cmd=vx, yaw_rate_cmd=wz, body_forward_m=round(sum(distances), 3),
                   yaw_change_deg=round(math.degrees(sum(turns)), 2), steady_rate=steady,
                   wheel_turns=((d.qpos[env.q[env.wheel]]-wheel_start)/(2*np.pi)).round(3).tolist(),
                   fallen=fallen, passed=passed)
        print(row, flush=True)
        rows.append(row)
        if fallen:
            break
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speed", type=float, default=.4)
    parser.add_argument("--yaw-rate", type=float, default=.8)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--viewer", action="store_true", help="watch the automatic direction check")
    mode.add_argument("--controller", action="store_true", help="drive manually with on-screen buttons")
    args = parser.parse_args()
    if not (0 < args.speed <= .7 and 0 < args.yaw_rate <= 1.):
        parser.error("speed must be in (0, 0.7] m/s and yaw-rate in (0, 1] rad/s")
    actor = session("policy.onnx")
    with WheelDrive() as env:
        if args.controller:
            from wf_controller import drive
            drive(env, actor, args.speed, args.yaw_rate)
            return
        env.reset(options={"speed": 0.})
        with wf_viewer(env, args.viewer) as viewer:
            rows = run(env, actor, viewer, args.speed, args.yaw_rate)
    passed = len(rows) == 4 and all(row["passed"] for row in rows)
    report = dict(policy="official WF ONNX", phase_s=2, settling_s=.6,
                  minimum_signed_rate_fraction=.5, phases=rows, passed=passed)
    path = ROOT / "results/wf_direction.json"
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(path)
    if not passed:
        raise SystemExit("Direction check failed")


if __name__ == "__main__":
    main()
