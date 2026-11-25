"""Apply a measured horizontal impulse to the floating base and check for a fall."""
import argparse
import json
import math

import numpy as np

from train_tron2_wf import ROOT, WheelDrive, session
from view_tron2_wf import display_trial, wf_viewer


def trial(env, actor, input_name, viewer, force, axis, pulse_steps):
    obs, _ = env.reset(seed=101, options={"speed": 0.})
    d = env.data
    period = env.substeps * env.model.opt.timestep
    start = round(2 / period)
    body = env.model.body("base_Link").id
    peak_tilt = applied_impulse = 0.
    min_height = float(d.qpos[2])
    fallen = False
    try:
        while True:
            pushing = start <= env.ticks < start + pulse_steps
            d.xfrc_applied[body, axis] = force if pushing else 0.
            action = actor.run(None, {input_name: obs})[0].ravel()
            obs, _, fallen, done, _ = env.step(action)
            if pushing:
                applied_impulse += force * period
            tilt = math.degrees(math.acos(np.clip(d.body("base_Link").xmat[8], -1, 1)))
            peak_tilt = max(peak_tilt, tilt)
            min_height = min(min_height, float(d.qpos[2]))
            phase = "FALL" if fallen else ("PUSH" if pushing else "Observe")
            display_trial(viewer, "WF push trial\nTime / phase\nForce applied\nBody tilt",
                          f"{force:g} N on world {'xy'[axis]}\n{d.time:.2f} s  {phase}\n"
                          f"{d.xfrc_applied[body, axis]:g} N\n{tilt:.1f} deg", period)
            if fallen or done:
                break
    finally:
        d.xfrc_applied[body, :] = 0
    return dict(force_N=force, delivered_impulse_Ns=round(applied_impulse, 6),
                fallen=fallen, end_s=round(d.time, 3), peak_tilt_deg=round(peak_tilt, 2),
                min_base_height_m=round(min_height, 4))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", type=float, nargs="+", default=[50, 150, 400],
                        help="signed forces in newtons; a zero-force control is always run")
    parser.add_argument("--axis", choices=["x", "y"], default="y", help="world-frame direction")
    parser.add_argument("--duration", type=float, default=.2, help="pulse duration in seconds")
    parser.add_argument("--viewer", action="store_true")
    args = parser.parse_args()
    if not 0 < args.duration <= 6 or not np.isfinite(args.force).all():
        parser.error("use finite forces and a duration in (0, 6] seconds")
    actor = session("policy.onnx")
    with WheelDrive() as env:
        period = env.substeps * env.model.opt.timestep
        pulse_steps = max(1, round(args.duration / period))
        rows = []
        env.reset(options={"speed": 0.})
        with wf_viewer(env, args.viewer) as viewer:
            for force in dict.fromkeys([0., *args.force]):
                row = trial(env, actor, actor.get_inputs()[0].name, viewer, force,
                            "xy".index(args.axis), pulse_steps)
                rows.append(row)
                print(row, flush=True)
    report = dict(policy="official WF ONNX", speed_m_s=0, axis_world=args.axis,
                  push_start_s=2., pulse_s=pulse_steps*period, horizon_s=8.,
                  fall_rule="base upright cosine < 0.65 or base z < 0.40 m", trials=rows)
    path = ROOT / "results/wf_push.json"
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(path)
    if rows[0]["fallen"]:
        raise SystemExit("Control fell before the end of the observation period")


if __name__ == "__main__":
    main()
