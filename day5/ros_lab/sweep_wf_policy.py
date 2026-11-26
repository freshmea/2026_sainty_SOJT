"""Sweep signed WF policy inputs on flat MuJoCo ground without an SDK connection."""
import argparse
import csv
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tron2_sdk'))
from train_tron2_wf import WheelDrive, session


def trial(env, actor, input_name, axis, target, ramp_seconds, hold_seconds):
    obs, _ = env.reset(seed=101, options={'speed': 0.})
    dt = env.substeps * env.model.opt.timestep
    body = env.data.body('base_Link')
    rates = []
    max_tilt = 0.
    min_height = float('inf')
    last_yaw = math.atan2(body.xmat[3], body.xmat[0])
    for tick in range(round((1 + ramp_seconds + hold_seconds) / dt)):
        t = tick * dt
        if t < 1:
            cmd = 0.
        else:
            cmd = target * (min(1., (t-1)/ramp_seconds) if ramp_seconds else 1.)
        vx, wz = (cmd, 0.) if axis == 'forward' else (0., cmd)
        env.speed = vx
        obs[-3:] = [vx, 0., wz]
        xy = body.xpos[:2].copy()
        action = actor.run(None, {input_name: obs})[0].ravel()
        obs, _, fallen, _, _ = env.step(action)
        yaw = math.atan2(body.xmat[3], body.xmat[0])
        turn = yaw - last_yaw
        travel = (body.xpos[:2]-xy) @ [math.cos(last_yaw), math.sin(last_yaw)]
        rate = travel/dt if axis == 'forward' else math.atan2(math.sin(turn), math.cos(turn))/dt
        last_yaw = yaw
        max_tilt = max(max_tilt, math.degrees(math.acos(np.clip(body.xmat[8], -1., 1.))))
        min_height = min(min_height, body.xpos[2])
        if t >= 1 + ramp_seconds + hold_seconds - 2:
            rates.append(rate)
        if fallen:
            break
    return dict(axis=axis, target=target, ramp_s=ramp_seconds, hold_s=hold_seconds,
                sim_s=round(env.data.time, 2), fallen=fallen,
                steady_rate=round(float(np.mean(rates)), 4) if rates and not fallen else '',
                max_tilt_deg=round(max_tilt, 2), min_height_m=round(float(min_height), 4))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--linear', type=float, nargs='*', default=[.7, 1., 2., 3., 4., 5., -.7, -2., -5.])
    parser.add_argument('--yaw', type=float, nargs='*', default=[1., 2., 3., 4., -1., -2., -3., -4.])
    parser.add_argument('--hold-seconds', type=float, default=6.)
    args = parser.parse_args()
    if not math.isfinite(args.hold_seconds) or args.hold_seconds < 2.:
        parser.error('--hold-seconds must be finite and at least two seconds.')
    if not all(math.isfinite(x) for x in [*args.linear, *args.yaw]):
        parser.error('Targets must be finite.')
    with WheelDrive() as env:
        actor = session('policy.onnx')
        name = actor.get_inputs()[0].name
        writer = None
        for axis, values in [('forward', args.linear), ('yaw', args.yaw)]:
            for target in values:
                for ramp in [0., 2.]:
                    result = trial(env, actor, name, axis, target, ramp, args.hold_seconds)
                    if writer is None:
                        writer = csv.DictWriter(sys.stdout, fieldnames=list(result))
                        writer.writeheader()
                    writer.writerow(result)
                    sys.stdout.flush()


if __name__ == '__main__':
    main()
