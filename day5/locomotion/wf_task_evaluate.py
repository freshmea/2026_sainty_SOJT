"""Compare WF ONNX files on paired flat-ground episodes, including failures."""

import argparse
from concurrent.futures import ProcessPoolExecutor
import csv
import math
from itertools import product
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import numpy as np
import mujoco

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "tron2_sdk"))
from train_tron2_wf import WheelDrive, session
from wf_task_runtime import place_fallen, Recovery


def evaluate(case):
    task, name, path, side, target, ramp, seed, encoder = case
    rng = np.random.default_rng(seed)
    with WheelDrive() as env:
        if encoder:
            env.encoder = session(encoder)
            env.encoder_input = env.encoder.get_inputs()[0].name
        original = session("policy.onnx")
        actor = session(path) if path else original
        key = actor.get_inputs()[0].name
        is_recovery_actor = actor.get_inputs()[0].shape == [42]
        if task == "recovery":
            yaw, angle = (
                (0.0, 0.0) if seed == 0 else rng.uniform([-0.5, -0.06], [0.5, 0.06])
            )
            obs = place_fallen(env, side, angle, yaw)
            recovery = Recovery(obs)
        else:
            obs, _ = env.reset(options={"speed": 0.0})
            if seed:
                env.data.qpos[env.q[~env.wheel]] += rng.uniform(-0.008, 0.008, 8)
                mujoco.mj_forward(env.model, env.data)
                obs = env.observe(first=True)
        dt = env.model.opt.timestep * env.substeps
        duration = 4.6 if task == "recovery" else 1 + ramp + 20.0
        body = env.data.body("base_Link")
        initial_z, initial_up = float(body.xpos[2]), float(body.xmat[8])
        history = []
        status = "timeout"
        for tick in range(round(duration / dt)):
            time = tick * dt
            if task == "recovery":
                env.speed = 0.0
                if is_recovery_actor and not recovery.balance:
                    action = actor.run(None, {key: recovery.input(obs)})[0]
                else:
                    balance = original if is_recovery_actor else actor
                    action = balance.run(None, {balance.get_inputs()[0].name: obs})[0]
            else:
                env.speed = (
                    0.0
                    if time < 1
                    else target * (min((time - 1) / ramp, 1.0) if ramp else 1.0)
                )
                obs[-3:] = [env.speed, 0, 0]
                action = actor.run(None, {key: obs})[0]
            xy = body.xpos[:2].copy()
            yaw = math.atan2(body.xmat[3], body.xmat[0])
            obs, _, fall, _, _ = env.step(action.ravel())
            v = (body.xpos[:2] - xy) @ [math.cos(yaw), math.sin(yaw)] / dt
            tilt = math.degrees(math.acos(np.clip(body.xmat[8], -1.0, 1.0)))
            history.append(
                (
                    v,
                    tilt,
                    env.data.subtree_com[body.id, 2],
                    float(abs(env.data.qpos[env.q[[3, 8]]]).mean()),
                )
            )
            if task == "recovery":
                status = recovery.update(env, fall)
                if status != "running":
                    break
            elif fall:
                status = "fall"
                break
        h = np.array(history)
        v = float(h[-100:, 0].mean())
        if task == "high_speed" and status != "fall":
            status = "tracking" if abs(v - target) <= 0.15 * abs(target) else "miss"
        accel = h[50:150] if len(h) > 50 else h
        return dict(
            task=task,
            actor=name,
            side=side,
            target=target,
            ramp=ramp,
            seed=seed,
            initial_height=round(initial_z, 4),
            initial_up=round(initial_up, 4),
            result=status,
            seconds=round(len(h) * dt, 2),
            last_speed=round(v, 4),
            peak_tilt_deg=round(h[:, 1].max(), 2),
            accel_tilt_deg=round(accel[:, 1].max(), 2),
            accel_com_m=round(accel[:, 2].mean(), 4),
            accel_knee_rad=round(accel[:, 3].mean(), 4),
            final_height=round(body.xpos[2], 4),
        )


def ablations(folder, trained):
    import torch
    from torch import nn
    from stable_baselines3 import PPO
    from wf_task_train import ActionMap, ExportActor

    class ReferenceOnly(nn.Module):
        def __init__(self, task):
            super().__init__()
            self.reference = ActionMap(task, 0.35)

        def forward(self, obs):
            return self.reference(obs, torch.zeros_like(obs[:4]))

    configurations = [
        ("recovery_prior", ReferenceOnly("recovery"), 42),
        ("speed_prior", ReferenceOnly("high_speed"), 40),
        (
            "same_net_no_flex",
            ExportActor(
                PPO.load(trained.with_name("ppo.zip"), device="cpu"),
                ActionMap("high_speed", 0.0),
            ),
            40,
        ),
    ]
    paths = {}
    for name, model, size in configurations:
        paths[name] = folder / (name + ".onnx")
        torch.onnx.export(
            model.eval(),
            torch.zeros(size),
            paths[name],
            input_names=["obs"],
            output_names=["actions"],
            opset_version=17,
        )
    return paths


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recovery", type=Path, required=True)
    parser.add_argument("--speed", type=Path, required=True)
    parser.add_argument("--straight", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=3)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument(
        "--isaac-export",
        type=Path,
        help="Compare the policy.onnx and encoder.onnx exported together by LimX play.py.",
    )
    parser.add_argument(
        "--isaac-speed-export",
        type=Path,
        help="High-speed Isaac variant export folder.",
    )
    parser.add_argument(
        "--isaac-recovery-export",
        type=Path,
        help="Recovery Isaac variant export folder.",
    )
    parser.add_argument(
        "--ablate",
        action="store_true",
        help="Also test reference actions and identical learned weights without knee flexion.",
    )
    args = parser.parse_args()
    with TemporaryDirectory() as scratch:
        run(args, Path(scratch))


def run(args, scratch):
    cases = []

    def speed(name, path, targets=(3.0, 5.0), encoder=None):
        for target, ramp, seed in product(targets, (0.0, 2.0), (0, 201, 202)):
            cases.append(
                ("high_speed", name, path, "standing", target, ramp, seed, encoder)
            )

    def recovery(name, path, encoder=None):
        for side, seed in product(("front", "back"), (0, 201, 202, 203, 204, 205)):
            cases.append(("recovery", name, path, side, 0.0, 0.0, seed, encoder))

    recovery("official", None)
    recovery("recovery", args.recovery.resolve())
    wide = (0.7, 1.0, 3.0, 5.0) if args.isaac_export else (3.0, 5.0)
    speed("official", None, wide)
    speed("crouch", args.speed.resolve(), wide)
    speed("straight", args.straight.resolve())
    if args.ablate:
        controls = ablations(scratch, args.speed)
        recovery("recovery_prior", controls["recovery_prior"])
        for name in ("speed_prior", "same_net_no_flex"):
            speed(name, controls[name])

    for name, directory in (
        ("isaac", args.isaac_export),
        ("isaac_speed", args.isaac_speed_export),
        ("isaac_recovery", args.isaac_recovery_export),
    ):
        if directory is None:
            continue
        actor = directory.resolve() / "policy.onnx"
        encoder = directory.resolve() / "encoder.onnx"
        if not actor.is_file() or not encoder.is_file():
            raise ValueError(f"Missing the policy/encoder pair in {directory}")
        if name != "isaac_speed":
            recovery(name, actor, encoder)
        if name != "isaac_recovery":
            speed(name, actor, (0.7, 1.0, 3.0, 5.0), encoder)
    with args.csv.open("x") as f, ProcessPoolExecutor(args.workers) as pool:
        writer = None
        for result in pool.map(evaluate, cases):
            if writer is None:
                writer = csv.DictWriter(f, fieldnames=list(result))
                writer.writeheader()
            writer.writerow(result)
            f.flush()


if __name__ == "__main__":
    main()
