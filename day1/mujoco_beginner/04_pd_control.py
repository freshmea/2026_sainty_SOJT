"""P/PD/zero 비교. qpos 강제 대입 없이 모터 토크로 목표각을 추종한다."""
import argparse
import time
import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from common import load_model, save_csv, save_json, RESULTS


def run(kp, kd, target_deg, seconds=4.0, viewer_enabled=False):
    model, data = load_model("arm.xml")
    target = np.deg2rad(target_deg)
    rows = []
    viewer = None
    if viewer_enabled:
        from mujoco import viewer as mjviewer
        viewer = mjviewer.launch_passive(model, data)
        with viewer.lock():
            viewer.cam.lookat[:] = [0.2, 0, 0.5]
            viewer.cam.distance = 1.6
            viewer.cam.elevation = -40
    try:
        for _ in range(round(seconds / model.opt.timestep)):
            if viewer is not None and not viewer.is_running():
                break
            start = time.perf_counter()
            q = float(data.joint("hinge").qpos[0])
            dq = float(data.joint("hinge").qvel[0])
            raw = kp * (target - q) - kd * dq
            torque = float(np.clip(raw, -2.0, 2.0))
            data.actuator("motor").ctrl[0] = torque
            # 시작 상태와 그 상태에서 계산한 입력을 같은 행에 기록한다.
            rows.append([data.time, q, dq, target, torque, raw])
            mujoco.mj_step(model, data)
            if viewer is not None:
                viewer.sync()
                time.sleep(max(0.0, model.opt.timestep - (time.perf_counter() - start)))
    finally:
        if viewer is not None:
            viewer.close()
    a = np.asarray(rows)
    if not len(a):
        raise RuntimeError("Viewer closed before the first sample.")
    return a, {
        "kp": kp, "kd": kd, "target_deg": target_deg,
        "final_error_deg": float(np.rad2deg(abs(target-data.joint('hinge').qpos[0]))),
        "rmse_deg": float(np.rad2deg(np.sqrt(np.mean((a[:, 1]-target)**2)))),
        "saturation_fraction": float(np.mean(np.abs(a[:, 5]) > 2.0)),
        "completed_time_s": float(data.time),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--kp", type=float, default=4.0)
    parser.add_argument("--kd", type=float, default=0.6)
    parser.add_argument("--target", type=float, default=45.0)
    parser.add_argument("--viewer", action="store_true")
    args = parser.parse_args()
    reports = {}
    fig, ax = plt.subplots(figsize=(8, 4))
    for name, kp, kd in [("zero", 0, 0), ("P", args.kp, 0), ("PD", args.kp, args.kd)]:
        a, report = run(kp, kd, args.target, viewer_enabled=args.viewer and name == "PD")
        reports[name] = report
        save_csv(f"04_{name}.csv", ["time_s", "angle_rad", "speed_rad_s", "target_rad", "torque_Nm", "raw_torque_Nm"], a)
        ax.plot(a[:, 0], np.rad2deg(a[:, 1]), label=name)
    ax.axhline(args.target, color="black", linestyle="--", label="target")
    ax.set(xlabel="Simulation time (s)", ylabel="Angle (deg)", title="One-joint arm: zero / P / PD")
    ax.legend()
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(RESULTS / "04_pd_comparison.png", dpi=160)
    plt.close(fig)
    save_json("04_pd_metrics.json", reports)
    print(reports)


if __name__ == "__main__":
    main()
