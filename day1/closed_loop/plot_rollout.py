# -*- coding: utf-8 -*-
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent / "results"
with (root / "rollout.csv").open(encoding="utf-8") as file:
    rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(file)]
plt.rcParams.update({"font.size": 17, "axes.spines.top": False, "axes.spines.right": False})
fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True, layout="constrained")
time = [r["time_s"] for r in rows]
axes[0].plot(time, [r["command_vx_m_s"] for r in rows], label="Command", color="#a66b22", linewidth=3)
axes[0].plot(time, [r["vx_world_m_s"] for r in rows], label="Measured", color="#237b70", linewidth=2)
axes[0].set_ylabel("Velocity (m/s)")
axes[0].legend(loc="upper left", ncol=2)
axes[1].plot(time, [r["trunk_height_m"] for r in rows], color="#21484b", linewidth=2)
axes[1].set_ylabel("Trunk height (m)")
axes[1].set_xlabel("Simulation time (s)")
axes[1].set_ylim(0.10, 0.13)
for axis in axes:
    axis.grid(alpha=0.2)
fig.suptitle("Microduck: legacy-actuator educational closed loop", fontsize=20)
fig.savefig(root / "microduck_closed_loop_metrics.png", dpi=150)
plt.close(fig)
