"""Plot measured held-out evaluations; training curves retain episode noise."""
from pathlib import Path
import argparse
import json
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
parser = argparse.ArgumentParser()
parser.add_argument("--results-dir", type=Path, default=root / "results")
args = parser.parse_args()
results = args.results_dir
reports = [json.loads(p.read_text(encoding="utf-8")) for p in sorted(results.glob("ppo_seed*/report.json"))]
assert reports, "Run training first"
plt.rcParams.update({"font.size": 18})
fig, ax = plt.subplots(figsize=(11, 5.5))
x = np.arange(len(reports))
for offset, key, label, color in [(-0.24, "baseline_zero", "Zero action", "#B3BFBA"),
                                 (0, "untrained_policy", "Untrained PPO", "#E0AC4E"),
                                 (0.24, "trained_policy", "Trained PPO", "#16796D")]:
    bars = ax.bar(x + offset, [r[key]["mean_return"] for r in reports], 0.24, label=label, color=color)
    ax.bar_label(bars, fmt="%.1f", fontsize=15, padding=3)
ax.set_xticks(x, [f"Seed {r['training_seed']}" for r in reports])
ax.set_ylabel("Mean episode return (20 resets)")
ax.set_ylim(0, 1300)
ax.legend(fontsize=14, loc="upper left", ncol=3)
ax.set_title("MuJoCo InvertedPendulum-v5: CPU PPO")
fig.tight_layout()
fig.savefig(results / "ppo_evaluation.png", dpi=160)
plt.close(fig)
fig, ax = plt.subplots(figsize=(11, 5.5))
for report in reports:
    path = results / f"ppo_seed{report['training_seed']}" / "monitor.csv"
    with path.open(encoding="utf-8") as stream:
        next(stream)
        episodes = list(csv.DictReader(stream))
    steps = np.cumsum([int(row["l"]) for row in episodes])
    returns = np.array([float(row["r"]) for row in episodes])
    smooth = np.array([returns[max(0, i-19):i+1].mean() for i in range(len(returns))])
    ax.plot(steps, smooth, label=f"Seed {report['training_seed']}")
ax.set_xlabel("Training environment steps")
ax.set_ylabel("Episode return (20-episode mean)")
ax.set_title("Training behavior differs from deterministic evaluation")
ax.legend()
fig.tight_layout()
fig.savefig(results / "ppo_training.png", dpi=160)
plt.close(fig)
