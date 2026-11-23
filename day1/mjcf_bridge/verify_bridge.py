"""Run both classroom checks and save their numerical evidence."""
from pathlib import Path
import json
import mujoco

import branch_motion
import reacher_motion


if __name__ == "__main__":
    report = {"mujoco": mujoco.__version__, "reacher": reacher_motion.experiment(),
              "pendulum": branch_motion.experiment(), "status": "PASS"}
    output = Path(__file__).with_name("results") / "bridge_checks.json"
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(report["status"], output)
