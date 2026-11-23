"""기본 예제 실행과 교육 목표의 수치 검사. --rl, --render는 선택."""
import argparse
import json
import os
import platform
import subprocess
import sys
import numpy as np
import mujoco
from common import ROOT, RESULTS, save_json

parser = argparse.ArgumentParser()
parser.add_argument("--rl", action="store_true")
parser.add_argument("--render", action="store_true")
args = parser.parse_args()
scripts = ["00_first_sim.py", "02_inspect_model.py", "03_motor.py", "04_pd_control.py", "05_contacts.py"]
if args.rl:
    scripts.append("07_gym_bridge.py")
if args.render:
    scripts.append("06_render.py")
runs = []
for script in scripts:
    result = subprocess.run([sys.executable, str(ROOT/script)], cwd=ROOT, env={**os.environ, "PYTHONUTF8": "1"}, capture_output=True, text=True, encoding="utf-8", errors="replace")
    runs.append({"script": script, "exit_code": result.returncode, "stdout": result.stdout, "stderr": result.stderr})
    print(script, "PASS" if result.returncode == 0 else "FAIL", flush=True)
    if result.returncode:
        print(result.stderr)
report = {"python": sys.version, "platform": platform.platform(), "mujoco": mujoco.__version__, "numpy": np.__version__, "runs": runs}
save_json("verification.json", report)
assert all(r["exit_code"] == 0 for r in runs), "One or more examples failed; read results/verification.json"
pd = json.loads((RESULTS / "04_pd_metrics.json").read_text(encoding="utf-8"))
assert pd["PD"]["final_error_deg"] < 0.5
assert pd["PD"]["rmse_deg"] < pd["zero"]["rmse_deg"]
assert pd["PD"]["completed_time_s"] > 3.99
if args.rl:
    gym = json.loads((RESULTS / "07_gym_metrics.json").read_text(encoding="utf-8"))
    assert np.mean([e["return"] for e in gym["PD"]]) > np.mean([e["return"] for e in gym["zero"]])
print("ALL CHECKS PASS: simulation, model geometry, torque, tracking and contact.")
