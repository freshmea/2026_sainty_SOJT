#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
export PATH="$HOME/.local/bin:$PATH"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-86}"
day="$(cd "$(dirname "$0")/.." && pwd)"
robot=${1:-}
case "$robot" in
  k1) project="$day/K1"; count=23 ;;
  tron2) project="$day/tron2_sdk"; count=10 ;;
  *) echo "usage: ./run.sh {k1|tron2}" >&2; exit 2 ;;
esac
cd "$project"
uv sync --locked
mkdir -p results
out=$(mktemp -d "$PWD/results/ros-XXXXXXXX")
echo "Evidence directory: $out"
pids=()
finish() {
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap finish EXIT
trap 'exit 130' INT TERM
if [[ $robot == tron2 ]]; then
  uv run --locked fetch_models.py
  export ROBOT_IP=127.0.0.1 ROBOT_TYPE=SF_TRON2A
  setsid uv run --locked python vendor/tron2_mujoco_sim/simulator.py --headless --duration 75 > "$out/simulator.log" 2>&1 &
  pids+=("$!")
  setsid uv run --locked sdk_bridge.py > "$out/sdk.log" 2>&1 &
  pids+=("$!")
else
  setsid uv run --locked verify_policy.py --no-render --ros2 --output-dir "$out/sim" > "$out/simulator.log" 2>&1 &
  producer=$!
  pids+=("$producer")
fi
uv run --locked python "$day/ros_lab/observe.py" "$robot" "$count" "$out"
if [[ $robot == k1 ]]; then
  wait "$producer"
  uv run --locked python - "$out/sim/policy_rollout.csv" <<'PYTHON'
import csv, sys
with open(sys.argv[1]) as stream:
    commands = [float(row["command_x"]) for row in csv.DictReader(stream)]
if 0.15 not in commands or commands[-1] != 0.0:
    raise RuntimeError("K1 ROS command or timeout was not observed")
PYTHON
fi
ros2 bag info "$out/telemetry" > "$out/bag_info.txt"
