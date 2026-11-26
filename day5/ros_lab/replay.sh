#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
export PATH="$HOME/.local/bin:$PATH" ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
day="$(cd "$(dirname "$0")/.." && pwd)"
robot=${1:?select k1 or tron2}
input=$(realpath "${2:?pass the original ros result directory}")
case "$robot" in
  k1) project="$day/../day1/K1"; count=23 ;;
  tron2) project="$day/tron2_sdk"; count=10 ;;
  *) exit 2 ;;
esac
cd "$project"
test -f "$input/receipt.json"
out=$(mktemp -d "$input/replay-XXXXXXXX")
uv run --locked python "$day/ros_lab/observe.py" "$robot" "$count" "$out" --passive > "$out/receiver.log" 2>&1 &
observer=$!
trap 'kill "$observer" 2>/dev/null || true' EXIT
ros2 bag play "$input/telemetry" --delay 3 > "$out/player.log" 2>&1
wait "$observer"
uv run --locked python - "$input" "$out" <<'PYTHON'
from pathlib import Path
import json, sys, sqlite3
from sensor_msgs.msg import Imu, JointState
from rclpy.serialization import deserialize_message
def payloads(folder):
    db = next((folder/"telemetry").glob("*.db3"))
    with sqlite3.connect(db) as con:
        rows = con.execute("SELECT topics.name, messages.data FROM messages JOIN topics ON topics.id=messages.topic_id ORDER BY topics.name,messages.id")
        return [(t, deserialize_message(raw, Imu if t.endswith("/imu") else JointState)) for t, raw in rows]
a,b = map(Path, sys.argv[1:])
if payloads(a) != payloads(b):
    raise RuntimeError("replayed fields differ")
(b/"comparison.json").write_text(json.dumps({"all_fields_equal": True, "messages": len(payloads(a))}, indent=2))
print("PASS: original and replayed ROS messages match", b)
PYTHON
