"""Check live ROS messages, write a rosbag2 bag and verify its serialized samples."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.serialization import serialize_message
import rosbag2_py as bag
from sensor_msgs.msg import Imu, JointState


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("robot", choices=["k1", "tron2"])
    parser.add_argument("joint_count", type=int)
    parser.add_argument("output", type=Path)
    parser.add_argument("--count", type=int, default=60)
    parser.add_argument("--passive", action="store_true", help="receive only; use for replay")
    args = parser.parse_args()
    if args.count < 2:
        parser.error("at least two samples required")
    args.output.mkdir(parents=True, exist_ok=True)
    bag_path = args.output / "telemetry"
    storage = bag.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter = bag.ConverterOptions("", "")
    topics = {f"/{args.robot}/joint_states": JointState, f"/{args.robot}/imu": Imu}
    samples = {t: [] for t in topics}
    stamps = {}
    names = []
    rclpy.init()
    node = rclpy.create_node(f"{args.robot}_lab_observer")
    if args.robot == "k1" and not args.passive:
        from geometry_msgs.msg import Twist
        cmd_pub = node.create_publisher(Twist, "/k1/cmd_vel", 10)
        command = Twist()
        command.linear.x = 0.15
        node.create_timer(0.1, lambda: cmd_pub.publish(command))
    writer = bag.SequentialWriter()
    writer.open(storage, converter)

    def receive(topic, msg):
        rows = samples[topic]
        if len(rows) == args.count:
            return
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp <= stamps.get(topic, -1):
            raise ValueError(f"{topic}: non-increasing source time")
        stamps[topic] = stamp
        if isinstance(msg, JointState):
            if not names:
                names.extend(msg.name)
            if (list(msg.name) != names or len(set(names)) != args.joint_count or
                    any(len(v) != args.joint_count for v in (names, msg.position, msg.velocity, msg.effort))):
                raise ValueError("joint names, order or widths changed")
            values = [*msg.position, *msg.velocity, *msg.effort]
        else:
            q = msg.orientation
            norm = math.sqrt(sum(x*x for x in (q.x, q.y, q.z, q.w)))
            if not math.isclose(norm, 1.0, abs_tol=0.01):
                raise ValueError("invalid IMU quaternion")
            values = [getattr(vector, axis) for vector in (msg.angular_velocity, msg.linear_acceleration)
                      for axis in ("x", "y", "z")]
        if not all(map(math.isfinite, values)):
            raise ValueError("non-finite sample")
        raw = serialize_message(msg)
        writer.write(topic, raw, node.get_clock().now().nanoseconds)
        rows.append((stamp, hashlib.sha256(raw).hexdigest()))

    for topic, cls in topics.items():
        writer.create_topic(bag.TopicMetadata(name=topic, type=f"sensor_msgs/msg/{cls.__name__}", serialization_format="cdr"))
        node.create_subscription(cls, topic, lambda msg, t=topic: receive(t, msg), 10)
    deadline = time.monotonic() + 50
    try:
        while min(map(len, samples.values())) < args.count:
            if time.monotonic() >= deadline:
                raise TimeoutError(str({k: len(v) for k, v in samples.items()}))
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        del writer
        node.destroy_node()
        rclpy.shutdown()
    reader = bag.SequentialReader()
    reader.open(storage, converter)
    digests = {topic: [] for topic in topics}
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        digests[topic].append(hashlib.sha256(payload).hexdigest())
    report = {"robot": args.robot, "joint_order": names, "topics": {}}
    for topic, rows in samples.items():
        if digests[topic] != [digest for _, digest in rows]:
            raise ValueError(f"{topic}: bag differs from live stream")
        report["topics"][topic] = {"live": len(rows), "bag": len(digests[topic]),
                                   "first_stamp_ns": rows[0][0], "last_stamp_ns": rows[-1][0]}
    report["status"] = "PASS"
    (args.output / "receipt.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()
