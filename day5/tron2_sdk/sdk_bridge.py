"""Bridge LimX SDK motor state and IMU to ROS 2 and route optional WF commands."""
import argparse
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
from limxsdk.robot import Robot, RobotType

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros_lab"))
from telemetry import Telemetry


sys.path.insert(0, str(Path(__file__).resolve().parent / "vendor/tron2_mujoco_sim"))
from tron2_sim.spec import ARM_DA, LEG_SF, LEG_WF


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ip", default="127.0.0.1")
    parser.add_argument("--variant", choices=["SF_TRON2A", "WF_TRON2A", "DA_TRON2A"], default="SF_TRON2A")
    parser.add_argument("--control", action="store_true", help="enable the WF simulation position controller")
    parser.add_argument("--wf-panel", action="store_true", help="use WF leg position / wheel velocity mode")
    parser.add_argument("--enable-motion", action="store_true")
    args = parser.parse_args()
    if args.wf_panel and (args.variant != "WF_TRON2A" or args.control):
        parser.error("--wf-panel requires WF_TRON2A and excludes the legacy angle controller")
    if args.enable_motion and not args.wf_panel:
        parser.error("--enable-motion requires --wf-panel")
    if args.control and (args.variant != "WF_TRON2A" or args.robot_ip != "127.0.0.1"):
        parser.error("--control uses the loopback WF_TRON2A simulator")
    wire = {"SF_TRON2A": LEG_SF, "WF_TRON2A": LEG_WF, "DA_TRON2A": ARM_DA}[args.variant]
    rclpy.init()
    node = Telemetry("tron2")
    robot = Robot(RobotType.Tron2)
    if not robot.init(args.robot_ip):
        raise RuntimeError("SDK init failed")
    lock = threading.Lock()
    samples = {}

    from wf_servo import WheelFootServo, motor_order
    motor_sample = None

    def state(message):
        nonlocal motor_sample
        raw = list(message.motor_names)
        names = motor_order(raw, wire)
        q, dq, tau = list(message.q), list(message.dq), list(message.tau)
        with lock:
            if (names is None or any(len(a) != len(wire) for a in (q, dq, tau))
                    or not all(math.isfinite(v) for a in (q, dq, tau) for v in a)):
                motor_sample = None
                return
            if "state" in samples and samples["state"][0] == message.stamp:
                return
            # Unnamed WF states follow the vendor wire order; reply with the SDK spelling.
            motor_sample = (time.monotonic(), raw if any(raw) else [""] * len(wire), names, q, dq)
            samples["state"] = (message.stamp, names, q, dq, tau)

    def imu(message):
        frame = "tron2/base_imu" if args.wf_panel or args.control else "base_imu"
        with lock:
            samples["imu"] = (message.stamp, frame, list(message.quat),
                              list(message.gyro), list(message.acc))

    robot.subscribeRobotState(state)
    robot.subscribeImuData(imu)
    control = None
    if args.control:
        from tron2_bringup.joint_control import PositionControl
        control = PositionControl(node, robot, wire)
    servo = WheelFootServo(node, robot, wire, args.enable_motion) if args.wf_panel else None
    sent = {}
    next_ros = 0.0
    deadline = time.monotonic()
    try:
        while rclpy.ok():
            if servo is not None or control is not None:
                rclpy.spin_once(node, timeout_sec=0)
            if control is not None:
                control.update()
            with lock:
                latest = dict(samples)
                current_motor = motor_sample
            if servo is not None:
                servo.update(current_motor)
            if servo is None or time.monotonic() >= next_ros:
                next_ros = time.monotonic() + 0.02
                for key, publish in (("state", node.joint_sample), ("imu", node.imu_sample)):
                    if key in latest and latest[key][0] != sent.get(key):
                        stamp, *fields = latest[key]
                        if control is not None or servo is not None:
                            # SDK stamps can use a monotonic epoch. RViz TF uses the ROS clock.
                            stamp = node.get_clock().now().nanoseconds
                        publish(stamp, *fields)
                        sent[key] = latest[key][0]
            if servo is None:
                time.sleep(0.02)
            else:
                deadline = max(deadline + servo.period, time.monotonic())
                time.sleep(max(0., deadline-time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        if servo is not None:
            servo.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
