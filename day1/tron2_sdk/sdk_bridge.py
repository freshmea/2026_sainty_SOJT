"""Receive SF/WF/DA simulated RobotState and IMU through the actual LimX SDK."""
import argparse
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
    args = parser.parse_args()
    wire = {"SF_TRON2A": LEG_SF, "WF_TRON2A": LEG_WF, "DA_TRON2A": ARM_DA}[args.variant]
    rclpy.init()
    node = Telemetry("tron2")
    robot = Robot(RobotType.Tron2)
    if not robot.init(args.robot_ip):
        raise RuntimeError("SDK init failed")
    lock = threading.Lock()
    samples = {}

    def state(message):
        names = list(message.motor_names)
        if not any(names):
            names = wire
        if names != wire or len(message.q) != len(wire):
            return
        with lock:
            samples["state"] = (message.stamp, list(names), list(message.q),
                                list(message.dq), list(message.tau))

    def imu(message):
        with lock:
            samples["imu"] = (message.stamp, "base_imu", list(message.quat),
                              list(message.gyro), list(message.acc))

    robot.subscribeRobotState(state)
    robot.subscribeImuData(imu)
    sent = {}
    try:
        while rclpy.ok():
            with lock:
                latest = dict(samples)
            for key, publish in (("state", node.joint_sample), ("imu", node.imu_sample)):
                if key in latest and latest[key][0] != sent.get(key):
                    publish(*latest[key])
                    sent[key] = latest[key][0]
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
