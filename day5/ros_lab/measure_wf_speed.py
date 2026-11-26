"""Read the WF simulation odometry and compare forward twist with displacement."""
import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=3.)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be positive and finite')
    rclpy.init()
    node = rclpy.create_node('measure_wf_speed')
    samples = []

    def receive(msg):
        pose, v = msg.pose.pose, msg.twist.twist.linear.x
        q, stamp = pose.orientation, msg.header.stamp
        heading = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        samples.append((stamp.sec+stamp.nanosec*1e-9, pose.position.x, pose.position.y, heading, v))

    node.create_subscription(Odometry, '/tron2/odom', receive, 10)
    deadline = time.monotonic() + args.seconds + 10
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if len(samples) >= 2 and samples[-1][0]-samples[0][0] >= args.seconds:
                break
        else:
            raise SystemExit('Insufficient /tron2/odom samples. Check the launch and ROS_DOMAIN_ID.')
        distance = twist_integral = 0.
        for (t, x, y, yaw, v), (t2, x2, y2, _, v2) in zip(samples, samples[1:]):
            distance += (x2-x)*math.cos(yaw) + (y2-y)*math.sin(yaw)
            twist_integral += (v+v2)*.5*(t2-t)
        dt = samples[-1][0] - samples[0][0]
        print(f'{dt:.2f} s, {len(samples)} samples, body heading displacement {distance:+.3f} m, '
              f'pose rate {distance/dt:+.3f} m/s, mean twist {twist_integral/dt:+.3f} m/s')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
