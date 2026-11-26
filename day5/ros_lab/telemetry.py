"""Two read-only ROS topics, with the timestamp from the data source."""
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu, JointState
from rclpy.node import Node


def source_time(ns):
    sec, nanosec = divmod(int(ns), 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


class Telemetry(Node):
    def __init__(self, robot):
        super().__init__(f"{robot}_telemetry")
        self.joints = self.create_publisher(JointState, f"/{robot}/joint_states", 10)
        self.imu = self.create_publisher(Imu, f"/{robot}/imu", 10)

    def joint_sample(self, ns, names, q, dq, effort):
        msg = JointState()
        msg.header.stamp = source_time(ns)
        msg.name = list(names)
        msg.position, msg.velocity, msg.effort = (list(map(float, v)) for v in (q, dq, effort))
        self.joints.publish(msg)

    def imu_sample(self, ns, frame, quat, gyro, accel):
        msg = Imu()
        msg.header.stamp = source_time(ns)
        msg.header.frame_id = frame
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = map(float, quat)
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, gyro)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, accel)
        self.imu.publish(msg)
