"""Read the knee by name; log one sample per twenty received messages."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class KneeWatch(Node):
    def __init__(self):
        super().__init__("knee_watch")
        self.count = 0
        self.create_subscription(
            JointState, "joint_states", self.receive, qos_profile_sensor_data)

    def receive(self, msg):
        if "knee_joint" not in msg.name or len(msg.name) != len(msg.position):
            self.get_logger().warning("Unexpected JointState layout")
            return
        self.count += 1
        if self.count % 20 == 1:
            angle = msg.position[msg.name.index("knee_joint")]
            self.get_logger().info(f"sample={self.count} angle={angle:+.3f} rad")


def main():
    rclpy.init()
    node = KneeWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
