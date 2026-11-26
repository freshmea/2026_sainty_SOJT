"""Publish a classroom joint angle and expose a motion-enable service."""
import math

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool


class KneeSource(Node):
    def __init__(self):
        super().__init__("knee_source")
        self.add_on_set_parameters_callback(self.validate)
        self.declare_parameter("target_rad", 0.30)
        self.angle = 0.0
        self.enabled = True
        self.publisher = self.create_publisher(
            JointState, "joint_states", qos_profile_sensor_data)
        self.create_service(SetBool, "enable", self.set_enabled)
        self.create_timer(0.05, self.sample)

    def validate(self, parameters):
        for parameter in parameters:
            if parameter.name == "target_rad":
                if (parameter.type_ != Parameter.Type.DOUBLE
                        or not math.isfinite(parameter.value)
                        or abs(parameter.value) > 1.0):
                    return SetParametersResult(
                        successful=False, reason="target_rad must be a double in [-1, 1]")
        return SetParametersResult(successful=True)

    def set_enabled(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = "tracking" if self.enabled else "holding"
        return response

    def sample(self):
        if self.enabled:
            error = self.get_parameter("target_rad").value - self.angle
            self.angle += max(-0.025, min(0.025, error))
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["knee_joint"]
        msg.position = [self.angle]
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = KneeSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
