"""WF position targets, in radians, for the loopback simulator."""
import math
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_path
from limxsdk import datatypes
from sensor_msgs.msg import JointState

COMMAND_TOPIC = '/tron2/joint_commands'


def limits():
    model = get_package_share_path('tron2_description') / 'urdf' / 'wf_tron2a.urdf'
    bounds = {}
    for joint in ElementTree.parse(model).findall("joint[@type='revolute']"):
        name = joint.get('name')
        limit = joint.find('limit')
        bounds[name] = (-math.pi, math.pi) if name.startswith('wheel_') else (
            float(limit.get('lower')), float(limit.get('upper')))
    return bounds


class PositionControl:
    def __init__(self, node, robot, wire):
        self.robot = robot
        self.node = node
        self.wire = wire
        self.bounds = limits()
        if set(self.bounds) != set(wire):
            raise ValueError('WF URDF and SDK motor names differ')
        self.command = None
        node.create_subscription(JointState, COMMAND_TOPIC, self.receive, 1)

    def receive(self, msg):
        # The panel sends one complete named pose. Reject partial or ambiguous poses.
        if (len(msg.name) != len(self.wire) or len(msg.position) != len(self.wire)
                or set(msg.name) != set(self.wire)):
            self.node.get_logger().warning('Expected one position for each of the ten WF motors')
            return
        targets = dict(zip(msg.name, msg.position))
        for name, value in targets.items():
            low, high = self.bounds[name]
            if not (low <= value <= high):
                self.node.get_logger().warning(f'Invalid target for {name}: {value}')
                return
        cmd = datatypes.RobotCmd()
        cmd.motor_names = list(self.wire)
        cmd.q = [targets[name] for name in self.wire]
        cmd.dq = [0.0] * len(self.wire)
        cmd.tau = [0.0] * len(self.wire)
        cmd.mode = [0] * len(self.wire)
        # Position servos for the lab, including wheel angle over one revolution.
        gains = [(8.0, 0.8) if n.startswith('wheel_') else
                 (53.22, 3.39) if '_yaw_' in n else (159.67, 10.16) for n in self.wire]
        cmd.Kp, cmd.Kd = map(list, zip(*gains))
        self.command = cmd

    def update(self):
        if self.command is not None:
            self.command.stamp = self.node.get_clock().now().nanoseconds
            self.robot.publishRobotCmd(self.command)
