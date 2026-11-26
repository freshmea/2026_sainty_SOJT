"""Common WF_TRON2A panel command contract for physical and simulated motors."""
from functools import lru_cache
from math import isfinite
from pathlib import Path

from time import monotonic
from xml.etree import ElementTree

import yaml
from ament_index_python.packages import get_package_share_path
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

TARGET = '/tron2/wf_targets'
ENABLED = '/tron2/panel_motion_enabled'
TIMEOUT = 0.25


@lru_cache(maxsize=1)
def wf_config():
    root = Path(__file__).resolve().parent
    path = root / 'vendor/tron2_rl_deploy_python/controllers/model/WF_TRON2A/params.yaml'
    return yaml.safe_load(path.read_text())['PointfootCfg']


def panel_wheel_speed():
    # Rolling speed at the configured maximum body command; this is a UI range.
    path = get_package_share_path('tron2_description') / 'urdf' / 'wf_tron2a.urdf'
    geom = ElementTree.parse(path).find("link[@name='wheel_L_Link']/collision/geometry/cylinder")
    return wf_config()['commands']['max']['lin_vel_x'] / float(geom.get('radius'))


def joint_limits():
    path = get_package_share_path('tron2_description') / 'urdf' / 'wf_tron2a.urdf'
    return {j.get('name'): tuple(float(j.find('limit').get(k)) for k in ('lower', 'upper'))
            for j in ElementTree.parse(path).findall("joint[@type='revolute']")}


def motor_order(names, wire):
    """Return URDF names in SDK packet order; the vendor WF blank layout is positional."""
    if not any(names):
        return list(wire)
    ordered = [n if n.endswith('_Joint') else n + '_Joint' for n in names]
    return ordered if len(ordered) == len(wire) and set(ordered) == set(wire) else None


def clip(value, low, high):
    return min(high, max(low, value))


class WheelFootServo:
    def __init__(self, node, robot, wire, allow_motion):
        self.node, self.robot, self.wire = node, robot, wire
        self.allow_motion = allow_motion
        self.bounds = joint_limits()
        if set(self.bounds) != set(wire):
            raise ValueError('WF model and SDK joint names differ')
        self.target = None
        self.active = False
        self.release_required = False
        self.last_status = 0.0
        self.control = wf_config()['control']
        self.period = 1.0 / wf_config()['loop_frequency']
        self.wheel_speed = panel_wheel_speed()
        self.measured_cycles = 0
        self.measure_start = None
        self.status = node.create_publisher(Bool, ENABLED, 1)
        node.create_subscription(JointState, TARGET, self.receive, 1)

    def receive(self, m):
        if not m.name:
            self.target = None
            self.release_required = False
            return
        age = (self.node.get_clock().now().nanoseconds -
               (m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)) / 1e9
        if (not self.allow_motion or self.release_required or not -0.05 <= age <= TIMEOUT
                or len(m.name) != len(self.wire) or set(m.name) != set(self.wire)
                or len(m.position) != len(m.name) or len(m.velocity) != len(m.name)):
            self.target = None
            return
        targets = {}
        for n, p, v in zip(m.name, m.position, m.velocity):
            low, high = self.bounds[n]
            if not (isfinite(p) and isfinite(v) and low <= p <= high
                    and abs(v) <= self.wheel_speed + 1e-9 and (p == 0 if n.startswith('wheel_') else v == 0)):
                self.target = None
                return
            targets[n] = v if n.startswith('wheel_') else p
        self.target = (monotonic(), targets)

    def stop(self):
        if not self.active:
            return
        from limxsdk.datatypes import RobotCmd
        cmd = RobotCmd()
        cmd.motor_names = self.raw_names
        size = len(self.order)
        cmd.mode = [0] * size
        cmd.q, cmd.dq, cmd.tau, cmd.Kp = ([0.] * size for _ in range(4))
        cmd.Kd = [self.control['wheel_joint_damping' if n.startswith('wheel_') else 'leg_joint_damping'] for n in self.order]
        cmd.stamp = self.node.get_clock().now().nanoseconds
        if self.robot.publishRobotCmd(cmd) is False:
            self.node.get_logger().error('SDK rejected damping command')
            return
        self.active = False
        self.measure_start = None
        self.measured_cycles = 0

    def update(self, sample):
        now = monotonic()
        fresh = sample is not None and now - sample[0] <= TIMEOUT
        if now - self.last_status > 0.1:
            self.status.publish(Bool(data=self.allow_motion and fresh))
            self.last_status = now
        if self.release_required or not fresh or self.target is None or now - self.target[0] > TIMEOUT:
            if self.active and (not fresh or self.target is not None):
                self.release_required = True
            self.stop()
            return
        _, raw, names, q, dq = sample
        if self.active and (raw != self.raw_names or names != self.order):
            self.release_required = True
            self.target = None
            self.stop()
            return
        from limxsdk.datatypes import RobotCmd
        self.raw_names, self.order = raw, names
        cmd = RobotCmd()
        cmd.motor_names = raw
        cmd.mode = [0] * len(names)
        cmd.tau = [0.] * len(names)
        cmd.q, cmd.dq, cmd.Kp, cmd.Kd = [], [], [], []
        for i, name in enumerate(names):
            target = self.target[1][name]
            if name.startswith('wheel_'):
                kd = self.control['wheel_joint_damping']
                limit = self.control['wheel_joint_torque_limit']
                p, kp = 0., 0.
                v = clip(target, dq[i] - limit/kd, dq[i] + limit/kd)
            else:
                group = 'light_joint' if '_yaw_' in name else 'leg_joint'
                kp = self.control[group + '_stiffness']
                kd = self.control[group + '_damping']
                cap = self.control[group + '_torque_limit']
                p = clip(target, q[i] + (kd*dq[i]-cap)/kp, q[i] + (kd*dq[i]+cap)/kp)
                p = clip(p, *self.bounds[name])
                # If the torque envelope and joint limits do not intersect at high speed,
                # keep the URDF position target valid and scale the requested effort.
                effort = abs(kp*(p-q[i]) - kd*dq[i])
                if effort > cap:
                    kp, kd = kp*cap/effort, kd*cap/effort
                v = 0.
            cmd.q.append(p)
            cmd.dq.append(v)
            cmd.Kp.append(kp)
            cmd.Kd.append(kd)
        cmd.stamp = self.node.get_clock().now().nanoseconds
        self.active = True
        if self.measure_start is None:
            self.measure_start = now
        self.measured_cycles += 1
        if self.measured_cycles == 1001:
            hz = 1000 / (now - self.measure_start)
            self.node.get_logger().info(f'WF command loop measured {hz:.1f} Hz over 1000 intervals')
        if self.robot.publishRobotCmd(cmd) is False:
            self.node.get_logger().error('SDK rejected position/velocity command')
            self.release_required = True
            self.target = None
            self.stop()
