"""Run a continuous WF rollout and publish feedback from the integrated model."""
import math
import os
from pathlib import Path
import sys
import time

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from .wf_camera import BodyCamera
from tf2_ros import TransformBroadcaster


class WFActor:
    """Pair a WF actor with the velocity encoder from its own training run."""
    def __init__(self, model, encoder=None, recovery=False):
        self.model = model
        self.key = model.get_inputs()[0].name
        shape = model.get_inputs()[0].shape
        if shape not in ([[40], [42]] if recovery else [[40]]) or model.get_outputs()[0].shape != [10]:
            raise ValueError(f'Unexpected WF actor shape: {shape} -> {model.get_outputs()[0].shape}')
        self.staged = shape == [42]
        self.encoder = encoder
        if encoder is not None and (encoder.get_inputs()[0].shape != [340] or encoder.get_outputs()[0].shape != [3]):
            raise ValueError('WF history encoder must map 340 inputs to 3 velocity estimates')

    def act(self, obs, history, recovery=None):
        if self.encoder is not None:
            obs = obs.copy()
            obs[:3] = self.encoder.run(None, {self.encoder.get_inputs()[0].name: history.ravel()})[0]
        if self.staged:
            obs = recovery.input(obs)
        return self.model.run(None, {self.key: obs})[0].ravel()


class PolicyDrive(Node):
    def __init__(self, env):
        super().__init__('policy_drive')
        self.viewer = self.declare_parameter('viewer', True).value
        self.max_linear = self.declare_parameter('max_linear', .7).value
        self.max_yaw = self.declare_parameter('max_yaw', 1.).value
        self.command = (0., 0.)
        self.last_command = -math.inf
        self.create_subscription(Twist, 'cmd_vel', self.receive, 1)
        self.joints = self.create_publisher(JointState, 'joint_states', 10)
        self.odometry = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self)
        self.names = [env.model.actuator(int(i)).name + '_Joint' for i in env.act]
        self.camera = BodyCamera(self, env.model)
        self.body = env.model.body('base_Link').id
        self.velocity = np.empty(6)

    def receive(self, msg):
        vx, wz = msg.linear.x, msg.angular.z
        if math.isfinite(vx) and math.isfinite(wz):
            self.command = (max(-self.max_linear, min(self.max_linear, vx)),
                            max(-self.max_yaw, min(self.max_yaw, wz)))
        else:
            self.command = (0., 0.)
        self.last_command = time.monotonic()

    def publish_feedback(self, env):
        data = env.data
        stamp = self.get_clock().now().to_msg()
        self.camera.sample(data, stamp)
        joint = JointState()
        joint.header.stamp = stamp
        joint.name = self.names
        joint.position = data.qpos[env.q].tolist()
        joint.velocity = data.qvel[env.v].tolist()
        joint.effort = data.actuator_force[env.act].tolist()
        self.joints.publish(joint)

        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'world'
        odom.child_frame_id = 'tron2/base_Link'
        pos, rot = odom.pose.pose.position, odom.pose.pose.orientation
        pos.x, pos.y, pos.z = map(float, data.xpos[self.body])
        rot.w, rot.x, rot.y, rot.z = map(float, data.xquat[self.body])
        # XBODY uses the URDF link frame. BODY uses rotated principal-inertia axes.
        mujoco.mj_objectVelocity(env.model, data, mujoco.mjtObj.mjOBJ_XBODY, self.body,
                                 self.velocity, 1)
        v, omega = odom.twist.twist.linear, odom.twist.twist.angular
        omega.x, omega.y, omega.z, v.x, v.y, v.z = map(float, self.velocity)
        self.odometry.publish(odom)
        tf = TransformStamped()
        tf.header, tf.child_frame_id = odom.header, odom.child_frame_id
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = pos.x, pos.y, pos.z
        tf.transform.rotation = rot
        self.tf.sendTransform(tf)

    def drive(self, env, actor, viewer, obs, rescue, balance, recovery):
        period = env.substeps * env.model.opt.timestep
        deadline = time.monotonic()
        preparation = None
        attempt = 1
        if rescue is not None:
            from wf_task_runtime import Recovery, RecoveryPreparation
        self.get_logger().info(f'WF inference ready at {1/period:.0f} Hz.')
        while rclpy.ok() and (viewer is None or viewer.is_running()):
            rclpy.spin_once(self, timeout_sec=0)
            if not rclpy.ok():
                break
            if preparation is not None:
                obs, ready = preparation.step(env)
                if ready:
                    preparation = None
                    recovery = Recovery(obs)
                    self.get_logger().info(f'Recovery attempt {attempt} started.')
            else:
                vx, wz = self.command if time.monotonic()-self.last_command < .25 else (0., 0.)
                if recovery is not None:
                    vx, wz = 0., 0.
                env.speed = vx
                obs[-3:] = (vx, 0., wz)
                active = actor
                if recovery is not None:
                    active = balance if recovery.balance and rescue.staged else rescue
                action = active.act(obs, env.history, recovery)
                obs, _, fallen, _, _ = env.step(action)
                if recovery is not None:
                    result = recovery.update(env, fallen)
                    if result == 'failed':
                        self.get_logger().warning(
                            f'Recovery attempt {attempt} failed. Repositioning for another attempt.')
                        recovery = None
                        attempt += 1
                        preparation = RecoveryPreparation(env, retry=True)
                    elif result == 'recovered':
                        self.get_logger().info('Recovered: two seconds upright. Drive control resumed.')
                        self.command = (0., 0.)
                        self.last_command = -math.inf
                        recovery = None
                elif fallen:
                    self.command = (0., 0.)
                    self.last_command = -math.inf
                    if rescue is None:
                        self.get_logger().warning('Fall detected. No recovery policy loaded. Resetting simulation upright.')
                        obs, _ = env.reset(options={'speed': 0.})
                    else:
                        env.speed = 0.
                        attempt = 1
                        preparation = RecoveryPreparation(env, retry=False)
                        self.get_logger().info('Fall detected. Settling before recovery.')
            self.publish_feedback(env)
            if viewer is not None:
                viewer.sync()
            deadline = max(deadline + period, time.monotonic())
            time.sleep(max(0., deadline-time.monotonic()))


def main():
    from .sim_session import reserve
    from contextlib import ExitStack
    with ExitStack() as stack:
        lease = reserve('mujoco', child=True)
        if lease is not None:
            stack.enter_context(lease)
        sys.path.insert(0, str(Path(os.environ['TRON2_SDK_ROOT']).resolve()))
        from train_tron2_wf import WEIGHTS, WheelDrive, session
        from view_tron2_wf import wf_viewer
        env = stack.enter_context(WheelDrive())
        vendor_encoder = (WEIGHTS / 'encoder.onnx').resolve()
        vendor_policy = (WEIGHTS / 'policy.onnx').resolve()
        sessions = {vendor_encoder: env.encoder}

        def get_session(path):
            if path not in sessions:
                sessions[path] = session(path)
            return sessions[path]

        balance = WFActor(get_session(vendor_policy))
        model_paths = {}

        def load_actor(role, default=None):
            raw = os.getenv(role + '_ONNX') or default
            if not raw:
                return None
            model = Path(raw).resolve()
            encoder = Path(os.getenv(role + '_ENCODER') or vendor_encoder).resolve()
            model_paths[role] = (model, encoder)
            # WheelDrive.observe() already filled obs[:3] with this encoder's result.
            override = None if encoder == vendor_encoder else get_session(encoder)
            return WFActor(get_session(model), override, recovery=(role == 'RECOVERY'))

        actor = load_actor('DRIVE', vendor_policy)
        rescue = load_actor('RECOVERY')
        recovery = None
        if rescue is not None:
            sys.path.insert(0, str(Path(os.environ['TRON2_SDK_ROOT']).parent/'locomotion'))
            from wf_task_runtime import Recovery, place_fallen
        obs, _ = env.reset(seed=101, options={'speed': 0.})
        pose = os.getenv('WF_START_POSE', 'standing')
        if pose != 'standing':
            obs = place_fallen(env, pose)
            recovery = Recovery(obs)
        rclpy.init()
        stack.callback(rclpy.try_shutdown)
        node = PolicyDrive(env)
        stack.callback(node.destroy_node)
        stack.callback(node.camera.close)
        for role, (model_path, encoder_path) in model_paths.items():
            node.get_logger().info(f'{role}: encoder={encoder_path} -> actor={model_path}')
        viewer = stack.enter_context(wf_viewer(env, node.viewer))
        node.drive(env, actor, viewer, obs, rescue, balance, recovery)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
