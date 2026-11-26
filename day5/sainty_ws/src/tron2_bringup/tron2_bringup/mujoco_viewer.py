"""Run the official WF simulator with world pose feedback for RViz."""
import os
from pathlib import Path
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from .wf_camera import BodyCamera

ROOT = Path(os.environ.get('TRON2_SDK_ROOT', Path.home() / 'sainty/day5/tron2_sdk'))
sys.path.insert(0, str(ROOT / 'upstream' / 'tron2_mujoco_sim'))
from tron2_sim.core import SimCore
from tron2_sim.modules import SimModule
from tron2_sim.variants import wf


class BaseTransform(SimModule):
    name = 'ros_base_transform'

    def __init__(self, node):
        self.node = node
        self.broadcaster = TransformBroadcaster(node)
        self.next_publish = 0.0
        self.camera = None

    def attach(self, core):
        # qpos holds the post-integration pose of the floating root, also used by the viewer.
        joint = core.model.body('base_Link').jntadr[0]
        self.start = int(core.model.jnt_qposadr[joint])
        self.camera = BodyCamera(self.node, core.model)
        return True

    def on_publish(self, core):
        now = time.monotonic()
        if now < self.next_publish or not rclpy.ok():
            return
        self.next_publish = now + 0.02
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        self.camera.sample(core.data, t.header.stamp)
        t.header.frame_id, t.child_frame_id = 'world', 'tron2/base_Link'
        pos = t.transform.translation
        rot = t.transform.rotation
        pos.x, pos.y, pos.z, rot.w, rot.x, rot.y, rot.z = map(
            float, core.data.qpos[self.start:self.start + 7])
        self.broadcaster.sendTransform(t)


def main():
    os.environ['ROBOT_IP'] = '127.0.0.1'
    rclpy.init()
    node = rclpy.create_node('mujoco_base_pose', namespace='/tron2')
    pose = BaseTransform(node)
    spec = wf.build('WF_TRON2A', 'tron2a', None)
    spec.modules.append(pose)
    spec.publish_when_paused = True
    spec.cam = (2.7, -18.0)
    try:
        core = SimCore(spec, str(ROOT / 'vendor' / 'tron2_mujoco_sim'))
        if pose not in core.modules:
            raise RuntimeError('World pose module failed to attach')
        core.viewer.cam.lookat[:] = [0, 0, 0.34]
        core.run()
    finally:
        if pose.camera is not None:
            pose.camera.close()
        node.destroy_node()
        rclpy.try_shutdown()


    # Same native DDS teardown convention as the upstream viewer.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)
