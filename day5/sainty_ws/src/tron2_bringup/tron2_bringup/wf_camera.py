"""Render the LimX WF body camera for ROS 2."""

import math
import sys
import threading
import time

import mujoco
from sensor_msgs.msg import CameraInfo, Image

OPTICAL_FRAME = "tron2/wf_camera_optical"


class BodyCamera:
    """Render the latest sampled pose outside the physics loop, only on demand."""

    def __init__(self, node, model):
        self.node, self.model = node, model
        self.color = node.create_publisher(Image, "color", 1)
        self.depth = node.create_publisher(Image, "depth", 1)
        self.info = node.create_publisher(CameraInfo, "camera_info", 1)
        self.condition = threading.Condition()
        self.pending = None
        self.stopped = False
        self.next_frame = 0.0
        self.thread = threading.Thread(target=self.run, name="wf-rgbd", daemon=True)
        self.thread.start()

    def sample(self, data, stamp):
        now = time.monotonic()
        if now < self.next_frame:
            return
        self.next_frame = now + 1 / 15
        if not any(
            p.get_subscription_count() for p in (self.color, self.depth, self.info)
        ):
            return
        with self.condition:
            self.pending = (data.qpos.copy(), stamp)
            self.condition.notify()

    def run(self):
        try:
            self.render()
        except Exception as exc:
            if self.node.context.ok():
                self.node.get_logger().error(f"WF camera renderer stopped: {exc}")

    def render(self):
        width, height = 320, 240
        fovy = self.model.camera("d435_depth").fovy.item()
        f = height / (2 * math.tan(math.radians(fovy) / 2))
        info = CameraInfo(width=width, height=height, distortion_model="plumb_bob")
        x, y = (width - 1) / 2, (height - 1) / 2
        info.d = [0.0] * 5
        info.k = [f, 0.0, x, 0.0, f, y, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, x, 0.0, 0.0, f, y, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.header.frame_id = OPTICAL_FRAME
        snapshot = mujoco.MjData(self.model)
        renderer = None
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(
                        lambda: self.stopped or self.pending is not None
                    )
                    if self.stopped:
                        return
                    pose, stamp = self.pending
                    self.pending = None
                if renderer is None:
                    renderer = mujoco.Renderer(self.model, height=height, width=width)
                snapshot.qpos[:] = pose
                # Camera and lighting frames depend on the complete position stage.
                mujoco.mj_fwdPosition(self.model, snapshot)
                info.header.stamp = stamp
                renderer.update_scene(snapshot, camera="d435_depth")
                for publisher, encoding in (
                    (self.color, "rgb8"),
                    (self.depth, "32FC1"),
                ):
                    if not publisher.get_subscription_count():
                        continue
                    if encoding == "32FC1":
                        renderer.enable_depth_rendering()
                    else:
                        renderer.disable_depth_rendering()
                    pixels = renderer.render()
                    image = Image(
                        header=info.header,
                        width=width,
                        height=height,
                        encoding=encoding,
                        step=pixels.strides[0],
                        is_bigendian=int(sys.byteorder == "big"),
                    )
                    image.data = pixels.tobytes()
                    publisher.publish(image)
                self.info.publish(info)
        finally:
            if renderer is not None:
                renderer.close()

    def close(self):
        with self.condition:
            self.stopped = True
            self.condition.notify()
        self.thread.join()
