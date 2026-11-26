"""Inspect the installed WF camera models, optionally sample their running ROS 2 output."""

import argparse
import math
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_path
import mujoco
import numpy as np
import xacro


def rotation(rpy):
    roll, pitch, yaw = rpy
    x, y, z = np.cos(rpy)
    a, b, c = np.sin([roll, pitch, yaw])
    return np.array(
        [
            [z * y, z * b * a - c * x, z * b * x + c * a],
            [c * y, c * b * a + z * x, c * b * x - z * a],
            [-b, y * a, y * x],
        ]
    )


def pose(element, attribute):
    return np.fromstring(element.attrib[attribute], sep=" ")


def check_models():
    course = Path(__file__).resolve().parents[1]
    description = get_package_share_path("tron2_description") / "urdf/wf_tron2a.urdf"
    gz = get_package_share_path("tron2_gz")
    urdf = ET.parse(description)
    mount = urdf.find("joint[@name='d435_Joint']/origin")
    frame = urdf.find("joint[@name='wf_camera_optical_joint']")
    assert frame.find("parent").get("link") == "d435_Link"
    assert frame.find("child").get("link") == "wf_camera_optical"
    optical = rotation(pose(mount, "rpy")) @ rotation(pose(frame.find("origin"), "rpy"))
    pos = pose(mount, "xyz") + rotation(pose(mount, "rpy")) @ pose(
        frame.find("origin"), "xyz"
    )

    # Use precisely the XML used by the policy exercise.
    import sys

    sys.path.insert(0, str(course / "tron2_sdk"))
    from train_tron2_wf import XML

    model = mujoco.MjModel.from_xml_path(str(XML))
    camera = model.camera("d435_depth")
    assert model.body(int(camera.bodyid[0])).name == "d435_Link"
    d = mujoco.MjData(model)
    mujoco.mj_forward(model, d)
    base = d.body("base_Link")
    sim_cam = d.camera("d435_depth")
    orientation = (
        base.xmat.reshape(3, 3).T @ sim_cam.xmat.reshape(3, 3) @ np.diag([1, -1, -1])
    )
    np.testing.assert_allclose(orientation, optical, atol=2e-5)
    np.testing.assert_allclose(
        base.xmat.reshape(3, 3).T @ (sim_cam.xpos - base.xpos), pos, atol=1e-5
    )

    from tempfile import TemporaryDirectory

    with TemporaryDirectory() as tmp:
        path = Path(tmp) / "wf.urdf"
        path.write_text(xacro.process_file(str(gz / "urdf/wf_gz.urdf.xacro")).toxml())
        sdf = ET.fromstring(subprocess.check_output(["ign", "sdf", "-p", str(path)]))
    sensor = sdf.find(".//link[@name='base_Link']/sensor[@name='wf_rgbd']")
    assert sensor is not None and sensor.get("type") == "rgbd_camera"
    assert sensor.findtext("topic") == "/tron2/wf_rgbd"
    xyzrpy = np.fromstring(sensor.findtext("pose"), sep=" ")
    # Gazebo camera forward/left/up to ROS optical right/down/forward.
    np.testing.assert_allclose(
        rotation(xyzrpy[3:]) @ rotation([-math.pi / 2, 0, -math.pi / 2]),
        optical,
        atol=2e-5,
    )
    np.testing.assert_allclose(xyzrpy[:3], pos, atol=1e-5)
    hfov = float(sensor.findtext("camera/horizontal_fov"))
    width = int(sensor.findtext("camera/image/width"))
    height = int(sensor.findtext("camera/image/height"))
    vfov = math.degrees(2 * math.atan(math.tan(hfov / 2) * height / width))
    assert (width, height) == (320, 240) and abs(
        vfov - float(camera.fovy.item())
    ) < 0.001
    world = ET.parse(gz / "worlds/classroom.sdf")
    assert (
        world.find(
            ".//plugin[@name='ignition::gazebo::systems::Sensors']/render_engine"
        ).text
        == "ogre"
    )
    print(
        "PASS: URDF optical TF, MJCF mounted camera, final SDF RGB-D sensor and world renderer"
    )
    print(
        f"Matched camera origin {pos} m, vertical FOV {vfov:.3f} degrees, {width} x {height}"
    )


def check_topics(seconds):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    n = rclpy.create_node("wf_camera_exercise_check")
    tf = Buffer()
    listener = TransformListener(tf, n)
    messages, stamps = {}, {}

    def receive(key, msg):
        assert msg.header.frame_id == "tron2/wf_camera_optical"
        assert (msg.width, msg.height) == (320, 240)
        messages[key] = msg
        stamps.setdefault(key, set()).add(
            (msg.header.stamp.sec, msg.header.stamp.nanosec)
        )

    try:
        for key, msg_type in [
            ("color", Image),
            ("depth", Image),
            ("camera_info", CameraInfo),
        ]:
            n.create_subscription(
                msg_type,
                "/tron2/" + key,
                lambda m, k=key: receive(k, m),
                qos_profile_sensor_data,
            )
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
            if (
                len(stamps) == 3
                and min(map(len, stamps.values())) >= 3
                and tf.can_transform(
                    "tron2/base_Link", "tron2/wf_camera_optical", rclpy.time.Time()
                )
            ):
                break
        else:
            raise RuntimeError(
                f"No complete camera stream and optical TF within {seconds}s: {list(messages)}"
            )
        color, depth, info = (messages[k] for k in ("color", "depth", "camera_info"))
        assert color.encoding == "rgb8" and len(color.data) == color.height * color.step
        assert (
            depth.encoding == "32FC1" and len(depth.data) == depth.height * depth.step
        )
        assert info.k[0] > 0 and info.k[4] > 0
        print(
            "PASS: /tron2/color rgb8, /tron2/depth 32FC1, /tron2/camera_info and optical TF"
        )
    finally:
        listener.unregister()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--live", action="store_true", help="also check a running simulation"
    )
    parser.add_argument("--timeout", type=float, default=15)
    args = parser.parse_args()
    check_models()
    if args.live:
        check_topics(args.timeout)
