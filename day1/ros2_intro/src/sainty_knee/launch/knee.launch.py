from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    folder = Path(get_package_share_directory("sainty_knee"))
    description = (folder / "urdf" / "leg.urdf").read_text()
    return LaunchDescription([
        Node(package="sainty_knee", executable="knee_source",
             namespace="classroom", output="screen"),
        Node(package="sainty_knee", executable="knee_watch",
             namespace="classroom", output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             namespace="classroom", parameters=[{"robot_description": description}]),
    ])
