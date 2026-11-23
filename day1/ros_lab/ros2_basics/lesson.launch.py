"""Classroom ROS 2 Humble example with one state source per mode."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build_nodes(context):
    folder = Path(__file__).resolve().parent
    mode = LaunchConfiguration("mode").perform(context)
    description = {"robot_description": (folder / "leg.urdf").read_text()}
    nodes = [Node(package="robot_state_publisher", executable="robot_state_publisher",
                  parameters=[description], output="screen")]
    if mode == "model":
        nodes.append(Node(package="joint_state_publisher", executable="joint_state_publisher",
                          parameters=[{"zeros.knee_joint": 0.2}], output="screen"))
    else:
        nodes.append(Node(package="controller_manager", executable="ros2_control_node",
                          parameters=[str(folder / "controllers.yaml")],
                          remappings=[("~/robot_description", "/robot_description")], output="screen"))
        for name in ("joint_state_broadcaster", "knee_controller"):
            nodes.append(Node(package="controller_manager", executable="spawner",
                              arguments=[name, "-c", "/controller_manager"], output="screen"))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="model", choices=["model", "control"]),
        OpaqueFunction(function=build_nodes),
    ])
