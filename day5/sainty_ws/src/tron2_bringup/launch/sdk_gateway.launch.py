from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from tron2_bringup.bringup import default_sdk_root, robot_state_publisher, sdk_bridge


def gateway(context):
    root = Path(LaunchConfiguration('sdk_root').perform(context)).expanduser().resolve()
    ip = LaunchConfiguration('robot_ip').perform(context).strip()
    if not ip or ip.startswith('-'):
        raise ValueError('robot_ip must name the robot SDK endpoint')
    python = LaunchConfiguration('sdk_python').perform(context) or root / '.venv/bin/python'
    motion = LaunchConfiguration('motion').perform(context) == 'true'
    return [robot_state_publisher(), sdk_bridge(root, python, ip, motion)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', description='Reachable LimX SDK endpoint.'),
        DeclareLaunchArgument('sdk_root', default_value=default_sdk_root()),
        DeclareLaunchArgument('sdk_python', default_value=''),
        DeclareLaunchArgument('motion', default_value='false', choices=['false', 'true']),
        OpaqueFunction(function=gateway),
    ])
