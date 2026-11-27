"""Open the TRON2 live view using the installed project configuration."""
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = get_package_share_path('tron2_description') / 'rviz/wf_live.rviz'
    return LaunchDescription([
        Node(package='rviz2', executable='rviz2', namespace='tron2',
             arguments=['-d', str(config)], output='screen'),
    ])
