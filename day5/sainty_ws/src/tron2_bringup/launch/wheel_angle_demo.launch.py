"""Run the original ten-angle exercise on the loopback WF simulator."""
from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tron2_bringup.bringup import camera_window, default_sdk_root, robot_state_publisher
from tron2_bringup.sim_session import reserve_launch


def start_demo(context):
    reserve_launch(context, 'mujoco')
    root = Path(LaunchConfiguration('sdk_root').perform(context)).expanduser().resolve()
    python = str(root / '.venv/bin/python')
    rviz = get_package_share_path('tron2_description') / 'rviz/wf_live.rviz'
    return [
        camera_window(),
        robot_state_publisher(),
        Node(package='tron2_bringup', executable='mujoco_sim', namespace='tron2',
             additional_env={'TRON2_SDK_ROOT': str(root)}, output='screen',
             on_exit=Shutdown(reason='MuJoCo viewer closed')),
        ExecuteProcess(cmd=[python, str(root / 'sdk_bridge.py'), '--variant', 'WF_TRON2A',
                            '--robot-ip', '127.0.0.1', '--control'], output='screen',
                       on_exit=Shutdown(reason='Angle controller bridge exited')),
        ExecuteProcess(cmd=[python, '-m', 'tron2_bringup.wheel_angle_panel'], output='screen',
                       on_exit=Shutdown(reason='Angle panel closed')),
        Node(package='rviz2', executable='rviz2', namespace='tron2',
             arguments=['-d', str(rviz)], on_exit=Shutdown(reason='RViz closed')),
        TimerAction(period=6.0, actions=[Node(package='rqt_graph', executable='rqt_graph')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sdk_root', default_value=default_sdk_root()),
        DeclareLaunchArgument('camera_view', default_value='true', choices=['true','false']),
        OpaqueFunction(function=start_demo),
    ])
