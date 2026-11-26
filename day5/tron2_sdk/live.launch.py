from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def assemble(context):
    mode = LaunchConfiguration('mode').perform(context)
    motion = LaunchConfiguration('motion').perform(context) == 'true'
    endpoint = LaunchConfiguration('robot_ip').perform(context)
    sdk = Path(__file__).resolve().parent
    share = get_package_share_path('tron2_description')
    simulated = mode in ('sim', 'control')
    if mode == 'hardware' and not endpoint:
        raise ValueError('hardware requires an explicit robot_ip')
    sdk_python = (str(sdk / '.venv/bin/python') if simulated else
                  LaunchConfiguration('sdk_python').perform(context))
    processes = []
    if mode != 'operator':
        description = (share / 'urdf/wf_tron2a.urdf').read_text()
        processes.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                              namespace='tron2', output='screen', parameters=[{
                                  'robot_description': ParameterValue(description, value_type=str),
                                  'frame_prefix': 'tron2/', 'publish_frequency': 50.0}]))
        flags = ['--control'] if mode == 'control' else ['--wf-panel']
        if mode == 'sim' or (mode == 'hardware' and motion):
            flags.append('--enable-motion')
        processes.append(ExecuteProcess(cmd=[sdk_python, str(sdk / 'sdk_bridge.py'),
            '--variant', 'WF_TRON2A', '--robot-ip', '127.0.0.1' if simulated else endpoint, *flags],
            output='screen'))
    if simulated:
        processes.append(ExecuteProcess(cmd=[sdk_python, str(sdk / 'live_sim.py')], output='screen'))
    extras = []
    if mode != 'hardware':
        rviz = 'wf_live.rviz' if simulated else 'wf_tron2a.rviz'
        panel = 'joint_panel.py' if mode == 'control' else 'wf_panel.py'
        processes.extend([
            Node(package='rviz2', executable='rviz2', namespace='tron2', output='screen',
                 arguments=['-d', str(share / 'rviz' / rviz)]),
            ExecuteProcess(cmd=[sdk_python if mode == 'control' else sys.executable,
                                str(sdk / panel), *(['--sim'] if mode == 'sim' else [])], output='screen')])
        graph = ExecuteProcess(cmd=['ros2', 'run', 'rqt_graph', 'rqt_graph'], output='screen')
        extras.append(TimerAction(period=6.0, actions=[graph]))
    handlers = [RegisterEventHandler(OnProcessExit(target_action=proc,
                on_exit=[EmitEvent(event=Shutdown(reason='TRON2 session closed'))]))
                for proc in processes]
    return handlers + processes + extras


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim', choices=['sim','control','hardware','operator']),
        DeclareLaunchArgument('robot_ip', default_value=''),
        DeclareLaunchArgument('motion', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('sdk_python', default_value=sys.executable),
        OpaqueFunction(function=assemble),
    ])
