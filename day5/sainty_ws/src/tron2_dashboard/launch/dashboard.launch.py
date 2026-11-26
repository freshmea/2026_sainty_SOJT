from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def dashboard(context):
    simulation = LaunchConfiguration('target').perform(context) == 'simulation'
    config = 'wf_live.rviz' if simulation else 'wf_tron2a.rviz'
    rviz = get_package_share_path('tron2_description') / 'rviz' / config
    control = get_package_share_path('tron2_control') / 'launch/control.launch.py'
    actions = []
    if not simulation:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(control)),
                       launch_arguments={'backend': 'hardware'}.items(),
                       condition=IfCondition(LaunchConfiguration('start_hardware_control'))))
    return actions + [
        Node(package='tron2_dashboard', executable='joint_panel', namespace='tron2',
             parameters=[{'simulation': simulation}], output='screen',
             on_exit=Shutdown(reason='Operator panel closed')),
        Node(package='rviz2', executable='rviz2', namespace='tron2',
             arguments=['-d', str(rviz)], condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='rqt_graph', executable='rqt_graph',
             condition=IfCondition(LaunchConfiguration('rqt'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target', default_value='hardware', choices=['hardware', 'simulation'],
                              description='Select hold-to-apply or immediate simulation controls.'),
        DeclareLaunchArgument('start_hardware_control', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rqt', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=dashboard),
    ])
