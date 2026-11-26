import os
from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tron2_bringup.sim_session import reserve_launch
from tron2_bringup.bringup import camera_window, default_sdk_root, control_stack, sdk_bridge


def backend(context):
    if LaunchConfiguration('backend').perform(context) == 'gz':
        gz = get_package_share_path('tron2_gz') / 'launch/gz.launch.py'
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz)),
                    launch_arguments={'gui': LaunchConfiguration('gz_gui'),
                                      'camera_view': LaunchConfiguration('camera_view'),
                                      'gpu_adapter': LaunchConfiguration('gpu_adapter')}.items())]
    reserve_launch(context, 'mujoco')
    root = Path(LaunchConfiguration('sdk_root').perform(context)).expanduser().resolve()
    python = root / '.venv/bin/python'
    return [
        camera_window(),
        control_stack('mujoco'),
        Node(package='tron2_bringup', executable='mujoco_sim', namespace='tron2',
             additional_env={'TRON2_SDK_ROOT': str(root)}, output='screen',
             on_exit=Shutdown(reason='MuJoCo viewer closed')),
        sdk_bridge(root, python, '127.0.0.1', motion=True),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='mujoco', choices=['mujoco', 'gz']),
        DeclareLaunchArgument('gpu_adapter', default_value=os.environ.get(
            'MESA_D3D12_DEFAULT_ADAPTER_NAME', 'NVIDIA' if Path('/dev/dxg').exists() else '')),
        DeclareLaunchArgument('gz_gui', default_value='true', choices=['true', 'false'],
                              description='Open the Gazebo viewer when using backend:=gz.'),
        DeclareLaunchArgument('sdk_root', default_value=default_sdk_root(),
                              description='Prepared LimX SDK workspace for the MuJoCo backend.'),
        DeclareLaunchArgument('camera_view', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=backend),
    ])
