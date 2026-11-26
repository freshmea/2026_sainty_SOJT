import os
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tron2_bringup.bringup import control_stack
from tron2_bringup.sim_session import reserve_launch


def start(context):
    reserve_launch(context, 'gz')
    share = get_package_share_path('tron2_gz')
    model = xacro.process_file(str(share / 'urdf/wf_gz.urdf.xacro'))
    # Only draw the decimated CAD. DART keeps the source model's collision geometry.
    for visual in model.getElementsByTagName('visual'):
        for mesh in visual.getElementsByTagName('mesh'):
            mesh.setAttribute('filename', mesh.getAttribute('filename').replace(
                'package://tron2_description/meshes/', 'package://tron2_gz/meshes/'))
    resources = os.pathsep.join([str(get_package_share_path('tron2_description').parent), str(share.parent)])
    plugins = str(Path(get_package_prefix('tron2_gz')) / 'lib')
    env = {}
    for key, value in [('IGN_GAZEBO_RESOURCE_PATH', resources), ('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', plugins),
                       ('IGN_GUI_PLUGIN_PATH', plugins)]:
        env[key] = os.pathsep.join(filter(None, [value, os.environ.get(key)]))
    adapter = LaunchConfiguration('gpu_adapter').perform(context)
    if adapter and Path('/dev/dxg').exists():
        env['MESA_D3D12_DEFAULT_ADAPTER_NAME'] = adapter
    flags = ['--render-engine-gui', 'ogre'] if LaunchConfiguration('gui').perform(context) == 'true' else ['-s']
    return [
        control_stack('gz'),
        ExecuteProcess(cmd=['ign', 'gazebo', '-r', *flags, str(share / 'worlds/classroom.sdf')],
                       additional_env=env, output='screen', on_exit=Shutdown(reason='Gazebo closed')),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', 'classroom', '-name', 'tron2', '-z', '0.755', '-string', model.toxml()],
             output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gpu_adapter', default_value=os.environ.get(
            'MESA_D3D12_DEFAULT_ADAPTER_NAME', 'NVIDIA' if Path('/dev/dxg').exists() else ''),
            description='WSLg D3D12 adapter substring, e.g. NVIDIA or AMD.'),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=start),
    ])
