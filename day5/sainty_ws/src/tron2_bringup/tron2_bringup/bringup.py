from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch.actions import ExecuteProcess, Shutdown, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def default_sdk_root():
    return EnvironmentVariable('TRON2_SDK_ROOT', default_value=str(Path.home() / 'sainty/day5/tron2_sdk'))


def robot_state_publisher():
    model = get_package_share_path('tron2_description') / 'urdf/wf_tron2a.urdf'
    return Node(package='robot_state_publisher', executable='robot_state_publisher',
                namespace='tron2', parameters=[{'frame_prefix': 'tron2/', 'publish_frequency': 50.0,
                'robot_description': ParameterValue(model.read_text(), value_type=str)}])


def sdk_bridge(root, python, ip, motion):
    # The fixed SBC bridge accepts argparse flags and owns its existing ROS node.
    flags = ['--enable-motion'] if motion else []
    return ExecuteProcess(cmd=[str(python), str(root / 'sdk_bridge.py'), '--variant', 'WF_TRON2A',
                               '--robot-ip', ip, '--wf-panel', *flags], output='screen',
                          on_exit=Shutdown(reason='SDK bridge exited'))

def control_stack(backend):
    path = get_package_share_path('tron2_control') / 'launch/control.launch.py'
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments={'backend': backend}.items())
