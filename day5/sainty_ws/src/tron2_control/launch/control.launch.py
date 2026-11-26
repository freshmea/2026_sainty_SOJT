import xacro
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    backend = LaunchConfiguration('backend').perform(context)
    device = {'gz': 'GazeboSystem', 'mujoco': 'MujocoSdkSystem', 'hardware': 'LimxSdkSystem'}[backend]
    config = get_package_share_path('tron2_control') / 'config'
    model = xacro.process_file(str(config / 'wf_control.urdf.xacro'), mappings={'device': device}).toxml()
    description = {'robot_description': ParameterValue(model, value_type=str)}
    actions = [
        Node(package='controller_manager', executable='ros2_control_node', namespace='tron2',
             parameters=[description, str(config / 'controllers.yaml')], output='screen',
             on_exit=Shutdown(reason='Controller manager exited')),
        Node(package='controller_manager', executable='spawner', namespace='tron2',
             arguments=['joint_state_broadcaster', 'imu_sensor_broadcaster', 'wf_joint_controller',
                        '-c', '/tron2/controller_manager', '-p', str(config / 'controllers.yaml')],
             output='screen'),
    ]
    if backend != 'hardware':
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                            namespace='tron2', parameters=[description, {'frame_prefix': 'tron2/',
                            'publish_frequency': 50.0}],
                            remappings=[('joint_states', 'joint_state_broadcaster/joint_states')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='hardware', choices=['gz','mujoco','hardware']),
        OpaqueFunction(function=start),
    ])
