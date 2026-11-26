from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_path('tron2_description')
    model = ElementTree.parse(share / 'urdf' / 'wf_tron2a.urdf').getroot()
    # Display the WF model at its zero-angle reference pose without encoder input.
    for joint in model.findall("joint[@type='revolute']"):
        joint.set('type', 'fixed')
    description = ElementTree.tostring(model, encoding='unicode')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='tron2',
            parameters=[{
                'robot_description': ParameterValue(description, value_type=str),
                'frame_prefix': 'tron2/',
            }],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            namespace='tron2',
            arguments=['-d', str(share / 'rviz' / 'wf_tron2a.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
