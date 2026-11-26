from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    model_dir = get_package_share_path('turtlebot3_description') / 'urdf'
    robot = xacro.process_file(str(model_dir / 'turtlebot3_burger.urdf'))

    # A static display has no wheel encoder updates. Keep both wheels at zero.
    for joint in robot.documentElement.getElementsByTagName('joint'):
        if joint.getAttribute('name') in ('wheel_left_joint', 'wheel_right_joint'):
            joint.setAttribute('type', 'fixed')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot.toxml()}],
        ),
    ])
