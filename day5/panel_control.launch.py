"""One-terminal shortcut for the two classroom launch files."""
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package, filename, **arguments):
    path = get_package_share_path(package) / 'launch' / filename
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments=arguments.items())


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='mujoco', choices=['mujoco', 'gz']),
        DeclareLaunchArgument('camera_view', default_value='true', choices=['true', 'false']),
        include('tron2_bringup', 'simulation_bringup.launch.py', backend=LaunchConfiguration('backend'),
                camera_view=LaunchConfiguration('camera_view')),
        include('tron2_dashboard', 'dashboard.launch.py', target='simulation'),
    ])
