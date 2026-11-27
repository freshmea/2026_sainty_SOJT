"""Balance and drive the wheel-foot MuJoCo model with the released ONNX policy."""
import math
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tron2_bringup.bringup import camera_window, default_sdk_root, robot_state_publisher
from tron2_bringup.sim_session import reserve_launch


def policy_exit(event, context):
    if event.returncode and not context.is_shutdown:
        raise RuntimeError(f'WF policy simulation exited with status {event.returncode}')
    return [Shutdown(reason='WF policy simulation ended')]


def start(context):
    root = Path(LaunchConfiguration('sdk_root').perform(context)).expanduser().resolve()
    python = root / '.venv/bin/python'
    if not python.is_file():
        raise RuntimeError(f'Missing SDK environment: {python}. Run uv sync --locked in {root}.')
    folder = root / 'vendor/tron2_rl_deploy_python/controllers/model/WF_TRON2A'
    reference = yaml.safe_load((folder / 'params.yaml').read_text())['PointfootCfg']['commands']['max']
    experiment = LaunchConfiguration('profile').perform(context) == 'experiment'
    limits = {}
    for param, key, trial_default in [('max_linear', 'lin_vel_x', 5.), ('max_yaw', 'ang_vel_yaw', 4.)]:
        value = LaunchConfiguration(param).perform(context)
        if value and not experiment:
            raise ValueError(f'{param} override requires profile:=experiment')
        limit = float(value) if value else (trial_default if experiment else float(reference[key]))
        if not math.isfinite(limit) or limit <= 0:
            raise ValueError(f'{param} must be positive and finite')
        limits[param] = limit
    models = {}
    for name in ('drive_onnx', 'recovery_onnx', 'drive_encoder', 'recovery_encoder'):
        raw = LaunchConfiguration(name).perform(context)
        if raw and not Path(raw).expanduser().is_file():
            raise ValueError(f'{name} does not exist: {raw}')
        models[name.upper()] = str(Path(raw).expanduser().resolve()) if raw else ''
    for role in ('DRIVE', 'RECOVERY'):
        if models[role + '_ENCODER'] and not models[role + '_ONNX']:
            raise ValueError(f'{role.lower()}_encoder requires the matching {role.lower()}_onnx')
    pose = LaunchConfiguration('start_pose').perform(context)
    if pose != 'standing' and not models['RECOVERY_ONNX']:
        raise ValueError('A fallen start requires recovery_onnx')
    reserve_launch(context, 'mujoco')
    viewer = LaunchConfiguration('viewer').perform(context)
    live_rviz = get_package_share_path('tron2_description') / 'launch/live_rviz.launch.py'
    return [
        camera_window(),
        robot_state_publisher(),
        ExecuteProcess(cmd=[str(python), '-m', 'tron2_bringup.policy_drive', '--ros-args',
                            '-r', '__ns:=/tron2', '-p', f'viewer:={viewer}',
                            '-p', f"max_linear:={limits['max_linear']}",
                            '-p', f"max_yaw:={limits['max_yaw']}"],
                       additional_env={'TRON2_SDK_ROOT': str(root), 'WF_START_POSE': pose, **models}, output='screen',
                       on_exit=policy_exit),
        Node(package='tron2_dashboard', executable='velocity_joystick', namespace='tron2',
             condition=IfCondition(LaunchConfiguration('joystick')), parameters=[limits], output='screen',
             on_exit=Shutdown(reason='Joystick closed')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(live_rviz)),
                                 condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sdk_root', default_value=default_sdk_root()),
        DeclareLaunchArgument('profile', default_value='reference', choices=['reference', 'experiment'],
                              description='WF example range or an expanded MuJoCo-only test range.'),
        DeclareLaunchArgument('max_linear', default_value='', description='Experiment stick range in m/s.'),
        DeclareLaunchArgument('max_yaw', default_value='', description='Experiment stick range in rad/s.'),
        DeclareLaunchArgument('drive_onnx', default_value='', description='Optional 40-input WF drive actor.'),
        DeclareLaunchArgument('recovery_onnx', default_value='', description='A classroom 42-input rescue or a continuous 40-input Isaac recovery actor.'),
        DeclareLaunchArgument('drive_encoder', default_value='', description='Drive history encoder. Empty uses the released WF encoder.onnx.'),
        DeclareLaunchArgument('recovery_encoder', default_value='', description='Recovery history encoder. Empty uses the released WF encoder.onnx.'),
        DeclareLaunchArgument('start_pose', default_value='standing', choices=['standing','front','back']),
        DeclareLaunchArgument('viewer', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('joystick', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value=LaunchConfiguration('viewer'), choices=['true', 'false'],
                              description='Open RViz with the WF model and TF. Defaults to the viewer setting.'),
        DeclareLaunchArgument('camera_view', default_value=LaunchConfiguration('viewer'), choices=['true', 'false']),
        OpaqueFunction(function=start),
    ])
