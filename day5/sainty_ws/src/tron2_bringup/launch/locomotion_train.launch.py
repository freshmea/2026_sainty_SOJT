"""Start one headless WF PPO training job in the locked locomotion environment."""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    Shutdown,
)
from launch.substitutions import LaunchConfiguration


def train(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    root = Path(value("root")).expanduser().resolve()
    python = root / ".venv/bin/python"
    if not python.exists():
        raise RuntimeError(
            f"Prepare the training environment with uv sync --locked in {root}"
        )
    args = [
        str(python),
        str(root / "wf_task_train.py"),
        "--task",
        value("task"),
        "--out",
        value("out"),
        "--steps",
        value("steps"),
        "--target",
        value("target"),
        "--crouch",
        value("crouch"),
        "--seed",
        value("seed"),
    ]
    if value("resume"):
        args.extend(["--resume", value("resume")])

    def finished(event, context):
        if event.returncode:
            raise RuntimeError(f"PPO training exited with status {event.returncode}")
        return [Shutdown(reason="Training and ONNX export complete")]

    return [ExecuteProcess(cmd=args, output="screen", on_exit=finished)]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "root", default_value=str(Path.home() / "sainty/day5/locomotion")
            ),
            DeclareLaunchArgument("task", choices=["recovery", "high_speed"]),
            DeclareLaunchArgument(
                "out", description="New output directory for this run."
            ),
            DeclareLaunchArgument("steps", default_value="32768"),
            DeclareLaunchArgument("target", default_value="5.0"),
            DeclareLaunchArgument("crouch", default_value="0.35"),
            DeclareLaunchArgument("seed", default_value="73"),
            DeclareLaunchArgument("resume", default_value=""),
            OpaqueFunction(function=train),
        ]
    )
