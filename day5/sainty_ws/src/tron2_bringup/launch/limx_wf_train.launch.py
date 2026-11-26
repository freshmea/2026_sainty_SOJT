"""Run LimX WF training or play with the selected Isaac Lab Python runtime."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    Shutdown,
)
from launch.substitutions import LaunchConfiguration


def start(context):
    def value(key):
        return LaunchConfiguration(key).perform(context)

    repo = Path(value("repo")).expanduser().resolve()
    course = Path(value("course")).expanduser().resolve()
    stage, task = value("stage"), value("task")
    script = repo / "scripts" / "rsl_rl" / f"{stage}.py"
    if not script.is_file() or not (repo / "rsl_rl/rsl_rl/runner").is_dir():
        raise RuntimeError(f"Expected the LimX scripts and vendored rsl_rl in {repo}")
    checkpoint = (
        Path(value("checkpoint")).expanduser().resolve()
        if value("checkpoint")
        else None
    )
    if checkpoint and not checkpoint.is_file():
        raise ValueError(f"Checkpoint is missing: {checkpoint}")
    if stage == "play" and checkpoint is None:
        raise ValueError(
            "Select the training checkpoint to play with checkpoint:=/path/to/model.pt"
        )
    if not value("isaac_python"):
        raise ValueError(
            "Set isaac_python to the Isaac Lab environment's Python or python.sh"
        )
    cmd = [os.path.expanduser(value("isaac_python"))]
    task_id = "Isaac-Limx-WF-TRON2A-Blind-Flat"
    if task == "reference":
        cmd.append(str(script))
    else:
        cmd.extend(
            [
                str(course / "locomotion/wf_isaac_entry.py"),
                "--repo",
                str(repo),
                "--stage",
                stage,
            ]
        )
        task_id = (
            "Sainty-WF-" + {"high_speed": "HighSpeed", "recovery": "Recovery"}[task]
        )
    cmd.extend(
        [
            "--headless",
            "--task",
            task_id + ("-Play" if stage == "play" else "") + "-v0",
            "--num_envs",
            value("num_envs"),
            "--seed",
            "73",
        ]
    )
    if stage == "train" and value("run_name"):
        cmd.extend(["--run_name", value("run_name")])
    if stage == "train" and value("iterations"):
        cmd.extend(["--max_iterations", value("iterations")])
    if checkpoint:
        if stage == "train":
            cmd.extend(["--resume", "True"])
        cmd.extend(["--checkpoint_path", str(checkpoint)])

    def finished(event, context):
        if event.returncode and not context.is_shutdown:
            raise RuntimeError(
                f"LimX {stage} process exited with status {event.returncode}"
            )
        return [Shutdown(reason="LimX process exited")]

    return [ExecuteProcess(cmd=cmd, cwd=str(repo), output="screen", on_exit=finished)]


def generate_launch_description():
    course = Path.home() / "sainty/day5"
    return LaunchDescription(
        [
            DeclareLaunchArgument("course", default_value=str(course)),
            DeclareLaunchArgument(
                "repo", default_value=str(course / "tron2_sdk/vendor/tron2_rl_lab")
            ),
            DeclareLaunchArgument(
                "isaac_python", default_value=os.environ.get("ISAAC_PYTHON", str(course / "isaac_wsl/run-python"))
            ),
            DeclareLaunchArgument(
                "stage", default_value="train", choices=["train", "play"]
            ),
            DeclareLaunchArgument(
                "task",
                default_value="reference",
                choices=["reference", "high_speed", "recovery"],
            ),
            DeclareLaunchArgument("num_envs", default_value="16"),
            DeclareLaunchArgument("run_name", default_value=""),
            DeclareLaunchArgument(
                "iterations",
                default_value="",
                description="Empty uses the LimX PPO config.",
            ),
            DeclareLaunchArgument(
                "checkpoint",
                default_value="",
                description="WSL path to the .pt checkpoint.",
            ),
            OpaqueFunction(function=start),
        ]
    )
