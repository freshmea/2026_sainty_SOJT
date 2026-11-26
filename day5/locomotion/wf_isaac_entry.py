"""Register classroom WF tasks, then run the unmodified LimX train or play script."""

import argparse
from pathlib import Path
import runpy
import sys

import gymnasium as gym


def main():
    parser = argparse.ArgumentParser(description=__doc__, add_help=False)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--stage", choices=("train", "play"), required=True)
    args, forwarded = parser.parse_known_args()
    for variant in ("HighSpeed", "Recovery"):
        for play in (True, False):
            name = variant + ("Play" if play else "")
            task_id = f"Sainty-WF-{variant}" + ("-Play" if play else "") + "-v0"
            gym.register(
                id=task_id,
                entry_point="isaaclab.envs:ManagerBasedRLEnv",
                disable_env_checker=True,
                kwargs={
                    "env_cfg_entry_point": f"wf_isaac_tasks:{name}",
                    "rsl_rl_cfg_entry_point": "bipedal_locomotion.tasks.locomotion.agents.limx_rsl_rl_ppo_cfg:WF_TRON2AFlatPPORunnerCfg",
                },
            )
    script = args.repo.resolve() / "scripts" / "rsl_rl" / f"{args.stage}.py"
    sys.path.insert(0, str(script.parent))
    sys.argv = [str(script), *forwarded]
    runpy.run_path(str(script), run_name="__main__")


if __name__ == "__main__":
    main()
