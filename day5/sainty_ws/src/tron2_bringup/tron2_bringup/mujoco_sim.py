"""Run the ROS executable using the pinned native MuJoCo and SDK environment."""
import os
from pathlib import Path
import sys


def main():
    sdk = Path(os.environ.get('TRON2_SDK_ROOT', Path.home() / 'sainty/day5/tron2_sdk')).expanduser().resolve()
    venv = sdk / '.venv'
    python = venv / 'bin/python'
    if not python.is_file():
        raise SystemExit(f'Missing simulator environment: {python}. Prepare the MuJoCo SDK workspace first.')
    if Path(sys.prefix).resolve() != venv:
        os.execv(python, [str(python), '-m', 'tron2_bringup.mujoco_sim', *sys.argv[1:]])
    from .sim_session import reserve
    lease = reserve('mujoco', child=True)
    from .mujoco_viewer import main as run_viewer
    try:
        run_viewer()
    finally:
        if lease is not None:
            lease.close()


if __name__ == '__main__':
    main()
