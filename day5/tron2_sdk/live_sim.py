"""Compatibility entry point for the original MuJoCo launch."""
import os
from pathlib import Path

if __name__ == '__main__':
    os.environ.setdefault('TRON2_SDK_ROOT', str(Path(__file__).resolve().parent))
    from tron2_bringup.mujoco_sim import main
    main()
