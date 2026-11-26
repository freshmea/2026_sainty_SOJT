"""Compatibility entry point for the former Python panel."""
import os
import sys

if __name__ == '__main__':
    mode = 'true' if '--sim' in sys.argv else 'false'
    os.execlp('ros2', 'ros2', 'run', 'tron2_dashboard', 'joint_panel',
              '--ros-args', '-p', 'simulation:=' + mode)
