"""Reserve one local TRON2 physics engine across ROS domains and terminals."""
import fcntl
import os
from pathlib import Path


def reserve(backend, *, child=False):
    folder = Path.home() / '.cache' / 'tron2'
    folder.mkdir(mode=0o700, parents=True, exist_ok=True)
    handle = (folder / 'simulation.lock').open('a+')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.seek(0)
        owner = handle.read().strip()
        handle.close()
        # The installed MuJoCo executable is also runnable directly with ros2 run.
        if child and owner == f'{backend} {os.getppid()}':
            return None
        raise RuntimeError(f'TRON2 simulation already running ({owner}). '
                           'Stop the active bringup before selecting another backend.') from None
    handle.seek(0)
    handle.truncate()
    handle.write(f'{backend} {os.getpid()}')
    handle.flush()
    return handle


def reserve_launch(context, backend):
    # Keep the descriptor through child shutdown. Linux releases it when launch exits.
    context.tron2_simulation_lock = reserve(backend)
