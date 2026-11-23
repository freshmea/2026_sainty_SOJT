"""Read contiguous phases from a trajectory produced by this lab's showcase."""
import argparse
import itertools
from pathlib import Path
import pickle

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trajectory", type=Path, nargs="?", default=Path("results/full.pkl"))
    args = parser.parse_args()
    with args.trajectory.open("rb") as stream:
        trajectory = pickle.load(stream)
    qpos, phases = trajectory["qpos"], trajectory["log"]["phase"]
    assert len(qpos) == len(phases) and np.isfinite(qpos).all()
    print("phase        start_frame   frames    first_z_mm   last_z_mm")
    start = 0
    for phase, group in itertools.groupby(phases):
        count = sum(1 for _ in group)
        end = start + count
        print(f"{phase:12} {start:11d} {count:8d} {1000*qpos[start, 2]:13.1f} {1000*qpos[end-1, 2]:11.1f}")
        start = end


if __name__ == "__main__":
    main()
