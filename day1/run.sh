#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export PATH="$HOME/.local/bin:$PATH"
mkdir -p results
(cd mujoco_beginner && uv run --locked 00_first_sim.py && uv run --locked 02_inspect_model.py && uv run --locked 05_contacts.py) | tee results/models_and_contacts.txt
(cd mjcf_bridge && uv run --locked verify_bridge.py)
(cd microduck_lab && uv run --locked inspect_model.py) | tee results/microduck_joints.txt
(cd K1 && uv run --locked verify_policy.py --no-render)
if [[ ! -e results/k1_validation.json ]]; then ln -s ../K1/validation/validation.json results/k1_validation.json; fi

../day5/ros_lab/run.sh k1
../day5/ros_lab/run.sh tron2
