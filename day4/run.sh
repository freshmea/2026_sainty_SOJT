#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export PATH="$HOME/.local/bin:$PATH"
export MICRODUCK_RL_DIR="$PWD/microduck_rl" MICRODUCK_POLICY_DIR="$PWD/policies"
sim=microduck-playground/hardware/microdock-v2/sim
mkdir -p results
case "${1:-showcase}" in
  showcase)
    uv run --locked python "$sim/docking_seated.py" --n 1 --seed 0 --noise 0 --start 450,15,178 --hold 4 --rest limp --doze_pause 0.1 --handover_s 0 --sit_until_still --save results/run.pkl --out results/showcase.json
    uv run --locked python "$sim/undocking_seated.py" --continue results/run.pkl --save results/full.pkl | tee results/undocking.log
    ;;
  compare)
    uv run --locked python "$sim/undocking_seated.py" --seeds 41 --n 3 --mag 4 --out results/latency_on.json
    uv run --locked python latency_ablation.py --workdir "$PWD" --seed 41 --n 3 --out "$PWD/results/latency_off.json"
    ;;
  render) MUJOCO_GL=egl uv run --locked python render_classroom.py --workdir "$PWD" ;;
  *) echo '사용법: ./run.sh showcase | compare | render' >&2; exit 2 ;;
esac
