#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export PATH="$HOME/.local/bin:$PATH"
case "${1:-help}" in
  train) shift; uv run --project .. --locked python lab.py train "$@" ;;
  official) shift; uv run --project ../../day1/K1 --locked python lab.py evaluate official "$@" ;;
  pd|ppo) uv run --project .. --locked python lab.py evaluate "$@" ;;
  view) shift; project=..
        if [[ "${1:-}" == official ]]; then project=../../day1/K1; fi
        uv run --project "$project" --locked python view.py "$@" ;;
  video) shift; export MUJOCO_GL="${MUJOCO_GL:-egl}"; uv run --project .. --locked python render.py "$@" ;;
  *) echo 'bash run.sh {official|pd|train|ppo|view|video} [options]' ;;
esac
