#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export PATH="$HOME/.local/bin:$PATH"
uv run --locked lab02_mujoco_pd.py
uv run --locked lab01_inspect.py
uv run --locked compare_control.py
uv run --locked check_reset.py
