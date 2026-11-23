#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ "$(uname -s)" != "Linux" || "$(uname -r)" != *WSL2* ]]; then
  echo "Run this script inside Ubuntu on WSL2." >&2
  exit 1
fi
if [[ -d .venv/Scripts ]]; then
  echo "This is a Windows venv. Copy only source files to a new Linux folder first." >&2
  exit 1
fi
if ! command -v uv >/dev/null 2>&1 && [[ -x "$HOME/.local/bin/uv" ]]; then
  export PATH="$HOME/.local/bin:$PATH"
fi
if ! command -v uv >/dev/null 2>&1; then
  echo "uv is missing. Install it inside Ubuntu, then rerun this script:" >&2
  echo "  curl -LsSf https://astral.sh/uv/install.sh | sh" >&2
  echo '  export PATH="$HOME/.local/bin:$PATH"' >&2
  exit 1
fi
if [[ ! -f pyproject.toml || ! -f uv.lock ]]; then
  echo "Copy the complete lab project, including pyproject.toml, uv.lock and .python-version." >&2
  exit 1
fi
uv sync --locked
uv run --locked check_wsl.py
uv run --locked 00_first_sim.py
echo "Ready. Open VS Code: code ."
echo "Next: uv run 01_viewer.py"
echo "All checks: uv run run_checks.py --rl --render"
