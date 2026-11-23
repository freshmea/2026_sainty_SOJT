#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export PATH="$HOME/.local/bin:$PATH"
case "${1:-smoke}" in
  dance1) shift; exec bash dance1/run.sh "$@" ;;
  smoke) uv run --locked lab03_ppo.py smoke ;;
  train) uv run --locked lab03_ppo.py train --seed "${2:-11}" --steps 65536 ;;
  evaluate) uv run --locked lab03_ppo.py evaluate --seed "${2:-11}" ;;
  *) echo '사용법: ./run.sh smoke | train [seed] | evaluate [seed] | dance1 [command]' >&2; exit 2 ;;
esac
