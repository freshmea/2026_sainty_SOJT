#!/usr/bin/env bash
# Run the MicroDock scoot task without patching the upstream checkout.
set -euo pipefail

day4=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
repo=$day4/microduck_rl_training
patch=$day4/microduck-playground/hardware/microdock-v2/training/butt_walk.patch
commit=bae39f5900b318f7e87fc485c30a31213cfa14eb
task=Mjlab-ButtWalk-Flat-MicroDuck

usage() {
    echo "Usage: $0 {list-envs|test|train|play|export} [options]"
    echo "train, play and export select $task automatically. Paths are relative to your current directory."
}

case ${1:-} in
    -h|--help) usage; exit 0 ;;
    list-envs|test|train|play|export) action=$1; shift ;;
    *) usage >&2; exit 2 ;;
esac
if [[ "$(pwd -P)/" == "$repo/"* ]]; then
    echo "Run from outside $repo so logs and checkpoints stay outside the clone." >&2
    exit 2
fi
uv=$(command -v uv || true)
if [[ -z "$uv" ]]; then uv="$HOME/.local/bin/uv"; fi
if [[ ! -x "$uv" ]]; then echo "uv is required." >&2; exit 1; fi

digest=$(sha256sum -- "$patch")
cache=${XDG_CACHE_HOME:-$HOME/.cache}/sainty/butt_walk
project=$cache/$commit-${digest%% *}
mkdir -p -- "$cache"
(
    flock -x 9
    if [[ ! -d "$project" ]]; then
        staging=$(mktemp -d "$cache/.prepare.XXXXXXXX")
        trap 'rm -rf -- "$staging"' EXIT
        git -C "$repo" archive "$commit" | tar -x -C "$staging"
        (cd "$staging" && git apply --no-index -- "$patch")
        mv -T -- "$staging" "$project"
    fi
) 9>"$cache/.prepare.lock"

# Warp in WSL needs the Windows-provided CUDA driver ahead of any toolkit stubs.
if [[ -f /usr/lib/wsl/lib/libcuda.so.1 ]]; then
    export LD_LIBRARY_PATH="/usr/lib/wsl/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
run=("$uv" run --frozen --project "$project")
case "$action" in
    list-envs) exec "${run[@]}" python -m mjlab.scripts.list_envs --keyword ButtWalk "$@" ;;
    test) exec "${run[@]}" --with pytest python -m pytest "$project/tests/test_butt_walk_cfg.py" "$@" ;;
    train|play) exec "${run[@]}" "$action" "$task" "$@" ;;
    export) exec "${run[@]}" python "$project/scripts/export.py" "$task" "$@" ;;
esac
