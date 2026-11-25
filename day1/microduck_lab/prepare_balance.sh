#!/usr/bin/env bash
# Day 1 전신 실습은 공식 저장소의 별도 Python 3.12 환경을 사용한다.
set -euo pipefail
lab=$(cd "$(dirname "$0")" && pwd)
repo="$lab/../../day4/microduck_rl_training"
commit=bae39f5900b318f7e87fc485c30a31213cfa14eb
if [[ ! -d "$repo" ]]; then
  mkdir -p "$(dirname "$repo")"
  git clone https://github.com/pollen-robotics/microduck_rl.git "$repo"
  git -C "$repo" checkout --detach "$commit"
fi
if [[ "$(git -C "$repo" rev-parse HEAD)" != "$commit" ]]; then
  echo "기존 체크아웃을 보존합니다. 교재 기준 커밋: $commit" >&2
  exit 1
fi
(cd "$repo" && uv sync --locked)
dest="$lab/results/head_balance"
mkdir -p "$dest"
sha=702c01bd02aeda1d7869390342e00298193e7071d475f3776e0a6eedfc5c8209
url=https://huggingface.co/pollen-robotics/microduck-training/resolve/64d28ce092253c509b51031ec7ccb53790a00d0d/bundles/velocity2/441tzs6d_3750/model_3750.pt
if [[ ! -f "$dest/warmstart.pt" ]]; then
  tmp=$(mktemp "$dest/.warmstart.XXXXXX")
  trap 'rm -f "$tmp"' EXIT
  curl -fL --retry 2 "$url" -o "$tmp"
  echo "$sha  $tmp" | sha256sum -c -
  mv "$tmp" "$dest/warmstart.pt"
fi
echo "$sha  $dest/warmstart.pt" | sha256sum -c -
