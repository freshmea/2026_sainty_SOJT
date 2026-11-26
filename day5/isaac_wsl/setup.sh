#!/bin/bash
# Install the Linux runtime next to this script. Run in Ubuntu 22.04 WSL2.
set -euo pipefail
root=$(cd "$(dirname "$0")" && pwd)
repo="$root/../tron2_sdk/vendor/tron2_rl_lab"
uv=$(command -v uv || printf '%s' "$HOME/.local/bin/uv")
if [ ! -d "$root/IsaacLab" ]; then
    git clone --depth 1 --branch v2.3.1 https://github.com/isaac-sim/IsaacLab.git "$root/IsaacLab"
fi
if [ ! -x "$root/env/bin/python" ]; then "$uv" venv --python 3.11 "$root/env"; fi
py="$root/env/bin/python"
"$uv" pip install --python "$py" --index https://pypi.nvidia.com --default-index https://pypi.org/simple \
    --index-strategy unsafe-best-match 'isaacsim[rl,extscache]==5.1.0.0'
"$uv" pip install --python "$py" 'setuptools<79' wheel toml GitPython onnx==1.18.0 tensordict==0.8.3 \
    tensorboard==2.19.0 hydra-core==1.3.2 gymnasium==1.2.0 prettytable==3.3.0 hidapi==0.14.0.post2 \
    einops 'pyglet<2' h5py moviepy==1.0.3 'protobuf>=4.25.8,<5' 'packaging<24' torch==2.7.0
"$uv" pip install --python "$py" --no-deps --no-build-isolation flatdict==4.0.1 \
    -e "$root/IsaacLab/source/isaaclab" -e "$root/IsaacLab/source/isaaclab_assets" \
    -e "$root/IsaacLab/source/isaaclab_tasks" -e "$root/IsaacLab/source/isaaclab_rl" \
    -e "$repo/rsl_rl" -e "$repo/exts/bipedal_locomotion"
if [ -f /usr/lib/wsl/lib/libcuda.so.1 ]; then
    mkdir -p "$root/driver"
    for library in libcuda libnvidia-ml; do
        for extension in so so.1; do
            ln -sfn "/usr/lib/wsl/lib/$library.so.1" "$root/driver/$library.$extension"
        done
    done
fi
