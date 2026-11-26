#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
day=$PWD
mode=${1:-sim}
shift || true
usage() {
  echo "usage: ./panel_control.sh [sim [mujoco|gz] | control | hardware ROBOT_IP [--enable-motion] | operator | check [WF_TRON2A|SF_TRON2A]]" >&2
  exit 2
}
args=("mode:=$mode")
packages=(tron2_description)
backend=mujoco
entry=("$day/tron2_sdk/live.launch.py")
source /opt/ros/humble/setup.bash
case "$mode" in
  check) exec ./ros_lab/run.sh tron2 "${1:-WF_TRON2A}" ;;
  SF_TRON2A|WF_TRON2A) exec ./ros_lab/run.sh tron2 "$mode" ;;
  sim|control)
    if [[ $mode == sim ]]; then
      [[ $# -le 1 ]] || usage
      backend=${1:-mujoco}
      [[ $backend == gz || $backend == mujoco ]] || usage
      packages=(tron2_dashboard tron2_bringup)
      if [[ $backend == gz ]]; then packages+=(tron2_gz); fi
      entry=("$day/panel_control.launch.py")
      args=("backend:=$backend")
    else
      [[ $# == 0 ]] || usage
      packages=(tron2_bringup)
    fi
    if [[ $backend == mujoco ]]; then
      if [[ ! -f tron2_sdk/upstream/tron2_mujoco_sim/tron2_sim/core.py ]]; then
        git submodule update --init --depth 1 -- tron2_sdk/upstream/tron2_mujoco_sim
      fi
      export PATH="$HOME/.local/bin:$PATH"
      uv sync --project tron2_sdk --locked
      uv run --project tron2_sdk --locked python tron2_sdk/fetch_models.py
    fi ;;
  hardware)
    [[ $# == 1 || ( $# == 2 && $2 == --enable-motion ) ]] || usage
    [[ -n $1 && $1 != -* ]] || usage
    python=${TRON2_SDK_PYTHON:-/usr/bin/python3}
    if ! "$python" -c 'import rclpy; from limxsdk.robot import Robot, RobotType'; then
      echo 'Set TRON2_SDK_PYTHON to a Python 3.10 interpreter with ROS 2 Humble and the LimX SDK for this CPU.' >&2
      exit 1
    fi
    args+=("robot_ip:=$1" "sdk_python:=$python")
    if [[ ${2:-} == --enable-motion ]]; then args+=("motion:=true"); fi ;;
  operator)
    [[ $# == 0 ]] || usage
    packages=(tron2_dashboard)
    entry=(tron2_dashboard dashboard.launch.py)
    args=(target:=hardware) ;;
  *) usage ;;
esac
if [[ ( $mode == hardware || ( $mode == sim && $backend == mujoco ) ) && ! -f tron2_sdk/vendor/tron2_rl_deploy_python/controllers/model/WF_TRON2A/params.yaml ]]; then
  git submodule update --init --depth 1 -- tron2_sdk/vendor/tron2_rl_deploy_python
fi
if [[ $mode == hardware || $mode == operator ]]; then
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-52}
fi
cd sainty_ws
colcon build --symlink-install --packages-up-to "${packages[@]}"
source install/local_setup.bash
exec ros2 launch "${entry[@]}" "${args[@]}"
