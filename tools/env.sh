#!/usr/bin/env bash
# Source in bash; do not source the legacy project overlay.
VERTER_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export VERTER_ROOT
export PYTHONNOUSERSITE=1
export PLATFORMIO_CORE_DIR="$VERTER_ROOT/vendor/platformio"
export ROS_LOG_DIR="$VERTER_ROOT/run/ros-logs"
export XDG_CACHE_HOME="$VERTER_ROOT/data/cache"
export TORCH_HOME="$VERTER_ROOT/data/cache/torch"
export HF_HOME="$VERTER_ROOT/data/cache/huggingface"
mkdir -p "$ROS_LOG_DIR" "$XDG_CACHE_HOME"
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
# Drop inherited workspace overlays; vendor dependencies are sourced explicitly below.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
# Rebuild Python/library paths from the selected underlays, not from .bashrc overlays.
unset PYTHONPATH LD_LIBRARY_PATH
export PYTHONPATH="$VERTER_ROOT/.python/site${PYTHONPATH:+:$PYTHONPATH}"
source /opt/ros/humble/setup.bash
if [[ -f "$VERTER_ROOT/vendor/microros_ws/install/local_setup.bash" ]]; then
    source "$VERTER_ROOT/vendor/microros_ws/install/local_setup.bash"
fi
[[ ! -f "$VERTER_ROOT/ros_ws/install/local_setup.bash" ]] || source "$VERTER_ROOT/ros_ws/install/local_setup.bash"
export VERTER_DATA_DIR="${VERTER_DATA_DIR:-$VERTER_ROOT/data}"
export VERTER_ASSETS_DIR="${VERTER_ASSETS_DIR:-$VERTER_ROOT/assets}"
export VERTER_WEB_DIR="${VERTER_WEB_DIR:-$VERTER_ROOT/web/frontend/dist}"
export VERTER_RUN_DIR="${VERTER_RUN_DIR:-$VERTER_ROOT/run}"
if [[ -f "$VERTER_ROOT/config/secrets.env" ]]; then
    set -a
    source "$VERTER_ROOT/config/secrets.env"
    set +a
fi
