#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
WS="$ROOT/vendor/microros_ws"
mkdir -p "$WS/src"
if [[ ! -d "$WS/src/micro_ros_setup/.git" ]]; then
    git clone https://github.com/micro-ROS/micro_ros_setup.git "$WS/src/micro_ros_setup"
fi
git -C "$WS/src/micro_ros_setup" checkout f8697a3262b0c42caf29052de1adff1561412827
cd "$WS"
rosdep install --from-paths src --ignore-src -y --rosdistro humble
colcon build --packages-select micro_ros_setup
source install/local_setup.bash
if [[ ! -d src/uros/micro-ROS-Agent ]]; then ros2 run micro_ros_setup create_agent_ws.sh; fi
git -C src/uros/micro-ROS-Agent checkout 4f363a79ae96aed39dcc5269f53f73f6a70d3a2c
git -C src/uros/micro_ros_msgs checkout 100bf269e78da3fe0a58f6531ebe47d6991bd9ab
rosdep install --from-paths src --ignore-src -y --rosdistro humble
ros2 run micro_ros_setup build_agent.sh
