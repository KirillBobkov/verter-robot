#!/bin/bash
# Setup patched rplidar_ros for Jetson Orin / tegra-xusb + cp210x
#
# Problem: cp210x USB-UART driver on Jetson has two bugs:
#   1. poll()/select() never fires POLLIN → rplidar_ros hangs waiting for data
#   2. BOTHER termios2 mode sets wrong baud rate → 0 Hz communication
#   3. ioctl(FIONREAD) always returns 0 → SDK reports OPERATION_TIMEOUT
#   4. O_NDELAY + fcntl(F_SETFL) to clear O_NONBLOCK fails silently
#
# Fix: patch SDK to use B115200 constant, VMIN=1 blocking reads,
#      bypass select()/FIONREAD, send STOP before starting rx thread.
#
# Usage:
#   cd ~/ros2_ws
#   bash /path/to/verter-robot/scripts/setup_rplidar_ros.sh
#   colcon build --packages-select rplidar_ros

set -e

PATCH_FILE="$(dirname "$(realpath "$0")")/../docs/patches/rplidar_ros_jetson_cp210x.patch"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
RPLIDAR_SRC="$ROS2_WS/src/rplidar_ros"

echo "[setup_rplidar_ros] ROS2 workspace: $ROS2_WS"
echo "[setup_rplidar_ros] Patch file:     $PATCH_FILE"

# --- Clone if not present ---
if [ ! -d "$RPLIDAR_SRC" ]; then
    echo "[setup_rplidar_ros] Cloning rplidar_ros (ros2 branch)..."
    git clone https://github.com/Slamtec/rplidar_ros.git \
        -b ros2 --depth 1 "$RPLIDAR_SRC"
else
    echo "[setup_rplidar_ros] rplidar_ros already present at $RPLIDAR_SRC"
    # Reset to clean state before applying patch
    cd "$RPLIDAR_SRC"
    if ! git diff --quiet; then
        echo "[setup_rplidar_ros] Resetting uncommitted changes..."
        git checkout .
    fi
fi

# --- Apply patch ---
echo "[setup_rplidar_ros] Applying Jetson cp210x patch..."
cd "$RPLIDAR_SRC"
git apply --whitespace=nowarn "$PATCH_FILE"
echo "[setup_rplidar_ros] Patch applied successfully."

# --- Build ---
echo "[setup_rplidar_ros] Building..."
cd "$ROS2_WS"
source /opt/ros/humble/setup.bash
colcon build --packages-select rplidar_ros \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "[setup_rplidar_ros] Done. Test with:"
echo "  source $ROS2_WS/install/setup.bash"
echo "  ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/rplidar -p scan_mode:=Express -p frame_id:=lidar_link"
