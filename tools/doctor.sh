#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
echo "ROS: $ROS_DISTRO / $(uname -m)"
for p in verter_drivers verter_voice verter_web verter_description verter_bringup verter_slam nav2_bringup slam_toolbox micro_ros_agent; do
    ros2 pkg prefix "$p" || true
done
for dev in /dev/chassis /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 /dev/esp32_imu /dev/esp32_ctrl; do ls -l "$dev" || true; done
python3 -c 'import serial,modbus_tk,flask,flask_restx; print("Core Python imports OK")'
