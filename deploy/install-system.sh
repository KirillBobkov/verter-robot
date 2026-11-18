#!/usr/bin/env bash
set -euo pipefail
[[ "$(id -u)" != 0 ]] || {
    echo 'Run as the target robot user, not root'
    exit 1
}
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] || {
    echo 'Requires Ubuntu 22.04 / ROS Humble'
    exit 1
}
# Jetson must first be flashed with a compatible JetPack/L4T by NVIDIA tooling.
sudo apt-get update
sudo apt-get install -y curl gnupg software-properties-common locales
sudo add-apt-repository -y universe
sudo locale-gen en_US.UTF-8
curl --fail --location https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/verter-ros.key
sudo install -m 0644 /tmp/verter-ros.key /usr/share/keyrings/ros-archive-keyring.gpg
printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://packages.ros.org/ros2/ubuntu jammy main\n' "$(dpkg --print-architecture)" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
sudo apt-get install -y build-essential cmake git rsync psmisc python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool python3-pytest python3-yaml python3-pil python3-numpy python3-opencv python3-serial ros-humble-ros-base ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-robot-localization ros-humble-laser-filters libpcap-dev libpcl-dev libboost-thread-dev ros-humble-pcl-conversions ros-humble-diagnostic-updater ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-rosbridge-server ros-humble-twist-mux libportaudio2 libusb-1.0-0
[[ -e /etc/ros/rosdep/sources.list.d/20-default.list ]] || sudo rosdep init
rosdep update --rosdistro humble
sudo usermod -aG dialout,audio,video "$USER"
echo 'System dependencies installed. Re-login for group membership. Nothing is started.'
