#!/usr/bin/env python3
"""Mapping launch — teleop + SLAM Toolbox (no Nav2).

Stack:
  micro_ros_agent (chassis)  — serial ↔ ESP32 chassis (wheel_encoders, cmd_vel)
  micro_ros_agent (imu)      — serial ↔ ESP32 IMU (imu/data)
  twist_mux                  — arbitrate cmd_vel sources by priority
  odometry_node              — wheel encoders → /odom
  ekf_node                   — /odom + /imu/data → /odometry/filtered + TF
  robot_state_publisher      — URDF → /tf_static
  rplidar_node               — Express mode (~10 Hz)
  laser_filter               — strip back 180° + range limits → /scan
  slam_toolbox               — async online mapping → /map

Usage:
  ros2 launch verter_admin mapping.launch.py
  ros2 launch verter_admin mapping.launch.py esp32_port:=/dev/ttyUSB0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from verter_admin.contracts.motion import TopicContract
from verter_admin.control.infrastructure import build_twist_mux_parameters


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory('verter_admin')

    urdf_file          = os.path.join(pkg, 'urdf', 'verter_robot_minimal.urdf')
    slam_params_file   = os.path.join(pkg, 'config', 'slam', 'slam_toolbox_params.yaml')
    ekf_config         = os.path.join(pkg, 'config', 'robot_localization', 'ekf.yaml')
    laser_filter_cfg   = os.path.join(pkg, 'config', 'laser_filters', 'laser_filter.yaml')

    with open(urdf_file) as f:
        robot_description = f.read()

    # ---- Launch arguments ------------------------------------------------------

    esp32_port_arg = DeclareLaunchArgument(
        'esp32_port', default_value='/dev/esp32_chassis',
        description='Serial port for ESP32 chassis micro-ROS agent',
    )
    imu_port_arg = DeclareLaunchArgument(
        'imu_esp32_port', default_value='/dev/esp32_imu',
        description='Serial port for ESP32 IMU micro-ROS agent',
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/rplidar',
        description='Serial port for RPLiDAR',
    )
    agent_extra_arg = DeclareLaunchArgument(
        'micro_ros_agent_extra_args', default_value='',
        description='Extra args for micro_ros_agent (e.g. -v6 for verbose)',
    )

    # ---- micro-ROS agents (auto-restart loops) ---------------------------------

    def micro_ros_agent(name: str, port_cfg: str) -> ExecuteProcess:
        return ExecuteProcess(
            cmd=[
                'bash', '-c',
                [
                    'trap "kill 0; exit" TERM INT; '
                    'while true; do '
                    'source ~/microros_ws/install/setup.bash && '
                    'ros2 run micro_ros_agent micro_ros_agent serial '
                    '--dev ', LaunchConfiguration(port_cfg),
                    ' -b 921600 ',
                    LaunchConfiguration('micro_ros_agent_extra_args'),
                    '; '
                    f'echo "[micro_ros_agent {name}] exited, restarting in 2 s..."; '
                    'sleep 2; done',
                ],
            ],
            output='screen',
            sigterm_timeout='5',
            sigkill_timeout='10',
        )

    micro_ros_chassis = micro_ros_agent('chassis', 'esp32_port')
    micro_ros_imu     = micro_ros_agent('imu',     'imu_esp32_port')

    # ---- Core nodes ------------------------------------------------------------

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[build_twist_mux_parameters()],
        remappings=[('cmd_vel_out', TopicContract.FINAL_CMD_VEL)],
        output='screen',
    )

    odometry_node = Node(
        package='verter_admin',
        executable='odometry_node',
        name='odometry_node',
        parameters=[{'publish_tf': False}],   # EKF publishes odom→base_footprint TF
        output='screen',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        output='screen',
    )

    # ---- LiDAR (delay 3 s to let serial devices settle) -----------------------

    # C++ rplidar_ros with patched SDK: BOTHER→B115200 fix for tegra-xusb+cp210x,
    # VMIN=1 + blocking reads instead of select() for Jetson USB-serial quirk.
    rplidar = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rplidar_ros',
                executable='rplidar_node',
                name='rplidar_node',
                parameters=[{
                    'serial_port':      LaunchConfiguration('lidar_port'),
                    'frame_id':         'lidar_link',
                    'scan_mode':        'Express',
                    'serial_baudrate':  115200,
                    'inverted':         False,
                    'angle_compensate': True,
                }],
                remappings=[('scan', '/scan_raw')],
                respawn=True,
                respawn_delay=5.0,
                output='screen',
            ),
        ],
    )

    laser_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        parameters=[laser_filter_cfg],
        remappings=[('scan', '/scan_raw'), ('scan_filtered', '/scan')],
        output='screen',
    )

    # ---- SLAM Toolbox ----------------------------------------------------------

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params_file, {'use_sim_time': False}],
        output='screen',
    )

    # ---- Launch description ----------------------------------------------------

    return LaunchDescription([
        esp32_port_arg,
        imu_port_arg,
        lidar_port_arg,
        agent_extra_arg,
        micro_ros_chassis,
        micro_ros_imu,
        twist_mux,
        odometry_node,
        ekf_node,
        robot_state_publisher,
        rplidar,
        laser_filter,
        slam_toolbox,
    ])
