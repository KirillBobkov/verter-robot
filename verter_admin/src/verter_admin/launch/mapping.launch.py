#!/usr/bin/env python3
"""
Launch файл для картографирования на реальном роботе Verter.

Компоненты:
1. micro_ros_agent (chassis) - связь с ESP32 шасси (micro-ROS)
2. micro_ros_agent (imu) - связь с ESP32 IMU (micro-ROS)
3. twist_mux - мультиплексор команд скорости
4. odometry_node - одометрия (энкодеры)
5. robot_state_publisher - публикация TF из URDF
6. rplidar_node - драйвер лидара
7. laser_filter - фильтрация заднего сектора
8. slam_toolbox - построение карты

Архитектура:
                                    ┌─────────────┐
  /teleop_keyboard/cmd_vel ────────>│             │
  /nav2/cmd_vel ───────────────────>│  twist_mux  │──> /cmd_vel ──> ESP32 (micro-ROS)
  /safety/cmd_vel ─────────────────>│             │                      │
                                    └─────────────┘                      │
                                                                         v
  ESP32 /wheel_encoders ──> odometry_node ──> /odom + /tf (odom->base_link)

  ESP32 IMU ──> micro_ros_agent ──> /imu/data (sensor_msgs/Imu)

  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
  │   RPLidar   │───>│ /scan_raw   │───>│laser_filter │───> /scan
  │   A1M8      │    └─────────────┘    │(remove back)│       │
  └─────────────┘                       └─────────────┘       v
                                                        ┌─────────────┐
                                                        │ slam_toolbox│──> /map
                                                        └─────────────┘

Использование:
    # На роботе:
    ros2 launch verter_admin mapping.launch.py

    # С другим портом ESP32:
    ros2 launch verter_admin mapping.launch.py esp32_port:=/dev/ttyUSB2

    # Управление (в другом терминале):
    ros2 run verter_admin teleop_keyboard
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_verter_admin = get_package_share_directory('verter_admin')

    # Пути к конфигурационным файлам
    urdf_file = os.path.join(pkg_verter_admin, 'urdf', 'verter_robot_minimal.urdf')
    slam_params_file = os.path.join(pkg_verter_admin, 'config', 'slam', 'slam_toolbox_params.yaml')
    twist_mux_config = os.path.join(pkg_verter_admin, 'config', 'twist_mux', 'twist_mux.yaml')
    laser_filter_config = os.path.join(pkg_verter_admin, 'config', 'laser_filters', 'laser_filter.yaml')

    # Читаем URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # =========================================================================
    # АРГУМЕНТЫ
    # =========================================================================

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/rplidar',
        description='Serial port for RPLiDAR'
    )

    esp32_port_arg = DeclareLaunchArgument(
        'esp32_port',
        default_value='/dev/esp32_chassis',
        description='Serial port for ESP32 chassis (micro-ROS)'
    )

    imu_esp32_port_arg = DeclareLaunchArgument(
        'imu_esp32_port',
        default_value='/dev/esp32_imu',
        description='Serial port for ESP32 IMU (micro-ROS)'
    )

    # =========================================================================
    # MICRO-ROS AGENT (связь с ESP32)
    # =========================================================================
    # ESP32 напрямую подписан на /cmd_vel и публикует /wheel_encoders
    # Agent обеспечивает serial ↔ DDS мост

    micro_ros_agent_chassis = ExecuteProcess(
        cmd=[
            'bash', '-c',
            ['source ~/microros_ws/install/setup.bash && '
             'exec ros2 run micro_ros_agent micro_ros_agent serial '
             '--dev ', LaunchConfiguration('esp32_port'), ' -b 115200']
        ],
        output='screen',
        sigterm_timeout='5',
        sigkill_timeout='10',
    )

    # =========================================================================
    # MICRO-ROS AGENT (связь с ESP32 IMU)
    # =========================================================================
    # ESP32 IMU публикует /imu/data (sensor_msgs/Imu)

    micro_ros_agent_imu = ExecuteProcess(
        cmd=[
            'bash', '-c',
            ['source ~/microros_ws/install/setup.bash && '
             'exec ros2 run micro_ros_agent micro_ros_agent serial '
             '--dev ', LaunchConfiguration('imu_esp32_port'), ' -b 115200']
        ],
        output='screen',
        sigterm_timeout='5',
        sigkill_timeout='10',
    )

    # =========================================================================
    # СИСТЕМА УПРАВЛЕНИЯ
    # =========================================================================

    # Twist Mux - мультиплексор команд скорости
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config],
        remappings=[
            ('cmd_vel_out', '/cmd_vel'),
        ],
        output='screen'
    )

    # Odometry Node - одометрия на основе энкодеров ESP32
    odometry_node = Node(
        package='verter_admin',
        executable='odometry_node',
        name='odometry_node',
        output='screen'
    )

    # =========================================================================
    # ROBOT STATE PUBLISHER (TF из URDF)
    # =========================================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # =========================================================================
    # СЕНСОРЫ
    # =========================================================================

    # RPLiDAR A1M8 - публикует в /scan_raw (до фильтрации)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port'),
            'frame_id': 'lidar_link',
            'scan_mode': 'Standard',
            'serial_baudrate': 115200,
            'inverted': False,
            'angle_compensate': True,
        }],
        remappings=[
            ('scan', '/scan_raw'),
        ],
        output='screen'
    )

    # Laser Filter - фильтрует задний сектор (корпус робота)
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        parameters=[laser_filter_config],
        remappings=[
            ('scan', '/scan_raw'),
            ('scan_filtered', '/scan'),
        ],
        output='screen'
    )

    # =========================================================================
    # SLAM TOOLBOX
    # =========================================================================

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': False}
        ]
    )

    # =========================================================================
    # LAUNCH DESCRIPTION
    # =========================================================================

    return LaunchDescription([
        # Аргументы
        lidar_port_arg,
        esp32_port_arg,
        imu_esp32_port_arg,

        # micro-ROS Agents
        micro_ros_agent_chassis,
        micro_ros_agent_imu,

        # Система управления
        twist_mux_node,
        odometry_node,

        # TF
        robot_state_publisher,

        # Сенсоры
        rplidar_node,
        laser_filter_node,

        # SLAM
        slam_toolbox_node,
    ])
