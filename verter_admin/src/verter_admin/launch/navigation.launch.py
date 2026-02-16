#!/usr/bin/env python3
"""
Launch файл для навигации по готовой карте на реальном роботе Verter.

Компоненты:
1. micro_ros_agent (chassis) - связь с ESP32 шасси
2. micro_ros_agent (imu) - связь с ESP32 IMU
3. twist_mux - мультиплексор команд скорости
4. odometry_node - одометрия (энкодеры)
5. EKF - фьюзинг энкодеров + IMU
6. robot_state_publisher - TF из URDF
7. rplidar_node + laser_filter - лидар
8. map_server - загрузка карты
9. amcl - локализация на карте
10. Nav2 - планирование и следование по пути

Архитектура:
  /teleop_keyboard/cmd_vel ────>│           │
  /nav2/cmd_vel ───────────────>│ twist_mux │──> /cmd_vel ──> ESP32
                                └───────────┘
  ESP32 /wheel_encoders ──> odometry_node ──> /odom ─┐
  ESP32 /imu/data ──────────────────────────────────┤
                                                     └──> EKF ──> /odometry/filtered
  RPLiDAR ──> /scan_raw ──> laser_filter ──> /scan ──> AMCL (локализация)
                                                    └──> Nav2 (навигация)
  map_server ──> /map ──> AMCL + Nav2 global costmap

Использование:
    ros2 launch verter_admin navigation.launch.py
    ros2 launch verter_admin navigation.launch.py map:=/home/jetson/maps/my_map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_verter_admin = get_package_share_directory('verter_admin')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Пути к конфигурационным файлам
    urdf_file = os.path.join(pkg_verter_admin, 'urdf', 'verter_robot_minimal.urdf')
    nav2_params_file = os.path.join(
        pkg_verter_admin, 'config', 'nav2', 'nav2_navigation_params.yaml'
    )
    twist_mux_config = os.path.join(
        pkg_verter_admin, 'config', 'twist_mux', 'twist_mux.yaml'
    )
    laser_filter_config = os.path.join(
        pkg_verter_admin, 'config', 'laser_filters', 'laser_filter.yaml'
    )
    ekf_config = os.path.join(
        pkg_verter_admin, 'config', 'robot_localization', 'ekf.yaml'
    )

    # Читаем URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # =========================================================================
    # АРГУМЕНТЫ
    # =========================================================================

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/maps/my_map.yaml'),
        description='Full path to map yaml file'
    )

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
    # MICRO-ROS AGENTS
    # =========================================================================

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
    # УПРАВЛЕНИЕ И ОДОМЕТРИЯ
    # =========================================================================

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

    odometry_node = Node(
        package='verter_admin',
        executable='odometry_node',
        name='odometry_node',
        parameters=[{'publish_tf': False}],
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    # =========================================================================
    # ROBOT STATE PUBLISHER
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
    # ЛИДАР
    # =========================================================================

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port'),
            'frame_id': 'lidar_link',
            'scan_mode': 'Express',
            'serial_baudrate': 115200,
            'inverted': False,
            'angle_compensate': True,
        }],
        remappings=[
            ('scan', '/scan_raw'),
        ],
        output='screen'
    )

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
    # ЛОКАЛИЗАЦИЯ (map_server + AMCL)
    # =========================================================================

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'yaml_filename': LaunchConfiguration('map')}
        ],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file],
    )

    # Lifecycle manager для map_server и amcl
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }],
    )

    # =========================================================================
    # NAV2 NAVIGATION (с задержкой для инициализации локализации)
    # =========================================================================

    nav2_bringup = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': nav2_params_file,
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    # =========================================================================
    # LAUNCH DESCRIPTION
    # =========================================================================

    return LaunchDescription([
        # Аргументы
        map_arg,
        lidar_port_arg,
        esp32_port_arg,
        imu_esp32_port_arg,

        # micro-ROS Agents
        micro_ros_agent_chassis,
        micro_ros_agent_imu,

        # Управление и одометрия
        twist_mux_node,
        odometry_node,
        ekf_node,

        # TF
        robot_state_publisher,

        # Лидар
        rplidar_node,
        laser_filter_node,

        # Локализация
        map_server,
        amcl,
        lifecycle_manager_localization,

        # Навигация (с задержкой 5 сек)
        nav2_bringup,
    ])
