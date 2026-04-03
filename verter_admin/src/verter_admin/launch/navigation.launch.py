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
7. rplidar_node + laser_filter - лидар (с задержкой и respawn)
8. range_converter_node - ультразвуковые датчики ESP32 → Range
9. proximity_safety_node - экстренная остановка по ультразвуку
10. map_server - загрузка карты
11. amcl - локализация на карте
12. Nav2 - планирование и следование по пути (через nav2_navigation_no_smoother)

Поток скорости:
  Nav2 controller/behaviors → /nav2/cmd_vel_raw
  velocity_smoother          → /nav2/cmd_vel
  twist_mux (priority 10)   → /cmd_vel → ESP32

  /safety/cmd_vel (priority 200)  ─┐
  /teleop_keyboard/cmd_vel (100)  ─┼─→ twist_mux → /cmd_vel
  /nav2/cmd_vel (10)              ─┘

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
from verter_admin.contracts.motion import (
    MotionTimeouts,
    TopicContract,
)
from verter_admin.control.infrastructure import build_twist_mux_parameters


def generate_launch_description():
    pkg_verter_admin = get_package_share_directory('verter_admin')

    # Пути к конфигурационным файлам
    urdf_file = os.path.join(pkg_verter_admin, 'urdf', 'verter_robot_minimal.urdf')
    nav2_params_file = os.path.join(
        pkg_verter_admin, 'config', 'nav2', 'nav2_navigation_params.yaml'
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

    micro_ros_agent_extra_args_arg = DeclareLaunchArgument(
        'micro_ros_agent_extra_args',
        default_value='',
        description='Optional extra args for micro_ros_agent, e.g. -v6',
    )

    stop_distance_arg = DeclareLaunchArgument(
        'stop_distance',
        default_value='0.15',
        description='Emergency stop distance (m)',
    )

    resume_distance_arg = DeclareLaunchArgument(
        'resume_distance',
        default_value='0.20',
        description='Distance to release safety stop (m)',
    )

    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9090',
        description='Optional: TCP port for rosbridge WebSocket (used when waypoint_ui.launch.py is started alongside navigation).',
    )

    # =========================================================================
    # MICRO-ROS AGENTS
    # =========================================================================

    micro_ros_agent_chassis = ExecuteProcess(
        cmd=[
            'bash', '-c',
            ['trap "kill 0; exit" TERM INT; '
             'while true; do '
             'source ~/microros_ws/install/setup.bash && '
             'ros2 run micro_ros_agent micro_ros_agent serial '
             '--dev ', LaunchConfiguration('esp32_port'), ' -b 921600 ',
             LaunchConfiguration('micro_ros_agent_extra_args'), '; '
             'echo "[micro_ros_agent chassis] exited, restarting in 2s..."; '
             'sleep 2; '
             'done']
        ],
        output='screen',
        sigterm_timeout='5',
        sigkill_timeout='10',
    )

    micro_ros_agent_imu = ExecuteProcess(
        cmd=[
            'bash', '-c',
            ['trap "kill 0; exit" TERM INT; '
             'while true; do '
             'source ~/microros_ws/install/setup.bash && '
             'ros2 run micro_ros_agent micro_ros_agent serial '
             '--dev ', LaunchConfiguration('imu_esp32_port'), ' -b 921600 ',
             LaunchConfiguration('micro_ros_agent_extra_args'), '; '
             'echo "[micro_ros_agent imu] exited, restarting in 2s..."; '
             'sleep 2; '
             'done']
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
        parameters=[build_twist_mux_parameters()],
        remappings=[
            ('cmd_vel_out', TopicContract.FINAL_CMD_VEL),
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
    # ЛИДАР (с задержкой 3с и авто-перезапуском)
    # =========================================================================

    rplidar_node = TimerAction(
        period=3.0,
        actions=[
            Node(
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
                respawn=True,
                respawn_delay=5.0,
                output='screen',
            ),
        ],
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
    # УЛЬТРАЗВУКОВЫЕ ДАТЧИКИ
    # =========================================================================

    range_converter_node = Node(
        package='verter_admin',
        executable='range_converter_node',
        name='range_converter_node',
        output='screen',
    )

    # =========================================================================
    # БЕЗОПАСНОСТЬ (экстренная остановка по ультразвуку/лидару)
    # =========================================================================

    proximity_safety_node = Node(
        package='verter_admin',
        executable='proximity_safety_node',
        name='proximity_safety_node',
        output='screen',
        parameters=[{
            'stop_distance': LaunchConfiguration('stop_distance'),
            'resume_distance': LaunchConfiguration('resume_distance'),
            'scan_topic': '/scan',
            'ultrasonic_topic': '/ultrasonic/ranges',
            'cmd_topic': TopicContract.SAFETY_CMD_VEL,
            'sensor_timeout_sec': MotionTimeouts.SENSOR_FRESHNESS_SEC,
            'command_timeout_sec': MotionTimeouts.TWIST_MUX_SOURCE_SEC,
            'override': False,
            'allow_degraded_motion': False,
        }],
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
    # NAV2 NAVIGATION (с задержкой 5с для инициализации AMCL)
    # Используем кастомный launch с правильными ремапами через twist_mux:
    #   controller/behaviors → /nav2/cmd_vel_raw → velocity_smoother → /nav2/cmd_vel → twist_mux
    # =========================================================================

    nav2_bringup = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_verter_admin, 'launch', 'nav2_navigation.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': nav2_params_file,
                    'autostart': 'true',
                    'use_respawn': 'false',
                    'log_level': 'info',
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
        micro_ros_agent_extra_args_arg,
        stop_distance_arg,
        resume_distance_arg,
        rosbridge_port_arg,

        # micro-ROS Agents
        micro_ros_agent_chassis,
        micro_ros_agent_imu,

        # Управление и одометрия
        twist_mux_node,
        odometry_node,
        ekf_node,

        # TF
        robot_state_publisher,

        # Лидар (с задержкой и respawn)
        rplidar_node,
        laser_filter_node,

        # Ультразвуковые датчики
        range_converter_node,

        # Безопасность
        proximity_safety_node,

        # Локализация
        map_server,
        amcl,
        lifecycle_manager_localization,

        # Навигация (с задержкой 5 сек)
        nav2_bringup,
    ])
