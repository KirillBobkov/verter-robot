#!/usr/bin/env python3
"""
Autonomous mapping launch for the real Verter robot.

Components:
1. micro_ros_agent (chassis + imu)
2. twist_mux, odometry, EKF
3. RPLiDAR + laser filter
4. Range converter
5. Proximity safety stop node
6. SLAM Toolbox
7. Nav2
8. Explore Lite
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
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

    urdf_file = os.path.join(pkg_verter_admin, 'urdf', 'verter_robot_minimal.urdf')
    slam_params_file = os.path.join(pkg_verter_admin, 'config', 'slam', 'slam_toolbox_params.yaml')
    nav2_params_file = os.path.join(
        pkg_verter_admin, 'config', 'nav2', 'nav2_mapping_real_params.yaml'
    )
    explore_params_file = os.path.join(
        pkg_verter_admin, 'config', 'explore', 'explore_lite_real_params.yaml'
    )
    laser_filter_config = os.path.join(
        pkg_verter_admin, 'config', 'laser_filters', 'laser_filter.yaml'
    )
    ekf_config = os.path.join(pkg_verter_admin, 'config', 'robot_localization', 'ekf.yaml')

    with open(urdf_file, 'r') as urdf_stream:
        robot_description = urdf_stream.read()

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/rplidar',
        description='Serial port for RPLiDAR',
    )
    esp32_port_arg = DeclareLaunchArgument(
        'esp32_port',
        default_value='/dev/esp32_chassis',
        description='Serial port for ESP32 chassis (micro-ROS)',
    )
    imu_esp32_port_arg = DeclareLaunchArgument(
        'imu_esp32_port',
        default_value='/dev/esp32_imu',
        description='Serial port for ESP32 IMU (micro-ROS)',
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
    micro_ros_agent_chassis = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            [
                'trap "kill 0; exit" TERM INT; '
                'while true; do '
                'source ~/microros_ws/install/setup.bash && '
                'ros2 run micro_ros_agent micro_ros_agent serial '
                '--dev ',
                LaunchConfiguration('esp32_port'),
                ' -b 921600 ',
                LaunchConfiguration('micro_ros_agent_extra_args'),
                '; '
                'echo "[micro_ros_agent chassis] exited, restarting in 2s..."; '
                'sleep 2; '
                'done',
            ],
        ],
        output='screen',
        sigterm_timeout='5',
        sigkill_timeout='10',
    )

    micro_ros_agent_imu = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            [
                'trap "kill 0; exit" TERM INT; '
                'while true; do '
                'source ~/microros_ws/install/setup.bash && '
                'ros2 run micro_ros_agent micro_ros_agent serial '
                '--dev ',
                LaunchConfiguration('imu_esp32_port'),
                ' -b 921600 ',
                LaunchConfiguration('micro_ros_agent_extra_args'),
                '; '
                'echo "[micro_ros_agent imu] exited, restarting in 2s..."; '
                'sleep 2; '
                'done',
            ],
        ],
        output='screen',
        sigterm_timeout='5',
        sigkill_timeout='10',
    )

    twist_mux_node = Node(
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
        parameters=[{'publish_tf': False}],
        output='screen',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    rplidar_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='rplidar_ros',
                executable='rplidar_node',
                name='rplidar_node',
                parameters=[
                    {
                        'serial_port': LaunchConfiguration('lidar_port'),
                        'frame_id': 'lidar_link',
                        'serial_baudrate': 115200,
                        'inverted': False,
                        'angle_compensate': True,
                    }
                ],
                remappings=[('scan', '/scan_raw')],
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
        remappings=[('scan', '/scan_raw'), ('scan_filtered', '/scan')],
        output='screen',
    )

    range_converter_node = Node(
        package='verter_admin',
        executable='range_converter_node',
        name='range_converter_node',
        output='screen',
    )

    proximity_safety_node = Node(
        package='verter_admin',
        executable='proximity_safety_node',
        name='proximity_safety_node',
        output='screen',
        parameters=[
            {
                'stop_distance': LaunchConfiguration('stop_distance'),
                'resume_distance': LaunchConfiguration('resume_distance'),
                'scan_topic': '/scan',
                'ultrasonic_topic': '/ultrasonic/ranges',
                'cmd_topic': TopicContract.SAFETY_CMD_VEL,
                'sensor_timeout_sec': MotionTimeouts.SENSOR_FRESHNESS_SEC,
                'command_timeout_sec': MotionTimeouts.TWIST_MUX_SOURCE_SEC,
                'override': False,
                'allow_degraded_motion': False,
            }
        ],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': False}],
    )

    nav2_bringup = TimerAction(
        period=18.0,
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
                }.items(),
            )
        ],
    )

    explore_lite = TimerAction(
        period=60.0,
        actions=[
            Node(
                package='explore_lite',
                executable='explore',
                name='explore_node',
                output='screen',
                parameters=[explore_params_file, {'use_sim_time': False}],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('costmap', '/global_costmap/costmap'),
                    ('costmap_updates', '/global_costmap/costmap_updates'),
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            lidar_port_arg,
            esp32_port_arg,
            imu_esp32_port_arg,
            micro_ros_agent_extra_args_arg,
            stop_distance_arg,
            resume_distance_arg,
            micro_ros_agent_chassis,
            micro_ros_agent_imu,
            twist_mux_node,
            odometry_node,
            ekf_node,
            robot_state_publisher,
            rplidar_node,
            laser_filter_node,
            range_converter_node,
            proximity_safety_node,
            slam_toolbox_node,
            nav2_bringup,
            explore_lite,
        ]
    )
