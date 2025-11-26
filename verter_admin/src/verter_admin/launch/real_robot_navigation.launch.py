#!/usr/bin/env python3
"""
Real Robot Navigation Launch File for Verter

Launches navigation stack for the real Verter robot with:
1. RPLiDAR A1M8 sensor
2. HC-SR04 ultrasonic sensors (7x)
3. SLAM Toolbox for mapping
4. Nav2 for navigation
5. RViz2 for visualization

Usage:
    ros2 launch verter_admin real_robot_navigation.launch.py

Note: This launch file is for REAL ROBOT only (not simulation).
      RPLiDAR driver must be installed on the robot.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_verter_admin = get_package_share_directory('verter_admin')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Paths to configuration files
    rviz_config = os.path.join(pkg_verter_admin, '..', '..', '..', '..', 'lidar_navigation.rviz')
    slam_params_file = os.path.join(pkg_verter_admin, 'config', 'slam', 'slam_toolbox_params.yaml')
    nav2_params_file = os.path.join(pkg_verter_admin, 'config', 'nav2', 'nav2_params.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')  # Real robot!
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')

    # RPLiDAR A1M8 Node
    # NOTE: Requires rplidar_ros package installed on the robot
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': serial_port,
            'frame_id': 'lidar_link',
            'scan_mode': 'Standard',
            'serial_baudrate': 115200,
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen'
    )

    # Distance Sensors Node (reads HC-SR04 from Arduino → publishes Range)
    distance_sensors = Node(
        package='verter_admin',
        executable='distance_sensors_node',
        name='distance_sensors_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # Range to LaserScan Converter (Range → LaserScan for Nav2)
    range_to_laserscan = Node(
        package='verter_admin',
        executable='range_to_laserscan',
        name='range_to_laserscan',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': False}
        ],
        remappings=[
            ('/scan', '/scan')  # RPLiDAR publishes on /scan
        ]
    )

    # Nav2 Bringup - delayed to allow SLAM to initialize
    nav2_bringup = TimerAction(
        period=5.0,  # Wait 5 seconds for SLAM to create initial map
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

    # # RViz2 (optional, usually run on remote PC)
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config],
    #     parameters=[{'use_sim_time': False}],
    #     output='screen'
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (false for real robot)'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for RPLiDAR'
        ),
        rplidar_node,
        distance_sensors,
        range_to_laserscan,
        slam_toolbox,
        nav2_bringup,
        # rviz,  # Comment out if running RViz on remote PC
    ])
