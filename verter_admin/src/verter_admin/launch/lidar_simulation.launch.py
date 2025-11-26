#!/usr/bin/env python3
"""
Launch file for Verter Robot with RPLiDAR A1M8 in Gazebo simulation with SLAM

This launch file starts:
1. Gazebo with test_room world
2. Spawns Verter robot with lidar at x=-3m
3. Robot state publisher for TF transforms
4. SLAM Toolbox for mapping
5. RViz2 with lidar navigation configuration

Usage:
    ros2 launch verter_admin lidar_simulation.launch.py

Control robot:
    ros2 run verter_admin teleop_keyboard
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_verter_admin = get_package_share_directory('verter_admin')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Path to URDF file
    urdf_file = os.path.join(pkg_verter_admin, 'urdf', 'verter_robot_gazebo.urdf')

    # Path to world file
    world_file = os.path.join(pkg_verter_admin, 'worlds', 'hospital_corridor.world')

    # Path to RViz config (in workspace root)
    rviz_config = os.path.join(pkg_verter_admin, '..', '..', '..', '..', 'lidar_navigation.rviz')

    # Path to SLAM Toolbox config
    slam_params_file = os.path.join(pkg_verter_admin, 'config', 'slam', 'slam_toolbox_params.yaml')

    # Read URDF content
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Gazebo launch with hospital_corridor world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'pause': 'false',
            'use_sim_time': 'true'
        }.items()
    )

    # Spawn robot in Gazebo
    # Spawn away from center to avoid pillar at (0,0) and other obstacles
    # Position: right side of corridor, clear of obstacles
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'verter_robot',
            '-topic', 'robot_description',
            '-x', '3.0',   # Right side of corridor, away from pillar
            '-y', '0.5',   # Slightly offset from center
            '-z', '0.1'  # Spawn at ground level (base_footprint at z=0)
        ],
        output='screen'
    )

    # Robot State Publisher - publishes TF transforms from URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
        }]
    )

    # Range to LaserScan converter for low obstacle detection
    # DISABLED: causing costmap issues, testing with lidar only
    # range_converter = Node(
    #     package='verter_admin',
    #     executable='range_to_laserscan',
    #     name='range_to_laserscan',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )

    # SLAM Toolbox - async mapping mode
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': True}
        ],
        remappings=[
            ('/scan', '/verter/scan')  # Remap to our lidar topic
        ]
    )

    # RViz2 with lidar navigation configuration
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        # range_converter,  # DISABLED: Convert distance sensors to LaserScan
        slam_toolbox,  # SLAM provides map->odom transform
        rviz,
    ])
