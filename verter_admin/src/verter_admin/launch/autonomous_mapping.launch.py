#!/usr/bin/env python3
"""
Autonomous Mapping Launch File for Verter Robot

Starts complete autonomous exploration and mapping system:
1. Gazebo simulation with hospital_corridor world
2. Robot spawn with sensors
3. SLAM Toolbox for mapping
4. Nav2 for navigation
5. Explore Lite for autonomous exploration
6. RViz2 for visualization

Usage:
    ros2 launch verter_admin autonomous_mapping.launch.py

The robot will automatically explore and map the environment.
You can also manually control it with:
    ros2 run verter_admin teleop_keyboard
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_verter_admin = get_package_share_directory('verter_admin')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Paths to configuration files
    urdf_file = os.path.join(pkg_verter_admin, 'urdf', 'verter_robot_gazebo.urdf')
    world_file = os.path.join(pkg_verter_admin, 'worlds', 'hospital_corridor.world')
    rviz_config = os.path.join(pkg_verter_admin, '..', '..', '..', '..', 'lidar_navigation.rviz')
    slam_params_file = os.path.join(pkg_verter_admin, 'config', 'slam', 'slam_toolbox_params.yaml')
    nav2_params_file = os.path.join(pkg_verter_admin, 'config', 'nav2', 'nav2_params.yaml')
    explore_params_file = os.path.join(pkg_verter_admin, 'config', 'explore', 'explore_lite_params.yaml')

    # Read URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo launch
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

    # Spawn robot
    # Position in corridor, away from center pillar at (0,0)
    # Robot should explore in ALL directions from this position
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'verter_robot',
            '-topic', 'robot_description',
            '-x', '3.0',   # Right side of corridor, away from pillar
            '-y', '0.5',   # Slightly offset from center
            '-z', '0.1'
        ],
        output='screen'
    )

    # Robot State Publisher
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

    # LaserScan merger for low obstacle detection (Gazebo simulation)
    # For real robot, use 'range_to_laserscan' instead
    laserscan_merger = Node(
        package='verter_admin',
        executable='laserscan_merger',
        name='laserscan_merger',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # SLAM Toolbox
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
            ('/scan', '/verter/scan')
        ]
    )

    # Nav2 Bringup - delayed to allow SLAM to initialize map
    nav2_bringup = TimerAction(
        period=8.0,  # Wait 8 seconds for SLAM to create initial map
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_params_file,
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    # Explore Lite for autonomous exploration - delayed to start after Nav2
    explore_lite = TimerAction(
        period=15.0,  # Wait for Nav2 to start (8s) + 7s buffer
        actions=[
            Node(
                package='explore_lite',
                executable='explore',
                name='explore_node',
                output='screen',
                parameters=[
                    explore_params_file,
                    {'use_sim_time': True}
                ],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('costmap', '/global_costmap/costmap'),
                    ('costmap_updates', '/global_costmap/costmap_updates')
                ]
            )
        ]
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        laserscan_merger,  # Merge LaserScan from distance sensors (Gazebo)
        slam_toolbox,
        nav2_bringup,
        explore_lite,
        rviz,
    ])
