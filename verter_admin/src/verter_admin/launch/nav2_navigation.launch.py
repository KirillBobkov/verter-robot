#!/usr/bin/env python3
"""Nav2 bringup for navigation with a saved map.

Sub-launch — included from a parent launch or run standalone:

  ros2 launch verter_admin nav2_navigation.launch.py map:=/path/to/map.yaml

Nodes started by this launch:
  map_server        — serves the saved map
  amcl              — Monte Carlo localisation
  controller_server — DWB local planner
  smoother_server   — path smoother
  planner_server    — NavFn global planner
  behavior_server   — spin / back-up / wait behaviours
  bt_navigator      — BT action server
  waypoint_follower — multi-waypoint action server
  velocity_smoother — smooth Nav2 cmd_vel output
  lifecycle_manager — manages the lifecycle of all above nodes

Velocity flow (matches twist_mux priorities):
  bt_navigator / waypoint_follower
      → controller_server
          → velocity_smoother → /nav2/cmd_vel
              → twist_mux → /cmd_vel → ESP32
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory('verter_admin')

    nav2_params_file = os.path.join(
        pkg, 'config', 'nav2', 'nav2_navigation_params.yaml'
    )

    # ---- Arguments -------------------------------------------------------------

    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Full path to map YAML file (required for standalone use)',
    )
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=nav2_params_file,
        description='Full path to Nav2 params YAML',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically start lifecycle nodes',
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')
    autostart    = LaunchConfiguration('autostart')
    map_file     = LaunchConfiguration('map')
    log_level    = LaunchConfiguration('log_level')

    # ---- Shared parameter -------------------------------------------------------

    set_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)

    # ---- Nav2 nodes ------------------------------------------------------------

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[params_file, {'yaml_filename': map_file}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[params_file],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[params_file],
        remappings=[
            ('cmd_vel', '/nav2/cmd_vel_raw'),    # → velocity_smoother
            ('odom',    '/odometry/filtered'),
        ],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[params_file],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[params_file],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[params_file],
        remappings=[('cmd_vel', '/nav2/cmd_vel_raw')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[params_file],
        remappings=[('odom', '/odometry/filtered')],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[params_file],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[params_file],
        remappings=[
            ('cmd_vel',      '/nav2/cmd_vel_raw'),   # input from controller
            ('cmd_vel_smoothed', '/nav2/cmd_vel'),   # output → twist_mux
            ('odom',         '/odometry/filtered'),
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'autostart': autostart,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }],
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        use_sim_time_arg,
        autostart_arg,
        log_level_arg,
        set_sim_time,
        map_server,
        amcl,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
    ])
