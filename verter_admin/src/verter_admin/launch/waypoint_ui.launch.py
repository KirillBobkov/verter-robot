#!/usr/bin/env python3
"""
Standalone launch file for the waypoint navigation UI.

Components started:
  1. WaypointManagerNode  — LifecycleNode for waypoint CRUD and patrol (L3/HMI).
  2. WebServerNode        — HTTP server for the static UI (L3/HMI, SI-9 isolated).
  3. rosbridge_websocket  — WebSocket bridge so the UI can reach ROS services.

Constraint C9: this launch file is STANDALONE — it does NOT bring up any driving stack.
  Nav2 must be running separately before this launch (e.g. via the full driving-stack launch).
  The waypoint_manager communicates with Nav2 only via the NavigateToPose action (C1).

Constraint C10: waypoints_file LaunchArgument default uses os.path.expanduser so
  ~/... paths are supported on the Jetson.

Usage:
  ros2 launch verter_admin waypoint_ui.launch.py
  ros2 launch verter_admin waypoint_ui.launch.py waypoints_file:=/absolute/path/waypoints.yaml
  ros2 launch verter_admin waypoint_ui.launch.py rosbridge_port:=9091 web_port:=8081
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    # -------------------------------------------------------------------------
    # Launch arguments
    # -------------------------------------------------------------------------

    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=os.path.expanduser('~/verter-robot/verter_admin/waypoints.yaml'),
        description='Full path to the YAML file used to persist waypoints (C10: expanduser applied).',
    )

    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9090',
        description='TCP port for the rosbridge WebSocket server.',
    )

    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='8080',
        description='TCP port for the static waypoint UI HTTP server.',
    )

    nav2_action_server_arg = DeclareLaunchArgument(
        'nav2_action_server',
        default_value='navigate_to_pose',
        description='Name of the Nav2 NavigateToPose action server (for remapping).',
    )

    # -------------------------------------------------------------------------
    # WaypointManagerNode — LifecycleNode (L3/HMI, SI-5/SI-6 enforced inside)
    # -------------------------------------------------------------------------

    waypoint_manager_node = LifecycleNode(
        package='verter_admin',
        executable='waypoint_manager_node',
        name='waypoint_manager',
        namespace='',
        parameters=[{
            # C10: os.path.expanduser applied to the default above;
            # if caller passes an absolute path it is used as-is.
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'nav2_action_server': LaunchConfiguration('nav2_action_server'),
            'patrol_warn_threshold': 3,
            'patrol_fault_threshold': 5,
        }],
        output='screen',
    )

    # -------------------------------------------------------------------------
    # WebServerNode — plain Node (SI-9: HMI only, not in actuator path)
    # -------------------------------------------------------------------------

    web_server_node = Node(
        package='verter_admin',
        executable='web_server_node',
        name='web_server_node',
        parameters=[{
            'port': LaunchConfiguration('web_port'),
        }],
        output='screen',
    )

    # -------------------------------------------------------------------------
    # rosbridge_websocket — WebSocket bridge for the UI
    # R6 NOTE: rosbridge allows unauthenticated access to ROS services.
    #           Add a topic blacklist for /cmd_vel and safety topics before
    #           any multi-user or production deployment. MVP accepted risk.
    # -------------------------------------------------------------------------

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': LaunchConfiguration('rosbridge_port'),
        }],
        output='screen',
    )

    # -------------------------------------------------------------------------
    # Launch description
    # -------------------------------------------------------------------------

    return LaunchDescription([
        waypoints_file_arg,
        rosbridge_port_arg,
        web_port_arg,
        nav2_action_server_arg,

        waypoint_manager_node,
        web_server_node,
        rosbridge_node,
    ])
