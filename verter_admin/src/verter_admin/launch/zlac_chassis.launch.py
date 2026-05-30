#!/usr/bin/env python3
"""ZLAC8015D chassis bringup — standalone launch.

Brings up the chassis driver node only; the higher-level launches
(mapping.launch.py, nav2_navigation.launch.py) include this file rather than
duplicating the lifecycle wiring.

Stack:
  zlac_chassis_node          — LifecycleNode: /cmd_vel → ZLAC8015D, position
                               feedback → /wheel_encoders.
  nav2_lifecycle_manager     — drives the lifecycle node through
                               configure → activate on startup. Bond
                               monitoring is disabled because rclpy
                               LifecycleNode does not bind a bond by default.

Usage:
  ros2 launch verter_admin zlac_chassis.launch.py
  ros2 launch verter_admin zlac_chassis.launch.py port:=/dev/ttyUSB0 left_sign:=-1
  ros2 launch verter_admin zlac_chassis.launch.py autostart:=false
      (use when you want to drive the lifecycle manually via `ros2 lifecycle`)

Launch arguments mirror the chassis node parameters one-to-one so launch users
can tune signs and watchdog timing without editing code.
"""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_NODE_NAME = "zlac_chassis_node"


def generate_launch_description() -> LaunchDescription:
    # ---- Launch arguments --------------------------------------------------
    port_arg = DeclareLaunchArgument(
        "port", default_value="/dev/zlac_chassis",
        description="Serial port symlink for the isolated USB-RS485 converter",
    )
    baud_arg = DeclareLaunchArgument(
        "baud", default_value="115200",
        description="Modbus RTU baud rate (must match ZLAC8015D setting)",
    )
    slave_arg = DeclareLaunchArgument(
        "slave", default_value="1",
        description="Modbus slave address (default 1)",
    )
    watchdog_arg = DeclareLaunchArgument(
        "comm_watchdog_ms", default_value="200",
        description=(
            "Hardware comm-loss watchdog in ms — driver brakes if no Modbus "
            "traffic for this long. Software policy watchdog is 500 ms; this "
            "must be ≤ that"
        ),
    )
    wheel_diameter_arg = DeclareLaunchArgument(
        "wheel_diameter", default_value="0.200",
        description="Wheel outer diameter in meters",
    )
    wheel_base_arg = DeclareLaunchArgument(
        "wheel_base", default_value="0.386",
        description="Distance between left/right wheel contact patches (m)",
    )
    gear_ratio_arg = DeclareLaunchArgument(
        "gear_ratio", default_value="1.0",
        description="Motor → wheel reduction (1.0 for direct-drive hub motors)",
    )
    max_motor_rpm_arg = DeclareLaunchArgument(
        "max_motor_rpm", default_value="200",
        description="Hard ceiling for motor RPM (rated for ZLLG80ASM250)",
    )
    max_linear_vel_arg = DeclareLaunchArgument(
        "max_linear_velocity", default_value="0.5",
        description="Policy cap on |v_x| in m/s",
    )
    max_angular_vel_arg = DeclareLaunchArgument(
        "max_angular_velocity", default_value="1.0",
        description="Policy cap on |ω_z| in rad/s",
    )
    cmd_timeout_arg = DeclareLaunchArgument(
        "cmd_timeout", default_value="0.5",
        description="Software watchdog: drop to SAFE_STOP if no /cmd_vel within this many seconds",
    )
    left_sign_arg = DeclareLaunchArgument(
        "left_sign", default_value="1",
        description="Direction sign for left motor (+1 / -1) — calibrate on bench",
    )
    right_sign_arg = DeclareLaunchArgument(
        "right_sign", default_value="1",
        description="Direction sign for right motor (+1 / -1) — calibrate on bench",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart", default_value="true",
        description="If true, lifecycle_manager auto-configures and activates the chassis node",
    )

    # ---- Chassis lifecycle node -------------------------------------------
    chassis_node = Node(
        package="verter_admin",
        executable="zlac_chassis_node",
        name=_NODE_NAME,
        output="screen",
        parameters=[{
            "port":                  LaunchConfiguration("port"),
            "baud":                  LaunchConfiguration("baud"),
            "slave":                 LaunchConfiguration("slave"),
            "comm_watchdog_ms":      LaunchConfiguration("comm_watchdog_ms"),
            "wheel_diameter":        LaunchConfiguration("wheel_diameter"),
            "wheel_base":            LaunchConfiguration("wheel_base"),
            "gear_ratio":            LaunchConfiguration("gear_ratio"),
            "max_motor_rpm":         LaunchConfiguration("max_motor_rpm"),
            "max_linear_velocity":   LaunchConfiguration("max_linear_velocity"),
            "max_angular_velocity":  LaunchConfiguration("max_angular_velocity"),
            "cmd_timeout":           LaunchConfiguration("cmd_timeout"),
            "left_sign":             LaunchConfiguration("left_sign"),
            "right_sign":            LaunchConfiguration("right_sign"),
        }],
    )

    # ---- Lifecycle manager ------------------------------------------------
    # nav2_lifecycle_manager is already in the stack for nav2_navigation.launch.
    # bond_timeout=0.0 disables bond monitoring — rclpy LifecycleNode does not
    # create a bond, so any non-zero timeout would cause the manager to kill
    # the node after `bond_timeout` seconds thinking it died.
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_chassis",
        output="screen",
        parameters=[{
            "autostart":   LaunchConfiguration("autostart"),
            "node_names":  [_NODE_NAME],
            "bond_timeout": 0.0,
        }],
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        slave_arg,
        watchdog_arg,
        wheel_diameter_arg,
        wheel_base_arg,
        gear_ratio_arg,
        max_motor_rpm_arg,
        max_linear_vel_arg,
        max_angular_vel_arg,
        cmd_timeout_arg,
        left_sign_arg,
        right_sign_arg,
        autostart_arg,
        chassis_node,
        lifecycle_manager,
    ])
