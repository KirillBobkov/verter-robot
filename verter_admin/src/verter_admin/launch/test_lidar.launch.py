#!/usr/bin/env python3
"""
Test launch file for RPLiDAR only
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # RPLiDAR Node with minimal parameters (removing scan_mode)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'lidar_link',
            'serial_baudrate': 115200,
        }],
        output='screen'
    )

    return LaunchDescription([
        rplidar_node,
    ])
