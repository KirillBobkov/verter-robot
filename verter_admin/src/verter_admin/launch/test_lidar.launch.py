#!/usr/bin/env python3
"""
Test launch file for RPLiDAR only
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # RPLiDAR Node with explicit parameters
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'lidar_link',
            'scan_mode': 'Standard',
            'serial_baudrate': 115200,
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen'
    )

    return LaunchDescription([
        rplidar_node,
    ])
