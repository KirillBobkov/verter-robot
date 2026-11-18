"""Geometry review only: publishes static robot TF; no drivers or cmd_vel."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('verter_description'))
    return LaunchDescription(
        [
            DeclareLaunchArgument('gui', default_value='true'),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': (share / 'urdf/verter_robot.urdf').read_text()}],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                condition=IfCondition(LaunchConfiguration('gui')),
                arguments=['-d', str(share / 'rviz/geometry.rviz')],
            ),
        ]
    )
