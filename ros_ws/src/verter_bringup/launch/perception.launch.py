"""Obstacle observations only; this launch cannot command the chassis."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    configuration = Path(get_package_share_directory('verter_bringup')) / 'config/perception.yaml'
    parameters = LaunchConfiguration('perception_params')
    return LaunchDescription(
        [
            DeclareLaunchArgument('perception_params', default_value=str(configuration)),
            Node(
                package='verter_perception',
                executable='depth_obstacles',
                name='depth_obstacles',
                parameters=[parameters],
                respawn=True,
                respawn_delay=5.0,
                output='screen',
            ),
            Node(
                package='verter_perception',
                executable='ultrasonic_ranges',
                name='ultrasonic_ranges',
                parameters=[parameters],
                respawn=True,
                respawn_delay=5.0,
                output='screen',
            ),
        ]
    )
