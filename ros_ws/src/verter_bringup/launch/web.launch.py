import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                respawn=True,
                respawn_delay=5.0,
                package='rosbridge_server',
                executable='rosbridge_websocket',
                parameters=[{'port': 9090, 'address': '127.0.0.1'}],
            ),
            Node(
                respawn=True,
                respawn_delay=5.0,
                package='verter_web',
                executable='web_server_node',
                parameters=[{'port': 8081, 'html_dir': os.environ['VERTER_WEB_DIR']}],
            ),
        ]
    )
