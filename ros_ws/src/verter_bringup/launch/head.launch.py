from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    # Disabled by owner: ESP32 ctrl has priority on the shared serial port.
    # Restore only after a separate head controller port is identified.
    # Node(package='verter_drivers', executable='doa_node')
    return LaunchDescription([LogInfo(msg='DOA disabled: ESP32 ctrl has priority; head port is not configured.')])
