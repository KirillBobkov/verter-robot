from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(share('verter_bringup')) / 'config'
    description = (Path(share('verter_description')) / 'urdf/verter_robot.urdf').read_text()
    return LaunchDescription(
        [
            DeclareLaunchArgument('esp32_imu', default_value='/dev/esp32_imu'),
            DeclareLaunchArgument('esp32_ctrl', default_value='/dev/esp32_ctrl'),
            DeclareLaunchArgument(
                'lidar_port',
                default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            ),
            DeclareLaunchArgument('chassis_params', default_value=str(config / 'chassis.yaml')),
            Node(
                respawn=True,
                respawn_delay=5.0,
                package='verter_drivers',
                executable='chassis_zlac_node',
                parameters=[LC('chassis_params')],
            ),
            Node(
                respawn=True,
                respawn_delay=5.0,
                package='verter_drivers',
                executable='micro_ros_serial_agent',
                name='imu_agent',
                arguments=['--port', LC('esp32_imu')],
            ),
            Node(
                respawn=True,
                respawn_delay=5.0,
                package='verter_drivers',
                executable='micro_ros_serial_agent',
                name='ctrl_agent',
                arguments=['--port', LC('esp32_ctrl')],
            ),
            Node(
                respawn=True,
                respawn_delay=5.0,
                package='lslidar_driver',
                executable='lslidar_driver_node',
                name='lslidar_driver_node',
                parameters=[str(config / 'lidar/m10p.yaml'), {'serial_port_': LC('lidar_port')}],
            ),
            Node(
                respawn=True,
                respawn_delay=5.0,
                package='laser_filters',
                executable='scan_to_scan_filter_chain',
                name='laser_filter',
                parameters=[str(config / 'laser_filters/m10p_filter.yaml')],
                remappings=[('scan', '/scan_raw'), ('scan_filtered', '/scan')],
            ),
            Node(
                respawn=True,
                respawn_delay=5.0,
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': description}],
            ),
            Node(
                respawn=True,
                respawn_delay=5.0,
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                parameters=[str(config / 'robot_localization/ekf.yaml')],
            ),
        ]
    )
