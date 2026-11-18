import hashlib
import os
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def _slam_executable():
    try:
        prefix = Path(get_package_prefix('verter_slam'))
    except PackageNotFoundError:
        # A running supervisor can predate this package in the isolated install overlay.
        prefix = Path(get_package_prefix('verter_bringup')).parent / 'verter_slam'

    executable = prefix / 'lib/verter_slam/async_slam_toolbox_node'
    if not executable.is_file():
        raise RuntimeError('Build verter_slam first: bash tools/build.sh')

    return str(executable)


def _start_slam(_context):
    executable = _slam_executable()
    config = Path(share('verter_bringup')) / 'config/slam/slam_toolbox_params.yaml'
    parameters = {'use_sim_time': False}
    settings = {}
    actions = []
    run_dir = Path(os.environ.get('VERTER_RUN_DIR', '/home/jetson/verter-robot/run'))
    profile = run_dir / 'mapping-control-test'
    masked_marker = profile / 'armed-near-mask'
    masked_test = masked_marker.is_file()
    marker = masked_marker if masked_test else profile / 'armed'
    if marker.is_file():
        library = profile / 'skip_near_chains.so'
        karto = Path('/opt/ros/humble/lib/libkartoSlamToolbox.so')
        expected = (profile / 'karto.sha256').read_text().strip()
        if hashlib.sha256(karto.read_bytes()).hexdigest() != expected:
            raise RuntimeError('Mapping control test library no longer matches Karto')
        if not library.is_file() or not os.access(library, os.R_OK):
            raise RuntimeError('Mapping control test library is missing or unreadable')
        mask_node = profile / 'mask_scan.py'
        if masked_test and not mask_node.is_file():
            raise RuntimeError('Mapping control test scan mask is missing')
        try:
            marker.unlink()
        except FileNotFoundError:
            pass
        else:
            parameters.update(
                minimum_travel_distance=0.01,
                max_laser_range=8.0,
                use_scan_matching=True,
                do_loop_closing=False,
            )
            preload = os.environ.get('LD_PRELOAD', '')
            settings['additional_env'] = {'LD_PRELOAD': f'{library} {preload}'.strip()}
            if masked_test:
                parameters.update(
                    scan_topic='/scan_mapping_control_test',
                    fine_search_angle_offset=0.00349,
                    scan_buffer_size=30,
                )
                actions.append(LogInfo(msg=(
                    'Mapping control test: masked scan, 0.2 degree fine step, 30-scan buffer, '
                    'loops off, LinkNearChains off, 0.01 m gate, 8 m range'
                )))
                actions.append(ExecuteProcess(
                    cmd=['/usr/bin/python3', str(mask_node)], name='mapping_control_scan_mask', output='screen',
                ))
            else:
                actions.append(LogInfo(msg='Mapping control test: loops off, LinkNearChains off, 0.01 m gate, 8 m range'))

    actions.append(
        Node(
            executable=executable,
            name='slam_toolbox',
            namespace='/',
            output='screen',
            parameters=[str(config), parameters],
            **settings,
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(event=Shutdown(reason='Managed process exited'))])),
            OpaqueFunction(function=_start_slam),
            Node(
                package='nav2_map_server',
                executable='map_saver_server',
                name='map_saver',
                parameters=[{'use_sim_time': False, 'save_map_timeout': 10.0}],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map_saver',
                parameters=[{'autostart': True, 'node_names': ['map_saver']}],
            ),
        ]
    )
