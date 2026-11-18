"""Map and AMCL only. Initial pose must be supplied explicitly by an operator."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    params = ParameterFile(
        RewrittenYaml(
            source_file=LC('params_file'),
            root_key='',
            param_rewrites={'yaml_filename': LC('map'), 'use_sim_time': LC('use_sim_time')},
            convert_types=True,
        ),
        allow_substs=True,
    )
    actions = [
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(Path(share('verter_bringup')) / 'config/nav2/nav2_navigation_params.yaml'),
        ),
        Node(package='nav2_map_server', executable='map_server', name='map_server', parameters=[params]),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[
                params,
                {
                    # Explicit overrides win even if the supplied params file
                    # contains a previous pose. Humble AMCL then waits for input.
                    'set_initial_pose': False,
                    'always_reset_initial_pose': True,
                    'save_pose_rate': -1.0,
                },
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{
                'autostart': True,
                'use_sim_time': LC('use_sim_time'),
                'node_names': ['map_server', 'amcl'],
            }],
        ),
    ]
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(event=Shutdown(reason='Localization process exited'))])),
        *actions,
    ])
