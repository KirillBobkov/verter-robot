from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    params = ParameterFile(
        RewrittenYaml(
            source_file=LC('params_file'),
            root_key='',
            param_rewrites={'yaml_filename': LC('map'), 'use_sim_time': 'false'},
            convert_types=True,
        ),
        allow_substs=True,
    )
    nodes = [
        ('nav2_map_server', 'map_server'),
        ('nav2_amcl', 'amcl'),
        ('nav2_controller', 'controller_server'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_waypoint_follower', 'waypoint_follower'),
    ]
    actions = [
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument(
            'params_file', default_value=str(Path(share('verter_bringup')) / 'config/nav2/nav2_navigation_params.yaml')
        ),
    ]
    actions += [
        Node(package=p, executable=n, name=n, parameters=[params], remappings=[('odom', '/odometry/filtered')])
        for p, n in nodes
    ]
    actions += [
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{'autostart': True, 'node_names': [n for _, n in nodes]}],
        )
    ]
    return LaunchDescription(
        [RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(event=Shutdown(reason='Managed process exited'))]))]
        + actions
    )
