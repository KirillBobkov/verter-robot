"""Local-only localization policies. ROS and process control are replaced by fakes."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as S
from unittest import mock

import pytest


ROOT = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


try:
    import fcntl
except ImportError:
    # The tested controller never calls fcntl. This permits pure policy tests on
    # Windows without pretending a production supervisor is portable there.
    with mock.patch.dict(sys.modules, {'fcntl': S()}):
        runtime = load_module('localization_runtime', ROOT / 'tools/runtime.py')
else:
    runtime = load_module('localization_runtime', ROOT / 'tools/runtime.py')
probe = load_module('localization_probe', ROOT / 'tools/ros_probe.py')


class Backend:
    def __init__(self):
        self.calls = []

    def validate_map(self, path):
        self.calls.append(('validate', path))
        if path != 'explicit.yaml':
            raise ValueError('explicit map required')
        return path

    def preflight(self):
        self.calls.append('preflight')

    def stop_mode(self):
        self.calls.append('stop')

    def start_mode(self, mode, path):
        self.calls.append(('start', mode, path))

    def wait_ready(self, mode):
        self.calls.append(('ready', mode))


def test_localization_requires_map_before_any_stop():
    backend = Backend()
    controller = runtime.ModeController(backend)
    controller.mode = 'mapping'
    with pytest.raises(ValueError):
        controller.switch('localization', discard=True)
    assert backend.calls == [('validate', None)]
    assert controller.mode == 'mapping'


def test_localization_keeps_mapping_discard_guard():
    backend = Backend()
    controller = runtime.ModeController(backend)
    controller.mode = 'mapping'
    with pytest.raises(ValueError, match='Save map first'):
        controller.switch('localization', 'explicit.yaml')
    assert backend.calls == [('validate', 'explicit.yaml')]
    assert controller.mode == 'mapping'


def test_explicit_localization_start_and_same_map_are_idempotent():
    backend = Backend()
    controller = runtime.ModeController(backend)
    controller.switch('localization', 'explicit.yaml')
    assert backend.calls == [
        ('validate', 'explicit.yaml'), 'preflight', 'stop',
        ('start', 'localization', 'explicit.yaml'), ('ready', 'localization'),
    ]
    assert controller.mode == 'localization'
    assert controller.map_path == 'explicit.yaml'
    backend.calls.clear()
    controller.switch('localization', 'explicit.yaml')
    assert backend.calls == [('validate', 'explicit.yaml')]


def test_localization_exclusive_probe_precedes_its_launch(tmp_path, monkeypatch):
    backend = runtime.Processes(tmp_path, tmp_path)
    calls = []
    monkeypatch.setattr(backend, 'probe', lambda kind, timeout: calls.append(('probe', kind)))
    monkeypatch.setattr(backend, 'launch', lambda *args: calls.append(('launch', *args)))
    backend.start_mode('localization', '/explicit.yaml')
    assert calls == [('probe', 'localization-exclusive'), ('launch', 'mode', 'localization', 'map:=/explicit.yaml')]


def test_stop_mode_still_has_zero_side_effect_and_must_be_authorized(tmp_path, monkeypatch):
    backend = runtime.Processes(tmp_path, tmp_path)
    backend.children['mode'] = object()
    calls = []
    monkeypatch.setattr(backend, 'probe', lambda kind, timeout: calls.append(kind))
    monkeypatch.setattr(backend, 'stop', lambda group: calls.append('stop:' + group))
    backend.stop_mode()
    assert calls == ['shutdown_nav', 'stop:mode', 'zero']


def grid():
    return S(
        header=S(frame_id='map'),
        info=S(width=2, height=2, resolution=.03,
               origin=S(position=S(x=0., y=0., z=0.), orientation=S(x=0., y=0., z=0., w=1.))),
        data=[0, 0, 100, -1],
    )


def pose(stamp=90.):
    covariance = [0.] * 36
    covariance[0] = covariance[7] = covariance[35] = .001
    return S(
        header=S(frame_id='map', stamp=S(sec=int(stamp), nanosec=int((stamp % 1) * 1e9))),
        pose=S(pose=S(position=S(x=1., y=2., z=0.), orientation=S(x=0., y=0., z=0., w=1.)),
               covariance=covariance),
    )


def test_active_amcl_can_be_ready_while_waiting_for_manual_pose():
    result = probe.localization_report(
        {'map_server': 3, 'amcl': 3}, probe.map_snapshot(grid()), None, None, None, [],
    )
    assert result['ready']
    assert result['waiting_for_initial_pose']
    assert not result['initialized']
    assert not result['localization_verified']


def test_latched_pose_age_is_not_a_physical_verification_or_an_automatic_reset():
    snapshot = probe.pose_snapshot(pose(stamp=10.), now=100.)
    assert snapshot['valid'] and not snapshot['fresh']
    result = probe.localization_report(
        {'map_server': 3, 'amcl': 3}, probe.map_snapshot(grid()), snapshot,
        {'valid': True, 'fresh': True}, {'valid': True, 'fresh': True}, [],
    )
    assert result['initialized']
    assert not result['waiting_for_initial_pose']
    assert not result['localization_verified']


def test_stale_tf_does_not_request_another_initial_pose():
    result = probe.localization_report(
        {'map_server': 3, 'amcl': 3}, probe.map_snapshot(grid()), probe.pose_snapshot(pose(), 100.),
        {'valid': True, 'fresh': False}, {'valid': True, 'fresh': False}, [],
    )
    assert result['ready'] and not result['initialized']
    assert not result['waiting_for_initial_pose']


def test_malformed_map_and_covariance_do_not_pass():
    message = grid()
    message.data.pop()
    assert not probe.map_snapshot(message)['valid']
    message = pose()
    message.pose.covariance[35] = -1.
    assert not probe.pose_snapshot(message, 100.)['valid']


def test_amcl_future_transform_tolerance_is_accepted_but_stale_tf_is_not():
    message = S(
        header=S(stamp=S(sec=100, nanosec=500000000)),
        transform=S(translation=S(x=0., y=0., z=0.), rotation=S(x=0., y=0., z=0., w=1.)),
    )
    assert probe.tf_snapshot(message, 100.)['fresh']
    assert not probe.tf_snapshot(message, 104.)['fresh']


def test_unmanaged_sources_are_rejected_without_stopping_them():
    assert probe.localization_conflicts(['slam_toolbox'], 1, 0)
    assert probe.localization_conflicts(['controller_server'], 0, 0)
    assert not probe.localization_conflicts(['hardware_driver', 'verter_readiness_probe'], 0, 0)
    assert not probe.localization_conflicts(['map_server', 'amcl', 'lifecycle_manager_localization'], 1, 1, allow_owned=True)
    assert probe.localization_conflicts(['map_server', 'amcl', 'amcl'], 1, 2, allow_owned=True)


def fake_launch_modules():
    def action(*args, **kwargs):
        return S(args=args, kwargs=kwargs)

    def argument(name, **kwargs):
        return S(kind='argument', name=name, kwargs=kwargs)

    def node(**kwargs):
        return S(kind='node', kwargs=kwargs)

    return {
        'ament_index_python': S(),
        'ament_index_python.packages': S(get_package_share_directory=lambda name: '/isolated/' + name),
        'launch': S(LaunchDescription=lambda entities: S(entities=entities)),
        'launch.actions': S(DeclareLaunchArgument=argument, EmitEvent=action, RegisterEventHandler=action),
        'launch.event_handlers': S(OnProcessExit=action),
        'launch.events': S(Shutdown=action),
        'launch.substitutions': S(LaunchConfiguration=lambda name: S(name=name)),
        'launch_ros': S(),
        'launch_ros.actions': S(Node=node),
        'launch_ros.descriptions': S(ParameterFile=action),
        'nav2_common': S(),
        'nav2_common.launch': S(RewrittenYaml=action),
    }


def test_localization_launch_has_only_map_and_amcl_and_never_auto_seeds(monkeypatch):
    for name, module in fake_launch_modules().items():
        monkeypatch.setitem(sys.modules, name, module)
    module = load_module('localization_launch', ROOT / 'ros_ws/src/verter_bringup/launch/localization.launch.py')
    description = module.generate_launch_description()
    nodes = [entity.kwargs for entity in description.entities if getattr(entity, 'kind', None) == 'node']
    assert [node['name'] for node in nodes] == ['map_server', 'amcl', 'lifecycle_manager_localization']
    assert {node['package'] for node in nodes} == {'nav2_map_server', 'nav2_amcl', 'nav2_lifecycle_manager'}
    assert nodes[1]['parameters'][-1] == {
        'set_initial_pose': False, 'always_reset_initial_pose': True, 'save_pose_rate': -1.,
    }
    assert nodes[2]['parameters'][0]['node_names'] == ['map_server', 'amcl']
    map_argument = next(entity for entity in description.entities if getattr(entity, 'name', None) == 'map')
    assert 'default_value' not in map_argument.kwargs


def test_full_startup_probe_accepts_waiting_amcl_without_any_publisher(monkeypatch):
    """Exercise the actual readiness branch, replacing every ROS capability."""
    class Node:
        def create_subscription(self, typ, topic, callback, qos):
            if topic == '/map':
                callback(grid())
            return object()

        def create_publisher(self, *args, **kwargs):
            raise AssertionError('Localization readiness must never publish')

        def create_client(self, typ, name):
            assert name in ('/map_server/get_state', '/amcl/get_state')
            future = S(done=lambda: True, result=lambda: S(current_state=S(id=3)))
            return S(service_is_ready=lambda: True, call_async=lambda request: future)

        def get_clock(self):
            return S(now=lambda: S(nanoseconds=100000000000))

        def get_node_names(self):
            return ['map_server', 'amcl', 'lifecycle_manager_localization']

        def count_publishers(self, topic):
            return 1

        def destroy_node(self):
            pass

    class Buffer:
        def lookup_transform(self, *args):
            raise LookupError('No manual initial pose yet')

    node = Node()
    service = S(Request=lambda: object())
    modules = {
        'sensor_health': S(sample_valid=lambda *args: True, missing_inputs=lambda *args: []),
        'rclpy': S(init=lambda: None, shutdown=lambda: None, create_node=lambda *args, **kwargs: node,
                   spin_once=lambda *args, **kwargs: None, spin_until_future_complete=lambda *args, **kwargs: None,
                   time=S(Time=lambda: None)),
        'rclpy.qos': S(qos_profile_sensor_data=object(), QoSProfile=lambda **kwargs: object(),
                      DurabilityPolicy=S(TRANSIENT_LOCAL=1)),
        'sensor_msgs': S(), 'sensor_msgs.msg': S(LaserScan=object, Imu=object),
        'nav_msgs': S(), 'nav_msgs.msg': S(Odometry=object, OccupancyGrid=object),
        'geometry_msgs': S(), 'geometry_msgs.msg': S(Twist=object, PoseWithCovarianceStamped=object),
        'lifecycle_msgs': S(), 'lifecycle_msgs.srv': S(GetState=service),
        'nav2_msgs': S(), 'nav2_msgs.srv': S(ManageLifecycleNodes=service),
        'tf2_ros': S(Buffer=Buffer, TransformListener=lambda *args: object()),
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(sys, 'argv', ['ros_probe.py', 'localization'])
    probe.main()
