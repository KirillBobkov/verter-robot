import asyncio
import math
import sys
from types import ModuleType, SimpleNamespace
import threading
import time
import unittest
from unittest.mock import patch

from robot_nats.localization import Localization
from robot_nats.map_data import MapStore
from robot_nats.telemetry import Telemetry, pose_observation


class FakeModes:
    def __init__(self):
        self.value = {
            'actual': 'localization',
            'map': 'floor',
            'map_path': '/maps/floor.yaml',
            'pid': 22,
            'stale': False,
        }

    def state(self):
        return dict(self.value)


class LocalizationTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.modes = FakeModes()
        self.telemetry = Telemetry(enabled=False)
        self.store = MapStore('robot')
        self.store.set_scope('live:22', self.modes.state())
        self.store.put_grid(
            'live:22', 100, 100, 0.1, {'x': -5, 'y': -5, 'yaw': 0}, [0] * 10000, time.time()
        )
        self.calls = []

        def publisher(pose, guard):
            guard()
            self.calls.append(dict(pose))
            return {'published_at': time.time()}

        self.localization = Localization(
            {'allow_initial_pose': True, 'ros_enabled': False},
            self.modes,
            self.telemetry,
            SimpleNamespace(store=self.store),
            publisher=publisher,
        )
        self.localization.snapshot()

    def observations(self, amcl_stamp=None, pose_stamp=None, scan_stamp=None):
        now = time.time()
        self.telemetry.put(
            'localization',
            {
                'x': 1.0,
                'y': 2.0,
                'yaw': 0.2,
                'frame_id': 'map',
                'stamp': now if amcl_stamp is None else amcl_stamp,
                'covariance': [0.0] * 36,
            },
        )
        self.store.put_pose('live:22', now if pose_stamp is None else pose_stamp, 1.1, 2.2, 0.3, 0)
        self.store.put_scan('live:22', now if scan_stamp is None else scan_stamp, [[1, 2]], 0, 'lidar', 1)

    def initial_args(self):
        state = self.localization.snapshot()
        return {'map': 'floor', 'session_id': state['session_id'], 'x': 1.0, 'y': 2.0, 'yaw': 0.3}

    def confirm_args(self):
        state = self.localization.snapshot()
        return {key: state[key] for key in ('map', 'session_id', 'initialization_id')}

    async def test_observation_and_tiny_covariance_never_confirm_automatically(self):
        self.observations()
        state = self.localization.snapshot()
        self.assertTrue(state['ready_for_confirmation'])
        self.assertFalse(state['verified'])
        self.assertEqual(self.calls, [])
        with self.assertRaisesRegex(ValueError, 'operator_confirmation_required'):
            self.localization.current_pose()

    async def test_explicit_confirmation_returns_current_tf_pose_not_old_amcl_pose(self):
        self.observations()
        self.localization.confirm(self.confirm_args())
        current = self.localization.current_pose()
        self.assertEqual((current['x'], current['y'], current['yaw']), (1.1, 2.2, 0.3))
        self.assertEqual(current['map'], 'floor')
        self.assertEqual(current['map_id'], 'live:22')
        self.assertEqual(self.calls, [])

    async def test_retained_pose_receipt_does_not_establish_current_session_provenance(self):
        self.observations(amcl_stamp=time.time() - 100)
        record = self.telemetry.snapshot()['sources']['localization']
        self.assertLess(record['age_s'], 1)
        self.assertGreater(record['sample_age_s'], 99)
        self.assertTrue(record['stale'])
        self.assertEqual(self.localization.snapshot()['reason'], 'waiting_for_current_session_amcl_pose')
        with self.assertRaises(ValueError):
            self.localization.confirm(self.confirm_args())

    async def test_stationary_amcl_age_separate_from_live_tf_and_scan(self):
        self.localization.minimum_stamp = time.time() - 100
        self.observations(amcl_stamp=time.time() - 60)
        self.assertTrue(self.telemetry.snapshot()['sources']['localization']['stale'])
        state = self.localization.confirm(self.confirm_args())
        self.assertTrue(state['verified'])
        self.assertGreater(state['amcl']['sample_age_s'], 59)

    async def test_stale_tf_or_scan_prevents_current_pose(self):
        self.observations()
        self.localization.confirm(self.confirm_args())
        self.observations(pose_stamp=time.time() - 3)
        with self.assertRaisesRegex(ValueError, 'fresh_current_session_map_pose_required'):
            self.localization.current_pose()
        self.observations(scan_stamp=time.time() - 3)
        with self.assertRaisesRegex(ValueError, 'fresh_current_session_map_scan_required'):
            self.localization.current_pose()

    async def test_scope_change_and_bridge_restart_invalidate_tokens(self):
        self.observations()
        old = self.confirm_args()
        self.localization.confirm(old)
        self.modes.value['pid'] = 23
        changed = self.localization.snapshot()
        self.assertFalse(changed['verified'])
        self.assertNotEqual(changed['session_id'], old['session_id'])
        with self.assertRaisesRegex(ValueError, 'localization_session_changed'):
            self.localization.confirm(old)
        restarted = Localization(
            self.localization.config, self.modes, self.telemetry, SimpleNamespace(store=self.store), self.localization.publisher
        )
        self.assertNotEqual(restarted.snapshot()['session_id'], changed['session_id'])

    async def test_initialize_publishes_once_and_requires_new_estimate_and_confirmation(self):
        self.observations()
        old = self.confirm_args()
        self.localization.confirm(old)
        state = await self.localization.initialize(self.initial_args())
        self.assertEqual(len(self.calls), 1)
        self.assertAlmostEqual(self.calls[0]['std_xy'], 0.25)
        self.assertAlmostEqual(self.calls[0]['std_yaw'], math.pi / 12)
        self.assertEqual(state['status'], 'waiting_estimate')
        self.assertFalse(state['verified'])
        self.assertNotEqual(state['initialization_id'], old['initialization_id'])
        with self.assertRaisesRegex(ValueError, 'localization_initialization_changed'):
            self.localization.confirm(old)
        with self.assertRaisesRegex(ValueError, 'waiting_for_current_session_amcl_pose'):
            self.localization.confirm(self.confirm_args())
        self.observations()
        self.assertTrue(self.localization.confirm(self.confirm_args())['verified'])
        self.assertEqual(len(self.calls), 1)

    async def test_map_session_and_mode_guards_before_publish(self):
        args = self.initial_args()
        args['map'] = 'other'
        with self.assertRaisesRegex(ValueError, 'localization_map_mismatch'):
            await self.localization.initialize(args)
        args = self.initial_args()
        self.modes.value['stale'] = True
        with self.assertRaisesRegex(ValueError, 'fresh_localization_mode_required'):
            await self.localization.initialize(args)
        self.assertEqual(self.calls, [])

    async def test_map_scope_change_while_waiting_is_rechecked_at_publish(self):
        def publisher(pose, guard):
            self.modes.value['pid'] = 23
            guard()
            self.calls.append(pose)

        self.localization.publisher = publisher
        with self.assertRaisesRegex(ValueError, 'localization_session_changed'):
            await self.localization.initialize(self.initial_args())
        self.assertEqual(self.calls, [])

    async def test_ttl_expires_while_waiting_without_a_publish(self):
        def publisher(pose, guard):
            time.sleep(0.02)
            guard()
            self.calls.append(pose)

        self.localization.publisher = publisher
        with self.assertRaisesRegex(ValueError, 'expired'):
            await self.localization.initialize(self.initial_args(), expires_at=time.time() + 0.005)
        self.assertEqual(self.calls, [])

    async def test_expired_command_and_disabled_ros_do_not_load_publisher(self):
        with self.assertRaisesRegex(ValueError, 'expired'):
            await self.localization.initialize(self.initial_args(), expires_at=time.time() - 1)
        self.localization.publisher = None
        with self.assertRaisesRegex(ValueError, 'ros_disabled'):
            await self.localization.initialize(self.initial_args())
        self.assertEqual(self.calls, [])

    async def test_close_revokes_a_waiting_publication(self):
        def publisher(pose, guard):
            self.localization.close()
            guard()
            self.calls.append(pose)

        self.localization.publisher = publisher
        with self.assertRaisesRegex(ValueError, 'localization_backend_closed'):
            await self.localization.initialize(self.initial_args())
        self.assertEqual(self.calls, [])

    async def test_cancel_revokes_worker_before_publication(self):
        entered = threading.Event()
        release = threading.Event()
        finished = threading.Event()

        def publisher(pose, guard):
            entered.set()
            release.wait(timeout=2)
            try:
                guard()
                self.calls.append(pose)
                return {'published_at': time.time()}
            finally:
                finished.set()

        self.localization.publisher = publisher
        task = asyncio.create_task(self.localization.initialize(self.initial_args()))
        while not entered.is_set():
            await asyncio.sleep(0)
        task.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await task
        release.set()
        self.assertTrue(await asyncio.to_thread(finished.wait, 2))
        self.assertEqual(self.calls, [])

    async def test_invalid_numeric_values_and_map_bounds(self):
        for key, value in [('x', True), ('y', float('nan')), ('yaw', float('inf')), ('std_xy', 0), ('std_yaw', -1)]:
            with self.subTest(key=key, value=value):
                args = self.initial_args()
                args[key] = value
                with self.assertRaises(ValueError):
                    await self.localization.initialize(args)
        args = self.initial_args()
        args['x'] = 100
        with self.assertRaisesRegex(ValueError, 'initial_pose_outside_map'):
            await self.localization.initialize(args)
        self.assertEqual(self.calls, [])

    async def test_missing_generation_cannot_confirm(self):
        self.observations()
        args = self.confirm_args()
        args['initialization_id'] = None
        with self.assertRaisesRegex(ValueError, 'invalid_localization_confirmation_args'):
            self.localization.confirm(args)

    async def test_map_path_change_with_same_pid_invalidates_session(self):
        old = self.localization.snapshot()['session_id']
        self.modes.value['map_path'] = '/maps/replaced.yaml'
        self.assertNotEqual(self.localization.snapshot()['session_id'], old)

    async def test_rotated_map_origin_is_respected_for_initial_pose_bounds(self):
        self.store.put_grid(
            'live:22', 100, 100, 0.1, {'x': -5, 'y': -5, 'yaw': math.pi / 2}, [0] * 10000, time.time()
        )
        args = self.initial_args()
        with self.assertRaisesRegex(ValueError, 'initial_pose_outside_map'):
            await self.localization.initialize(args)
        args.update(x=-6, y=-4, yaw=5 * math.pi)
        await self.localization.initialize(args)
        self.assertEqual(len(self.calls), 1)
        self.assertAlmostEqual(abs(self.calls[0]['yaw']), math.pi)

    async def test_wrong_amcl_frame_does_not_allow_confirmation(self):
        self.observations()
        record = dict(self.telemetry.snapshot()['sources']['localization']['value'])
        record['frame_id'] = 'odom'
        self.telemetry.put('localization', record)
        self.assertEqual(self.localization.snapshot()['reason'], 'invalid_amcl_pose')

    async def test_external_initialpose_revokes_confirmation_under_same_pid(self):
        self.observations()
        old = self.confirm_args()
        self.localization.confirm(old)
        self.telemetry.put('initial_pose', {'stamp': time.time(), 'x': 3.0, 'y': 4.0, 'yaw': 0.1})
        state = self.localization.snapshot()
        self.assertEqual(state['session_id'], old['session_id'])
        self.assertNotEqual(state['initialization_id'], old['initialization_id'])
        self.assertFalse(state['verified'])
        with self.assertRaisesRegex(ValueError, 'waiting_for_current_session_amcl_pose'):
            self.localization.current_pose()
        with self.assertRaisesRegex(ValueError, 'localization_initialization_changed'):
            self.localization.confirm(old)
        self.observations()
        self.assertFalse(self.localization.snapshot()['verified'])
        self.assertTrue(self.localization.confirm(self.confirm_args())['verified'])
        self.assertEqual(self.calls, [])

    async def test_own_reset_seen_during_async_publication_keeps_generation(self):
        generations = []

        def publisher(pose, guard):
            guard()
            generations.append(self.localization.initialization_id)
            stamp = time.time()
            self.telemetry.put('initial_pose', {**pose, 'stamp': stamp})
            self.localization.snapshot()
            self.calls.append(pose)
            return {'published_at': stamp}

        self.localization.publisher = publisher
        state = await self.localization.initialize(self.initial_args())
        self.assertEqual(state['initialization_id'], generations[0])
        self.assertFalse(state['verified'])
        self.assertEqual(len(self.calls), 1)

    async def test_delayed_own_echo_keeps_generation_after_publication(self):
        state = await self.localization.initialize(self.initial_args())
        stamp = state['last_initialization']['published_at']
        self.telemetry.put('initial_pose', {**self.calls[0], 'stamp': stamp})
        self.assertEqual(self.localization.snapshot()['initialization_id'], state['initialization_id'])

    async def test_external_reset_seen_during_async_publication_rotates_generation(self):
        generations = []

        def publisher(pose, guard):
            guard()
            generations.append(self.localization.initialization_id)
            stamp = time.time()
            self.telemetry.put('initial_pose', {**pose, 'x': 4.0, 'stamp': stamp})
            self.localization.snapshot()
            return {'published_at': stamp}

        self.localization.publisher = publisher
        state = await self.localization.initialize(self.initial_args())
        self.assertNotEqual(state['initialization_id'], generations[0])
        self.assertFalse(state['verified'])


class TelemetryPoseTests(unittest.TestCase):
    def test_pose_observation_preserves_header_frame_and_yaw(self):
        message = SimpleNamespace(
            header=SimpleNamespace(frame_id='map', stamp=SimpleNamespace(sec=100, nanosec=500000000)),
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=1, y=2),
                    orientation=SimpleNamespace(x=0, y=0, z=math.sin(0.4), w=math.cos(0.4)),
                ),
                covariance=[0.0] * 36,
            ),
        )
        record = pose_observation(message)
        self.assertAlmostEqual(record['yaw'], 0.8)
        self.assertEqual(record['stamp'], 100.5)
        self.assertEqual(record['frame_id'], 'map')
        message.pose.pose.orientation.w = 0
        self.assertIsNone(pose_observation(message)['yaw'])


class LazyPublisherTests(unittest.TestCase):
    def setUp(self):
        self.lifecycle = 3
        self.published = []
        self.interfaces = []
        self.after_lifecycle = lambda: None
        fixture = self

        class FakeContext:
            def shutdown(self):
                pass

        class FakeExecutor:
            def __init__(self, **kwargs):
                pass

            def add_node(self, node):
                pass

            def spin_once(self, **kwargs):
                pass

            def spin_until_future_complete(self, future, **kwargs):
                fixture.after_lifecycle()

            def shutdown(self):
                pass

        class FakeMessage:
            def __init__(self):
                self.header = SimpleNamespace(frame_id=None, stamp=None)
                self.pose = SimpleNamespace(
                    pose=SimpleNamespace(
                        position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
                    ),
                    covariance=[0.0] * 36,
                )

        class FakeNode:
            def create_publisher(self, kind, topic, qos):
                fixture.interfaces.append(('publish', topic))
                return SimpleNamespace(
                    get_subscription_count=lambda: 1,
                    publish=fixture.published.append,
                )

            def create_client(self, kind, topic):
                fixture.interfaces.append(('service', topic))
                future = SimpleNamespace(
                    done=lambda: True,
                    result=lambda: SimpleNamespace(current_state=SimpleNamespace(id=fixture.lifecycle)),
                )
                return SimpleNamespace(service_is_ready=lambda: True, call_async=lambda request: future)

            def get_clock(self):
                return SimpleNamespace(
                    now=lambda: SimpleNamespace(nanoseconds=int(time.time() * 1e9), to_msg=lambda: object())
                )

            def destroy_node(self):
                pass

        self.modules = {}

        def module(name, **attributes):
            result = ModuleType(name)
            result.__dict__.update(attributes)
            self.modules[name] = result

        module('rclpy', init=lambda **kwargs: None, create_node=lambda *args, **kwargs: FakeNode())
        module('rclpy.context', Context=FakeContext)
        module('rclpy.executors', SingleThreadedExecutor=FakeExecutor)
        module(
            'rclpy.qos',
            QoSProfile=lambda **kwargs: kwargs,
            ReliabilityPolicy=SimpleNamespace(RELIABLE=1),
            DurabilityPolicy=SimpleNamespace(VOLATILE=0),
        )
        module('geometry_msgs')
        module('geometry_msgs.msg', PoseWithCovarianceStamped=FakeMessage)
        module('lifecycle_msgs')
        module('lifecycle_msgs.srv', GetState=SimpleNamespace(Request=lambda: object()))

    def publish(self, guard=lambda: None):
        from robot_nats.localization_ros import publish_initial_pose

        with patch.dict(sys.modules, self.modules):
            return publish_initial_pose(
                {'x': 1.0, 'y': 2.0, 'yaw': 0.8, 'std_xy': 0.25, 'std_yaw': 0.2}, guard
            )

    def test_active_amcl_gets_exactly_one_initialpose_with_squared_uncertainty(self):
        result = self.publish()
        self.assertAlmostEqual(result['published_at'], time.time(), places=1)
        self.assertEqual(self.interfaces, [('publish', '/initialpose'), ('service', '/amcl/get_state')])
        self.assertEqual(len(self.published), 1)
        message = self.published[0]
        self.assertEqual(message.header.frame_id, 'map')
        self.assertAlmostEqual(message.pose.pose.orientation.z, math.sin(0.4))
        self.assertAlmostEqual(message.pose.covariance[0], 0.25 ** 2)
        self.assertAlmostEqual(message.pose.covariance[7], 0.25 ** 2)
        self.assertAlmostEqual(message.pose.covariance[35], 0.2 ** 2)

    def test_inactive_amcl_cannot_receive_deferred_initialpose(self):
        self.lifecycle = 2
        with self.assertRaisesRegex(RuntimeError, 'amcl_not_active'):
            self.publish()
        self.assertEqual(self.published, [])

    def test_guard_rechecked_after_lifecycle_response(self):
        valid = [True]
        self.after_lifecycle = lambda: valid.__setitem__(0, False)

        def guard():
            if not valid[0]:
                raise ValueError('session_changed')

        with self.assertRaisesRegex(ValueError, 'session_changed'):
            self.publish(guard)
        self.assertEqual(self.published, [])


if __name__ == '__main__':
    unittest.main()
