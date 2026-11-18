import copy
import json
import tempfile
import unittest
from pathlib import Path
from robot_nats.contracts import project, charging_state, node_id, catalog
from robot_nats.service import Service
from robot_nats.access import configure_client


def fixture():
    return {
        'v': 1,
        'robot_id': 'robot',
        'boot_id': 'bridge-session',
        'updated_at': 100.0,
        'uptime_s': 12,
        'settings': {'note': ''},
        'mode': {'actual': 'idle'},
        'transport': {'connected': True, 'kv_error': None},
        'battery': {
            'sample_at': 99.0,
            'sample_id': 'bms-session:3',
            'stale': False,
            'sample': {'soc_pct': 77, 'current_A': 2.0, 'voltage_V': 27.0, 'cells_V': {'1': 3.8}},
        },
        'telemetry': {
            'error': None,
            'sources': {
                'graph': {
                    'received_at': 99.0,
                    'stale': False,
                    'value': {'nodes': ['imu'], 'full_node_names': ['/sensors/imu']},
                },
                'ultrasonics': {'received_at': 98.0, 'stale': False, 'value': {'meters': [10.0, 0.3]}},
            },
        },
    }


class ContractTests(unittest.TestCase):
    def test_frame_identity_types_and_units(self):
        values = project(fixture())
        for key in ('battery.soc', 'battery.current', 'battery.voltage', 'battery.cell_01_voltage'):
            self.assertEqual(values[key]['sample_id'], 'bms-session:3')
            self.assertEqual(values[key]['sample_at'], 99.0)
            self.assertEqual(values[key]['quality'], 'ok')
        self.assertEqual(values['battery.soc']['value'], 77)
        self.assertEqual(values['battery.soc']['unit'], '%')

    def test_charging_not_mos_flag_and_deadband(self):
        battery = fixture()['battery']
        battery['sample']['charging_enabled'] = True
        for current, expected in [
            (2, 'charging'),
            (-2, 'discharging'),
            (0, 'idle'),
            (0.1, 'idle'),
            (-0.1, 'idle'),
            (None, 'unknown'),
        ]:
            battery['sample']['current_A'] = current
            self.assertEqual(charging_state(battery), expected)
        battery['stale'] = True
        battery['sample']['current_A'] = 2
        self.assertEqual(charging_state(battery), 'unknown')

    def test_stale_unavailable_and_sentinel(self):
        state = fixture()
        state['updated_at'] = 110.0
        self.assertEqual(project(state)['battery.voltage']['quality'], 'stale')
        state['battery']['stale'] = True
        self.assertEqual(project(state)['battery.charging_state']['value'], 'unknown')
        values = project(fixture())
        self.assertIsNone(values['topic.ultrasonic_right']['value'])
        self.assertEqual(values['topic.ultrasonic_right']['quality'], 'unavailable')
        self.assertEqual(values['topic.ultrasonic_front_right_outer']['value'], 0.3)
        self.assertEqual(values['topic.odometry_x']['quality'], 'unavailable')

    def test_node_namespace_no_subject_tokens(self):
        name = '/sensors/imu'
        key = 'node.' + node_id(name)
        self.assertEqual(project(fixture())[key]['value'], {'name': name, 'present': True})
        self.assertNotIn('.', node_id(name))
        self.assertNotIn('/', node_id(name))
        self.assertNotEqual(node_id('/a/imu'), node_id('/b/imu'))

    def test_disappeared_node_and_cell(self):
        service = object.__new__(Service)
        service.boot_id = 'test'
        service.catalog = {}
        service.previous_metrics = {}
        state = fixture()
        first = service.metric_snapshot(state)
        state['battery']['sample']['cells_V'] = {}
        state['telemetry']['sources']['graph']['value']['full_node_names'] = []
        state['telemetry']['sources']['graph']['stale'] = True
        stale = service.metric_snapshot(state)
        self.assertEqual(stale['node.' + node_id('/sensors/imu')]['quality'], 'stale')
        state['telemetry']['sources']['graph']['stale'] = False
        second = service.metric_snapshot(state)
        self.assertFalse(second['node.' + node_id('/sensors/imu')]['value']['present'])
        self.assertEqual(second['battery.cell_01_voltage']['quality'], 'unavailable')
        self.assertEqual(second['service.kv_error']['quality'], 'ok')

    def test_catalog_and_acl_no_motion_publish(self):
        cfg = {'client_user': 'client', 'robot_id': 'robot', 'bucket': 'STATE', 'domain': 'LEAF'}
        server = {'authorization': {'users': [{'user': 'client', 'permissions': {'publish': [], 'subscribe': []}}]}}
        configure_client(server, cfg)
        before = copy.deepcopy(server)
        configure_client(server, cfg)
        self.assertEqual(server, before)
        allowed = server['authorization']['users'][0]['permissions']['publish']
        self.assertNotIn('robot.cmd_vel.command.set', allowed)
        self.assertNotIn('robot.*.command.>', allowed)
        self.assertTrue(catalog('robot')['cmd_vel']['implemented'])
        self.assertFalse(catalog('robot')['cmd_vel']['enabled_by_default'])


class ReadJournalTests(unittest.IsolatedAsyncioTestCase):
    async def test_polling_is_fresh_and_does_not_grow_journal(self):
        from robot_nats.protocol import encode, envelope

        with tempfile.TemporaryDirectory() as directory:
            service = Service(
                {
                    'robot_id': 'test',
                    'domain': 'TEST',
                    'bucket': 'TEST',
                    'maps': {},
                    'ros_enabled': False,
                    'data_dir': directory,
                    'allow_mode_changes': False,
                }
            )
            try:
                message = envelope('metrics.get')
                first = await service.execute(encode(message))
                second = await service.execute(encode(message))
                self.assertTrue(first['ok'])
                self.assertTrue(second['ok'])
                self.assertGreaterEqual(second['at'], first['at'])
                self.assertEqual(service.db.execute('SELECT count(*) FROM commands').fetchone()[0], 0)
            finally:
                await service.close()


if __name__ == '__main__':
    unittest.main()
