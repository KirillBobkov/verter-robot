import json
from pathlib import Path
import tempfile
import unittest

from robot_nats.history import History, RETENTION


class HistoryTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / 'history.sqlite'
        self.history = History(self.path)

    def tearDown(self):
        self.directory.cleanup()

    def battery(self, sample_id='one', state='charging', stale=False, current=1):
        return {
            'sample_id': sample_id,
            'sample_at': 1000,
            'stale': stale,
            'charging_state': state,
            'sample': {
                'current_A': current,
                'voltage_V': 28,
                'charging_enabled': True,
                'cells_V': {'1': 4.1},
                'flags_raw': '0x00080000',
                'protocol': 'JK02_32S',
            },
        }

    def test_sampling_transitions_stale_and_restart(self):
        self.history.record(self.battery(), now=1000)
        self.history.record(self.battery('two'), now=1002)
        self.history.record(self.battery('three', 'discharging', current=-2), now=1004)
        self.history.record(self.battery('three', 'discharging', current=-2), now=1015)
        self.history.record(self.battery(stale=True), now=1020)
        result = History(self.path).query({'from': 990, 'to': 1100}, now=1100)
        self.assertEqual(result['samples'], 3)
        self.assertEqual(result['stored_samples'], 3)
        self.assertIn('cell_1_V', result['series'])
        self.assertIn('flags_raw', result['series'])
        self.assertEqual(result['series']['charging_enabled'][0][1], 1)
        self.assertNotIn('protocol', result['series'])
        self.assertGreater(sum(row[0] for row in result['mode_seconds']), 0)
        self.assertGreater(sum(row[1] for row in result['mode_seconds']), 0)
        self.assertGreater(sum(row[3] for row in result['mode_seconds']), 50)

    def test_downsampling_preserves_extrema_and_limits_payload(self):
        for index in range(1440):
            battery = self.battery(str(index), current=100 if index == 80 else 1)
            self.history.record(battery, now=1000 + index * 10)
        result = self.history.query({'from': 1000, 'to': 15400, 'max_points': 20}, now=15400)
        self.assertLessEqual(len(result['series']['current_A']), 20)
        self.assertEqual(max(p[3] for p in result['series']['current_A']), 100)
        self.assertLess(len(json.dumps(result)), 900000)

    def test_retention_and_no_extrapolation_across_gap(self):
        self.history.record(self.battery(), now=1000)
        self.history.record(self.battery('two'), now=1000 + RETENTION + 10)
        result = self.history.query({}, now=1000 + RETENTION + 20)
        self.assertEqual(result['stored_samples'], 1)
        self.assertLessEqual(sum(row[0] for row in result['mode_seconds']), 15)

    def test_invalid_queries_and_empty_history(self):
        self.assertEqual(self.history.query({}, now=1000000)['series'], {})
        for args in [
            {'max_points': 10000},
            {'max_points': True},
            {'from': float('nan')},
            {'period_s': RETENTION + 1},
            {'period_s': 10, 'from': 1},
            {'extra': 1},
            {'from': 1, 'to': 2},
            {'to': 1000010},
        ]:
            with self.assertRaises(ValueError, msg=str(args)):
                self.history.query(args, now=1000000)
