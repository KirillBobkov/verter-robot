import json
from pathlib import Path
import unittest
from robot_nats.jk_ble import Frames, decode, request, device_info
from robot_nats.battery_state import BatteryState

FIXTURE = Path(__file__).parent / 'fixtures/bms-telemetry.hex'


class BmsTests(unittest.TestCase):
    def test_real_frame_decode(self):
        d = decode(bytes.fromhex(FIXTURE.read_text()))
        self.assertEqual(len(d['cells_V']), 7)
        self.assertAlmostEqual(d['cell_sum_V'], d['voltage_V'], places=3)
        self.assertEqual(d['soc_pct'], 67)
        self.assertEqual(d['current_A'], 3.986)
        self.assertNotIn('password', json.dumps(d))

    def test_fragments_noise_and_corruption(self):
        frame = bytes.fromhex(FIXTURE.read_text())
        parser = Frames()
        received = []
        for chunk in [b'noise' + frame[:2], frame[2:34], frame[34:128], frame[128:]]:
            received += parser.feed(chunk)
        self.assertEqual(received, [frame])
        bad = bytearray(frame)
        bad[25] ^= 1
        self.assertEqual(parser.feed(bytes(bad) + frame + frame), [frame, frame])
        self.assertGreater(parser.invalid, 0)
        self.assertLess(len(parser.buffer), 300)

    def test_no_settings_commands(self):
        for code in (0x96, 0x97):
            data = request(code)
            self.assertEqual(len(data), 20)
            self.assertEqual(sum(data[:19]) & 255, data[19])
            self.assertEqual(data[5], 0)
        for code in (0x01, 0x06, 0xAA, 0xFF):
            with self.assertRaises(ValueError):
                request(code)

    def test_password_fields_never_extracted(self):
        frame = bytearray(300)
        frame[:5] = bytes.fromhex('55aaeb9003')
        frame[6:17] = b'JK-B2A8S20P'
        frame[97:103] = b'secret'
        frame[299] = sum(frame[:299]) & 255
        d = device_info(frame)
        self.assertEqual(set(d), {'model', 'hardware', 'software'})
        self.assertNotIn('secret', json.dumps(d))

    def test_sample_age_not_reset_by_heartbeat(self):
        now = [100.0]
        state = BatteryState('robot', lambda: now[0])
        self.assertTrue(state.snapshot()['stale'])
        msg = {
            'v': 1,
            'robot_id': 'robot',
            'connected': True,
            'stale': False,
            'status': 'streaming',
            'sample_age_s': 6,
            'sample': {'voltage_V': 27},
        }
        state.accept(msg)
        now[0] += 3
        self.assertTrue(state.snapshot()['stale'])
        self.assertEqual(state.snapshot()['sample_age_s'], 9)
        msg['sample_age_s'] = 9
        state.accept(msg)
        self.assertTrue(state.snapshot()['stale'])

    def test_publisher_loss_and_disconnect(self):
        now = [0.0]
        state = BatteryState('robot', lambda: now[0])
        msg = {
            'v': 1,
            'robot_id': 'robot',
            'connected': True,
            'stale': False,
            'status': 'streaming',
            'sample_age_s': 0,
            'sample': {'voltage_V': 27},
        }
        state.accept(msg)
        now[0] = 9
        self.assertEqual(state.snapshot()['status'], 'publisher_lost')
        self.assertFalse(state.snapshot()['connected'])
        msg.update(connected=False, stale=True, status='reconnecting')
        state.accept(msg)
        self.assertTrue(state.snapshot()['stale'])

    def test_other_robot_and_nan_rejected(self):
        state = BatteryState('robot')
        with self.assertRaises(ValueError):
            state.accept({'v': 1, 'robot_id': 'another'})
        with self.assertRaises(ValueError):
            state.accept({'v': 1, 'robot_id': 'robot', 'sample_age_s': float('nan')})


if __name__ == '__main__':
    unittest.main()
