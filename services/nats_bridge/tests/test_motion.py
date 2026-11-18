"""All output goes into a Python list. No ROS context or real publisher is created."""

import json
import time
import unittest
import uuid
from robot_nats.motion import Motion, validate_request


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.output = []
        self.motion = Motion(True, lambda: self.now)
        self.motion.ready = True
        self.session = str(uuid.uuid4())

    def args(self, linear=0.01, angular=-0.05, seq=1):
        return {'session_id': self.session, 'seq': seq, 'linear_x': linear, 'angular_z': angular}

    def tick(self, connected=True):
        self.motion.tick(lambda x, z: self.output.append((x, z)), connected)

    def test_startup_status_and_shutdown_are_silent(self):
        for _ in range(10):
            self.motion.command('status', {})
            self.tick()
            self.now += 1
        self.motion.shutdown(lambda *args: self.output.append(args))
        self.assertEqual(self.output, [])

    def test_explicit_set_and_lease_expiry_no_replay(self):
        self.motion.command('set', self.args())
        self.tick()
        self.assertEqual(self.output, [(0.01, -0.05)])
        self.now = 1.1
        self.tick()
        self.assertEqual(self.output[-1], (0.0, 0.0))
        count = len(self.output)
        self.now = 5
        self.tick()
        self.assertEqual(len(self.output), count)
        with self.assertRaises(ValueError):
            self.motion.command('keepalive', {'session_id': self.session, 'seq': 2})
        with self.assertRaises(ValueError):
            self.motion.command('set', self.args(seq=3))

    def test_stop_closes_session_and_delayed_requests(self):
        issued = time.time()
        self.motion.command('set', self.args())
        self.motion.command('stop', {})
        self.tick()
        self.assertEqual(self.output, [(0.0, 0.0)])
        with self.assertRaises(ValueError):
            self.motion.command('set', self.args(seq=2))
        self.session = str(uuid.uuid4())
        with self.assertRaisesRegex(ValueError, 'precedes'):
            self.motion.command('set', self.args(), issued)

    def test_limits_types_and_order(self):
        for value in (0.51, -0.51, float('nan'), True, '0.1'):
            with self.assertRaises(ValueError):
                self.motion.command('set', self.args(linear=value))
        with self.assertRaises(ValueError):
            self.motion.command('set', self.args(angular=0.51))
        self.motion.command('set', self.args(seq=2))
        with self.assertRaises(ValueError):
            self.motion.command('set', self.args(seq=1))
        with self.assertRaises(ValueError):
            self.motion.command('set', dict(self.args(), session_id=str(uuid.uuid4())))
        self.assertEqual(self.output, [])

    def test_keepalive_and_disconnect(self):
        self.motion.command('set', self.args())
        self.now = 0.8
        self.motion.command('keepalive', {'session_id': self.session, 'seq': 2})
        self.now = 1.1
        self.tick()
        self.assertEqual(self.output[-1], (0.01, -0.05))
        self.tick(False)
        self.assertEqual(self.output[-1], (0.0, 0.0))
        self.assertFalse(self.motion.snapshot()['active'])

    def test_disabled_and_not_ready(self):
        self.motion.enabled = False
        with self.assertRaises(ValueError):
            self.motion.command('set', self.args())
        self.motion.enabled = True
        self.motion.ready = False
        with self.assertRaises(ValueError):
            self.motion.command('stop', {})
        self.tick()
        self.assertEqual(self.output, [])

    def test_request_deadline_strict(self):
        body = {'v': 1, 'id': str(uuid.uuid4()), 'issued_at': 100, 'ttl_s': 1, 'args': {}}
        self.assertEqual(validate_request(json.dumps(body).encode(), 100.5), body)
        for now in (99, 102):
            with self.assertRaises(ValueError):
                validate_request(json.dumps(body).encode(), now)
        for key, value in [('v', True), ('ttl_s', 30), ('ttl_s', float('nan')), ('args', [])]:
            with self.assertRaises(ValueError):
                validate_request(json.dumps(dict(body, **{key: value})).encode(), 100)


if __name__ == '__main__':
    unittest.main()
