import json
import unittest
from robot_nats.protocol import encode, envelope, validate


class ProtocolTests(unittest.TestCase):
    def test_expired_and_future(self):
        msg = envelope('ping')
        msg['issued_at'] = 100
        self.assertEqual(validate(encode(msg), now=101)['action'], 'ping')
        for now in (90, 131):
            with self.assertRaises(ValueError):
                validate(encode(msg), now=now)

    def test_bad_types_and_nonfinite(self):
        for key, value in [('v', True), ('ttl_s', float('nan')), ('args', []), ('action', 'cmd_vel')]:
            msg = envelope('ping')
            msg[key] = value
            with self.assertRaises(ValueError):
                validate(json.dumps(msg).encode())

    def test_uuid_and_size(self):
        msg = envelope('ping')
        msg['id'] = 'anything'
        with self.assertRaises(ValueError):
            validate(encode(msg))
        with self.assertRaises(ValueError):
            validate(b' ' * 17000)


if __name__ == '__main__':
    unittest.main()
