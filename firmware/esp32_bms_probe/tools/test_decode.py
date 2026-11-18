import unittest
from decode import decode, Stream


def packet(data):
    b = bytearray.fromhex('4e57000000000000060001') + data + bytes.fromhex('00000000680000')
    b[2:4] = len(b).to_bytes(2, 'big')
    return bytes(b) + (sum(b) & 65535).to_bytes(2, 'big')


class Tests(unittest.TestCase):
    def setUp(self):
        self.frame = packet(bytes.fromhex('7906010ce4020cee830a5084806485648c0007c001'))

    def test_values(self):
        d = decode(self.frame)
        self.assertAlmostEqual(d['voltage_V'], 26.4)
        self.assertEqual(d['current_A'], 1)
        self.assertEqual(d['soc_pct'], 100)
        self.assertAlmostEqual(d['cell_delta_V'], 0.01)
        self.assertTrue(d['charging_enabled'])

    def test_checksum(self):
        b = bytearray(self.frame)
        b[14] ^= 1
        with self.assertRaises(ValueError):
            decode(bytes(b))

    def test_fragmented_noise_and_multiple(self):
        s = Stream()
        out = []
        for b in b'garbage' + self.frame + self.frame:
            out += s.feed(bytes([b]))
        self.assertEqual(len(out), 2)
        self.assertEqual(out[0]['current_A'], 1)

    def test_unknown_protocol_no_guessed_current(self):
        d = decode(packet(bytes.fromhex('840064c002')))
        self.assertNotIn('current_A', d)

    def test_truncation(self):
        with self.assertRaises(ValueError):
            decode(packet(bytes.fromhex('7906010ce4')))

    def test_known_read_request(self):
        d = decode(bytes.fromhex('4e5700130000000006030000000000006800000129'))
        self.assertEqual(d['frame_type'], 0)
        self.assertNotIn('voltage_V', d)


if __name__ == '__main__':
    unittest.main()
