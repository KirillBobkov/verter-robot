"""Offline JK 4E57 decoder. Input is a saved probe console log, never a serial port."""

import json, sys

WIDTH = {
    **{i: 2 for i in range(0x80, 0xB2)},
    0x85: 1,
    0x86: 1,
    0x89: 4,
    0x9D: 1,
    0xA9: 1,
    0xAA: 4,
    0xAB: 1,
    0xAC: 1,
    0xAE: 1,
    0xAF: 1,
    0xB1: 1,
    0xB2: 10,
    0xB3: 1,
    0xB4: 8,
    0xB5: 4,
    0xB6: 4,
    0xB7: 15,
    0xB8: 1,
    0xB9: 4,
    0xBA: 24,
    0xC0: 1,
}
for x in [0x88, 0x8D]:
    WIDTH.pop(x, None)


def decode(frame):
    if len(frame) < 20 or frame[:2] != b'NW':
        raise ValueError('header/short frame')
    size = int.from_bytes(frame[2:4], 'big')
    if len(frame) != size + 2:
        raise ValueError('length')
    if frame[-5] != 0x68:
        raise ValueError('end marker')
    if sum(frame[:-2]) & 65535 != int.from_bytes(frame[-2:], 'big'):
        raise ValueError('checksum')
    out = {'function': frame[8], 'frame_type': frame[10], 'raw_frame_hex': frame.hex()}
    if frame[8] != 6 or frame[10] not in (1, 2):
        return out
    data = frame[11:-9]
    i = 0
    tags = {}
    cells = {}
    while i < len(data):
        tag = data[i]
        i += 1
        if tag == 0x79:
            if i >= len(data):
                raise ValueError('cell length missing')
            length = data[i]
            i += 1
            if length % 3 or i + length > len(data):
                raise ValueError('cell data truncated')
            for j in range(i, i + length, 3):
                cells[str(data[j])] = int.from_bytes(data[j + 1 : j + 3], 'big') / 1000
            i += length
            continue
        width = WIDTH.get(tag)
        if width is None:
            out['unknown_tail_hex'] = data[i - 1 :].hex()
            break
        if i + width > len(data):
            raise ValueError('truncated tag')
        if tag in tags:
            raise ValueError('duplicate tag')
        tags[tag] = data[i : i + width]
        i += width
    out['registers_hex'] = {f'{k:02X}': v.hex() for k, v in tags.items() if k != 0xB2}
    if 0xB2 in tags:
        out['registers_hex']['B2'] = '[password omitted; present in raw frame]'
    out['cells_V'] = cells

    def num(k):
        return int.from_bytes(tags[k], 'big')

    for k, name, scale in [
        (0x83, 'voltage_V', 0.01),
        (0x85, 'soc_pct', 1),
        (0x87, 'cycles', 1),
        (0xAA, 'configured_capacity_Ah', 1),
        (0x8B, 'alarms_mask', 1),
        (0x8C, 'status_mask', 1),
        (0xC0, 'protocol_version', 1),
    ]:
        if k in tags:
            out[name] = num(k) * scale
    if 0x84 in tags:
        out['current_raw'] = num(0x84)
        if out.get('protocol_version') == 1:
            v = num(0x84)
            out['current_A'] = (v & 32767) * (0.01 if v & 32768 else -0.01)
            if 'voltage_V' in out:
                out['power_W'] = out['voltage_V'] * out['current_A']
    for k, name in [(0x80, 'mos_temperature_C'), (0x81, 'temperature_1_C'), (0x82, 'temperature_2_C')]:
        if k in tags:
            v = num(k)
            out[name] = v if v <= 100 else 100 - v
    if 'status_mask' in out:
        for bit, name in enumerate(['charging_enabled', 'discharging_enabled', 'balancer_enabled']):
            out[name] = bool(int(out['status_mask']) & (1 << bit))
    if cells:
        out['cell_delta_V'] = max(cells.values()) - min(cells.values())
    return out


class Stream:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, chunk):
        self.buf.extend(chunk)
        out = []
        while len(self.buf) >= 4:
            start = self.buf.find(b'NW')
            if start < 0:
                self.buf = self.buf[-1:]
                break
            del self.buf[:start]
            if len(self.buf) < 4:
                break
            n = int.from_bytes(self.buf[2:4], 'big') + 2
            if not 20 <= n <= 2048:
                del self.buf[0]
                continue
            if len(self.buf) < n:
                break
            try:
                out.append(decode(bytes(self.buf[:n])))
            except ValueError as e:
                out.append({'invalid_frame': str(e)})
                del self.buf[0]
                continue
            del self.buf[:n]
        return out


if __name__ == '__main__':
    stream = Stream()
    with open(sys.argv[1], encoding='utf-8', errors='replace') as f:
        for line in f:
            if not line.startswith('RX '):
                continue
            try:
                chunk = bytes.fromhex(line.split()[2])
            except (ValueError, IndexError):
                continue
            for obj in stream.feed(chunk):
                print(json.dumps(obj))
