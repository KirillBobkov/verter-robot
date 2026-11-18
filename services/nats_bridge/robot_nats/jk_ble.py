"""Allowlisted JK BLE reads and JK02_32S decoder; no settings-write API."""


def request(code):
    if code not in (0x96, 0x97):
        raise ValueError('Only telemetry and device-info reads are allowed')
    packet = bytearray.fromhex('aa5590eb') + bytearray([code]) + bytearray(15)
    packet[19] = sum(packet[:19]) & 255
    return packet


class Frames:
    def __init__(self):
        self.buffer = bytearray()
        self.invalid = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        result = []
        while len(self.buffer) >= 4:
            pos = self.buffer.find(bytes.fromhex('55aaeb90'))
            if pos < 0:
                self.buffer = self.buffer[-3:]
                break
            del self.buffer[:pos]
            if len(self.buffer) < 300:
                break
            frame = bytes(self.buffer[:300])
            if sum(frame[:299]) & 255 != frame[299]:
                self.invalid += 1
                del self.buffer[0]
                continue
            result.append(frame)
            del self.buffer[:300]
        return result


def device_info(frame):
    if len(frame) != 300 or frame[4] != 3 or sum(frame[:299]) & 255 != frame[299]:
        raise ValueError('Invalid device-info frame')

    def text(start, length):
        return frame[start : start + length].split(b'\0', 1)[0].decode('ascii', errors='replace')

    # Never extract, log or publish passcodes/user fields from this frame.
    return {'model': text(6, 16), 'hardware': text(22, 8), 'software': text(30, 8)}


def decode(frame):
    if len(frame) != 300 or frame[:5] != bytes.fromhex('55aaeb9002') or sum(frame[:299]) & 255 != frame[299]:
        raise ValueError('Invalid telemetry frame')

    def u16(i):
        return int.from_bytes(frame[i : i + 2], 'little')

    def i16(i):
        return int.from_bytes(frame[i : i + 2], 'little', signed=True)

    def u32(i):
        return int.from_bytes(frame[i : i + 4], 'little')

    def i32(i):
        return int.from_bytes(frame[i : i + 4], 'little', signed=True)

    mask = u32(70)
    cells = {str(i + 1): u16(6 + i * 2) / 1000 for i in range(32) if mask & (1 << i)}
    out = {
        'protocol': 'JK02_32S',
        'cells_V': cells,
        'voltage_V': u32(150) / 1000,
        'current_A': i32(158) / 1000,
        'soc_pct': frame[173],
        'remaining_Ah': u32(174) / 1000,
        'full_charge_capacity_Ah': u32(178) / 1000,
        'cycles': u32(182),
        'mos_temperature_C': i16(144) / 10,
        'temperature_1_C': i16(162) / 10,
        'temperature_2_C': i16(164) / 10,
        'flags_raw': f'0x{u32(166):08X}',
        'flags_interpretation': 'unverified_for_19.31',
        'balance_current_A': i16(170) / 1000,
        'balance_state': frame[172],
        'charging_enabled': bool(frame[198]),
        'discharging_enabled': bool(frame[199]),
    }
    out['power_W'] = round(out['voltage_V'] * out['current_A'], 3)
    out['cell_sum_V'] = round(sum(cells.values()), 3)
    out['cell_delta_V'] = round(max(cells.values()) - min(cells.values()), 3) if cells else None
    if (
        not cells
        or any(not 1 < v < 5 for v in cells.values())
        or abs(out['cell_sum_V'] - out['voltage_V']) > 0.3
        or not 0 <= out['soc_pct'] <= 100
    ):
        raise ValueError('Implausible telemetry')
    return out
