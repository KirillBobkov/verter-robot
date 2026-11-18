"""Offline JK02_32S telemetry decode for the confirmed JK-B2A8S20P 19H test."""

import json
from pathlib import Path
import sys


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
        'errors_mask': u32(166),
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
        raise ValueError('Decoded telemetry failed plausibility checks; do not use values')
    return out


if __name__ == '__main__':
    folder = Path(sys.argv[1])
    raw = [bytes.fromhex(x) for x in json.loads((folder / 'frames.json').read_text())]
    rows = [decode(x) for x in raw if x[4] == 2]
    result = {
        'samples': len(rows),
        'last': rows[-1],
        'voltage_min_V': min(x['voltage_V'] for x in rows),
        'voltage_max_V': max(x['voltage_V'] for x in rows),
        'current_min_A': min(x['current_A'] for x in rows),
        'current_max_A': max(x['current_A'] for x in rows),
    }
    (folder / 'telemetry.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
