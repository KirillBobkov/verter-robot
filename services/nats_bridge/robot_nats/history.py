"""Seven-day BMS observations; synchronous methods run outside the asyncio loop."""

import json
import math
from contextlib import contextmanager
from pathlib import Path
import sqlite3
import time

RETENTION = 7 * 86400
INTERVAL = 10
STATES = ('charging', 'discharging', 'idle', 'unknown')


def numeric_fields(sample):
    values = {}
    for key, value in sample.items():
        if key == 'cells_V' and isinstance(value, dict):
            for cell, voltage in value.items():
                if type(voltage) in (int, float) and math.isfinite(voltage):
                    values[f'cell_{cell}_V'] = voltage
        elif isinstance(value, bool):
            values[key] = int(value)
        elif type(value) in (int, float) and math.isfinite(value):
            values[key] = value
        elif key == 'flags_raw' and isinstance(value, str):
            try:
                values[key] = int(value, 16)
            except ValueError:
                pass
    return values


class History:
    def __init__(self, path):
        self.path = Path(path)
        self.last_recorded = None
        self.last_state = None
        self.last_sample_id = None
        self.last_pruned = 0

    @contextmanager
    def connect(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        connection = sqlite3.connect(self.path, timeout=2)
        connection.execute('PRAGMA journal_mode=WAL')
        connection.execute('PRAGMA synchronous=NORMAL')
        connection.execute(
            'CREATE TABLE IF NOT EXISTS observations ('
            'at REAL PRIMARY KEY, sample_at REAL, sample_id TEXT, state TEXT NOT NULL, sample TEXT)'
        )
        try:
            yield connection
            connection.commit()
        except Exception:
            connection.rollback()
            raise
        finally:
            connection.close()

    def record(self, battery, now=None):
        now = time.time() if now is None else now
        fresh = not battery.get('stale', True) and isinstance(battery.get('sample'), dict)
        state = battery.get('charging_state', 'unknown') if fresh else 'unknown'
        if state not in STATES:
            state = 'unknown'
        sample_id = battery.get('sample_id') if fresh else None
        # A repeated BLE sample must not become a new measurement in the archive.
        if self.last_recorded is not None and now >= self.last_recorded:
            if state == self.last_state and now - self.last_recorded < INTERVAL:
                return
            if fresh and sample_id and sample_id == self.last_sample_id:
                return
        sample = battery['sample'] if fresh else None
        with self.connect() as connection:
            connection.execute(
                'INSERT OR IGNORE INTO observations VALUES (?, ?, ?, ?, ?)',
                (
                    now,
                    battery.get('sample_at') if fresh else None,
                    sample_id,
                    state,
                    json.dumps(sample, allow_nan=False) if sample is not None else None,
                ),
            )
            if now - self.last_pruned >= 3600 or now < self.last_pruned:
                connection.execute('DELETE FROM observations WHERE at < ?', (now - RETENTION,))
                self.last_pruned = now
        self.last_recorded = now
        self.last_state = state
        self.last_sample_id = sample_id

    def query(self, args, now=None):
        now = time.time() if now is None else now
        if 'raw' in args:
            return self.raw_page(args, now)
        if set(args) - {'from', 'to', 'max_points', 'period_s'}:
            raise ValueError('unexpected_history_args')
        if 'period_s' in args and ('from' in args or 'to' in args):
            raise ValueError('period_s_cannot_be_combined_with_from_to')
        period = args.get('period_s', RETENTION)
        if type(period) not in (int, float) or not math.isfinite(period) or not 0 < period <= RETENTION:
            raise ValueError('invalid_history_period')
        end = args.get('to', now)
        start = args.get('from', end - period)
        points = args.get('max_points', 240)
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in (start, end)):
            raise ValueError('invalid_history_time')
        if not 0 < end - start <= RETENTION or end > now + 2:
            raise ValueError('history_range_must_be_within_seven_days')
        if type(points) is not int or not 20 <= points <= 360:
            raise ValueError('max_points_must_be_20_to_360')
        start = max(start, now - RETENTION)
        if start >= end:
            raise ValueError('history_range_expired')
        step = (end - start) / points
        with self.connect() as connection:
            rows = connection.execute(
                'SELECT at, state, sample FROM observations WHERE at >= ? AND at <= ? ORDER BY at', (start, end)
            ).fetchall()
            coverage = connection.execute(
                'SELECT MIN(at), MAX(at), COUNT(*) FROM observations WHERE at >= ?', (now - RETENTION,)
            ).fetchone()
        buckets = [{} for _ in range(points)]
        modes = [[0.0, 0.0, 0.0, 0.0] for _ in range(points)]
        for index, (at, state, raw) in enumerate(rows):
            bucket = min(points - 1, int((at - start) / step))
            if raw:
                for key, value in numeric_fields(json.loads(raw)).items():
                    stats = buckets[bucket].setdefault(key, [0.0, value, value, value, 0])
                    stats[0] += value
                    stats[1] = min(stats[1], value)
                    stats[2] = max(stats[2], value)
                    stats[3] = value
                    stats[4] += 1
            # Do not extrapolate a last known mode through a disconnected/stopped collector.
            next_at = rows[index + 1][0] if index + 1 < len(rows) else end
            until = min(next_at, at + INTERVAL + 5, end)
            cursor = at
            while cursor < until:
                b = min(points - 1, int((cursor - start) / step))
                boundary = min(until, start + (b + 1) * step)
                if boundary <= cursor:
                    break
                modes[b][STATES.index(state)] += boundary - cursor
                cursor = boundary
        series = {}
        for index, bucket in enumerate(buckets):
            for key, (total, low, high, last, count) in bucket.items():
                series.setdefault(key, []).append(
                    [round(start + (index + 0.5) * step, 3), total / count, low, high, last]
                )
        for durations in modes:
            durations[3] += max(0, step - sum(durations))
        return {
            'from': start,
            'to': end,
            'bucket_s': step,
            'retention_s': RETENTION,
            'sample_interval_s': INTERVAL,
            'samples': len(rows),
            'available_from': coverage[0],
            'available_to': coverage[1],
            'stored_samples': coverage[2],
            'point_fields': ['at', 'mean', 'min', 'max', 'last'],
            'series': series,
            'states': list(STATES),
            'mode_seconds': modes,
        }

    def raw_page(self, args, now):
        if set(args) - {'from', 'to', 'period_s', 'raw', 'after', 'limit'} or args.get('raw') is not True:
            raise ValueError('invalid_raw_history_args')
        if 'period_s' in args and ('from' in args or 'to' in args):
            raise ValueError('period_s_cannot_be_combined_with_from_to')
        period = args.get('period_s', RETENTION)
        if type(period) not in (int, float) or not math.isfinite(period) or not 0 < period <= RETENTION:
            raise ValueError('invalid_history_period')
        end = args.get('to', now)
        start = args.get('from', end - period)
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in (start, end)):
            raise ValueError('invalid_history_time')
        if not 0 < end - start <= RETENTION or end > now + 2:
            raise ValueError('history_range_must_be_within_seven_days')
        start = max(start, now - RETENTION)
        if start >= end:
            raise ValueError('history_range_expired')
        cursor = args.get('after')
        if cursor is not None:
            if type(cursor) not in (int, float) or not math.isfinite(cursor) or not start <= cursor <= end:
                raise ValueError('invalid_history_cursor')
        limit = args.get('limit', 250)
        if type(limit) is not int or not 1 <= limit <= 250:
            raise ValueError('history_limit_must_be_1_to_250')
        with self.connect() as connection:
            rows = connection.execute(
                'SELECT at, sample_at, sample_id, state, sample FROM observations '
                'WHERE at >= ? AND at <= ? AND (? IS NULL OR at > ?) ORDER BY at LIMIT ?',
                (start, end, cursor, cursor, limit + 1),
            ).fetchall()
        records = []
        size = 0
        for at, sample_at, sample_id, state, sample in rows[:limit]:
            row = [at, sample_at, sample_id, state, json.loads(sample) if sample else None]
            length = len(json.dumps(row, ensure_ascii=False).encode())
            if size + length > 512 * 1024:
                if not records:
                    raise ValueError('history_record_too_large')
                break
            records.append(row)
            size += length
        more = len(rows) > len(records)
        return {
            'from': start,
            'to': end,
            'raw': True,
            'row_fields': ['received_at', 'sample_at', 'sample_id', 'charging_state', 'sample'],
            'rows': records,
            'next_after': records[-1][0] if more else None,
        }
