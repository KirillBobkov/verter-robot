"""Validation and receipt diagnostics for the legacy seven-value sonar array."""

from dataclasses import dataclass
import math


SENSOR_NAMES = (
    'right',
    'front_right_outer',
    'front_right_inner',
    'front_center',
    'front_left_inner',
    'front_left_outer',
    'left',
)
SENSOR_FRAMES = tuple('sensor_' + name for name in SENSOR_NAMES)


@dataclass(frozen=True)
class SonarSample:
    values: tuple
    raw: tuple
    errors: tuple

    @property
    def valid_mask(self):
        return tuple(value is not None for value in self.values)


def validate_settings(min_range, max_range, field_of_view, output_rate_hz):
    settings = (min_range, max_range, field_of_view, output_rate_hz)
    if not all(math.isfinite(value) for value in settings):
        raise ValueError('Sonar settings must be finite')

    if min_range <= 0.0 or max_range <= min_range:
        raise ValueError('Require 0 < min_range < max_range')

    if not 0.0 < field_of_view < math.pi:
        raise ValueError('field_of_view must be between zero and pi radians')

    if not 0.0 < output_rate_hz <= 10.0:
        raise ValueError('output_rate_hz must be greater than zero and at most 10')


def classify_readings(readings, min_range=0.02, max_range=2.0):
    """Reject missing echoes; they never become a free-space observation."""
    if len(readings) != len(SENSOR_NAMES):
        raise ValueError('Expected exactly seven sonar readings')

    values = []
    raw = []
    errors = []

    for reading in readings:
        try:
            value = float(reading)
        except (TypeError, ValueError, OverflowError):
            values.append(None)
            raw.append(None)
            errors.append('not_numeric')
            continue

        if not math.isfinite(value):
            values.append(None)
            raw.append(str(value))
            errors.append('not_finite')
        elif value < min_range:
            values.append(None)
            raw.append(value)
            errors.append('below_min_range')
        elif value >= max_range:
            values.append(None)
            raw.append(value)
            errors.append('at_or_above_max_range')
        else:
            values.append(value)
            raw.append(value)
            errors.append(None)

    return SonarSample(tuple(values), tuple(raw), tuple(errors))


class SonarMonitor:
    """Track whole-array receipts, without implying individual sensor freshness."""

    def __init__(self, min_range=0.02, max_range=2.0, output_rate_hz=10.0):
        validate_settings(min_range, max_range, 0.26, output_rate_hz)
        self.min_range = min_range
        self.max_range = max_range
        self.output_interval = 1.0 / output_rate_hz
        self.received_arrays = 0
        self.malformed_arrays = 0
        self.throttled_arrays = 0
        self.published_arrays = 0
        self.published_ranges = 0
        self.valid_counts = [0] * len(SENSOR_NAMES)
        self.invalid_counts = [0] * len(SENSOR_NAMES)
        self.last_received_at = None
        self.last_usable_at = None
        self.last_published_at = None
        self.last_sample = None

    def observe(self, readings, received_at):
        self.received_arrays += 1
        self.last_received_at = received_at

        try:
            sample = classify_readings(readings, self.min_range, self.max_range)
        except (TypeError, ValueError):
            self.malformed_arrays += 1
            self.last_sample = None
            return None

        self.last_sample = sample
        for index, valid in enumerate(sample.valid_mask):
            if valid:
                self.valid_counts[index] += 1
            else:
                self.invalid_counts[index] += 1

        if not any(sample.valid_mask):
            return None

        self.last_usable_at = received_at
        if self.last_published_at is not None:
            elapsed = received_at - self.last_published_at
            if elapsed < self.output_interval - 1e-9:
                self.throttled_arrays += 1
                return None

        self.last_published_at = received_at
        self.published_arrays += 1
        self.published_ranges += sum(sample.valid_mask)
        return sample

    def status(self, now):
        def age(received_at):
            if received_at is None:
                return None

            return max(0.0, now - received_at)

        sample = self.last_sample
        raw_age = age(self.last_received_at)
        return {
            'source_topic': '/ultrasonics',
            'timestamp_basis': 'host_array_receipt',
            'per_sensor_freshness_available': False,
            'physical_indexing_verified': False,
            'field_of_view_calibrated': False,
            'raw_age_s': raw_age,
            'usable_array_age_s': age(self.last_usable_at),
            'raw_stale': raw_age is None or raw_age > 1.0,
            'raw': list(sample.raw) if sample else None,
            'valid_mask': list(sample.valid_mask) if sample else [False] * 7,
            'errors': list(sample.errors) if sample else None,
            'received_arrays': self.received_arrays,
            'malformed_arrays': self.malformed_arrays,
            'throttled_arrays': self.throttled_arrays,
            'published_arrays': self.published_arrays,
            'published_ranges': self.published_ranges,
            'sensors': [
                {
                    'index': index,
                    'name': name,
                    'frame_id': SENSOR_FRAMES[index],
                    'valid_readings': self.valid_counts[index],
                    'invalid_readings': self.invalid_counts[index],
                }
                for index, name in enumerate(SENSOR_NAMES)
            ],
            'limitations': [
                'Firmware has no per-sensor timestamps or sequence numbers.',
                'Fresh arrays do not establish freshness of individual fields.',
                'Physical sensor indexing and field of view require verification.',
                'Invalid or missing echoes do not indicate free space.',
            ],
        }
