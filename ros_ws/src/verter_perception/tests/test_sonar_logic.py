import json
import math
import unittest

from verter_perception.sonar_logic import (
    SENSOR_FRAMES,
    SENSOR_NAMES,
    SonarMonitor,
    classify_readings,
    validate_settings,
)


class SonarLogicTests(unittest.TestCase):
    def test_firmware_order_matches_urdf_frame_names(self):
        self.assertEqual(SENSOR_NAMES, (
            'right',
            'front_right_outer',
            'front_right_inner',
            'front_center',
            'front_left_inner',
            'front_left_outer',
            'left',
        ))
        self.assertEqual(SENSOR_FRAMES[0], 'sensor_right')
        self.assertEqual(SENSOR_FRAMES[3], 'sensor_front_center')
        self.assertEqual(SENSOR_FRAMES[6], 'sensor_left')

    def test_valid_readings_keep_indices_and_meter_values(self):
        values = [0.03, 0.2, 0.4, 0.6, 0.8, 1.2, 1.99]
        sample = classify_readings(values)
        self.assertEqual(sample.values, tuple(values))
        self.assertEqual(sample.valid_mask, (True,) * 7)

    def test_missing_echoes_never_become_maximum_or_free(self):
        values = [10.0, 0.0, float('nan'), float('inf'), -float('inf'), -1.0, 2.0]
        sample = classify_readings(values)
        self.assertEqual(sample.values, (None,) * 7)
        self.assertEqual(sample.valid_mask, (False,) * 7)
        json.dumps(sample.raw, allow_nan=False)

    def test_bounds_are_minimum_inclusive_and_maximum_exclusive(self):
        sample = classify_readings([0.0199, 0.02, 1.9999, 2.0, 2.0001, 10.0, 0.5])
        self.assertEqual(sample.valid_mask, (False, True, True, False, False, False, True))

    def test_custom_range_bounds(self):
        sample = classify_readings([0.09, 0.1, 0.2, 1.4, 1.5, 1.6, 0.5], 0.1, 1.5)
        self.assertEqual(sample.valid_mask, (False, True, True, True, False, False, True))

    def test_malformed_array_is_not_partially_mapped(self):
        for count in (0, 1, 6, 8):
            with self.subTest(count=count):
                with self.assertRaises(ValueError):
                    classify_readings([0.5] * count)

    def test_unexpected_non_numeric_fields_are_rejected(self):
        sample = classify_readings([None, 'bad', 0.3, 0.4, 0.5, 0.6, 0.7])
        self.assertEqual(sample.values[:3], (None, None, 0.3))

    def test_configuration_requires_finite_physical_values(self):
        validate_settings(0.02, 2.0, 0.26, 10.0)
        cases = [
            (0.0, 2.0, 0.26, 10.0),
            (2.0, 2.0, 0.26, 10.0),
            (0.02, 2.0, 0.0, 10.0),
            (0.02, 2.0, math.pi, 10.0),
            (0.02, 2.0, 0.26, 0.0),
            (0.02, 2.0, 0.26, 10.1),
            (0.02, float('inf'), 0.26, 10.0),
            (0.02, 2.0, float('nan'), 10.0),
        ]
        for values in cases:
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    validate_settings(*values)

    def test_publication_is_limited_to_ten_batches_per_second(self):
        monitor = SonarMonitor()
        emitted = []
        for tick in range(100):
            sample = monitor.observe([0.5] * 7, tick / 100.0)
            if sample is not None:
                emitted.append(tick)

        self.assertEqual(emitted, list(range(0, 100, 10)))
        self.assertEqual(monitor.published_ranges, 70)
        self.assertEqual(monitor.throttled_arrays, 90)

    def test_invalid_new_array_does_not_republish_previous_hits(self):
        monitor = SonarMonitor()
        self.assertIsNotNone(monitor.observe([0.5] * 7, 10.0))
        self.assertIsNone(monitor.observe([10.0] * 7, 10.2))
        self.assertEqual(monitor.published_arrays, 1)
        status = monitor.status(10.3)
        self.assertEqual(status['valid_mask'], [False] * 7)
        self.assertAlmostEqual(status['usable_array_age_s'], 0.3)

    def test_receipt_diagnostics_do_not_claim_per_sensor_freshness(self):
        monitor = SonarMonitor()
        initial = monitor.status(1.0)
        self.assertTrue(initial['raw_stale'])
        monitor.observe([0.5] * 7, 2.0)
        status = monitor.status(2.5)
        self.assertFalse(status['raw_stale'])
        self.assertFalse(status['per_sensor_freshness_available'])
        self.assertFalse(status['physical_indexing_verified'])
        self.assertFalse(status['field_of_view_calibrated'])
        self.assertEqual(status['timestamp_basis'], 'host_array_receipt')
        self.assertTrue(monitor.status(3.1)['raw_stale'])

    def test_malformed_latest_array_invalidates_latest_validity_mask(self):
        monitor = SonarMonitor()
        monitor.observe([0.5] * 7, 2.0)
        self.assertIsNone(monitor.observe([0.7] * 6, 2.1))
        status = monitor.status(2.2)
        self.assertEqual(status['malformed_arrays'], 1)
        self.assertEqual(status['valid_mask'], [False] * 7)
        self.assertIsNone(status['raw'])

    def test_status_is_strict_json_even_for_nonfinite_input(self):
        monitor = SonarMonitor()
        monitor.observe([float('nan'), float('inf'), -float('inf'), 10.0, 0.0, 0.5, 1.0], 2.0)
        json.dumps(monitor.status(2.2), allow_nan=False)

    def test_unchanged_readings_are_not_assumed_stale(self):
        monitor = SonarMonitor()
        self.assertIsNotNone(monitor.observe([0.5] * 7, 2.0))
        self.assertIsNotNone(monitor.observe([0.5] * 7, 2.2))
        self.assertEqual(monitor.valid_counts, [2] * 7)
        self.assertFalse(monitor.status(2.3)['raw_stale'])


if __name__ == '__main__':
    unittest.main()
