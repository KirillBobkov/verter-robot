"""Read-only navigation input validation; independent of ROS for offline tests."""

import math


def sample_valid(topic, message, now_s):
    stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
    if not -0.25 <= now_s - stamp < 1.0:
        return False
    if topic == '/scan':
        if len(message.ranges) < 2 or not message.header.frame_id:
            return False
        if not math.isfinite(message.angle_increment) or message.angle_increment <= 0:
            return False
        return any(math.isfinite(r) and message.range_min <= r <= message.range_max for r in message.ranges)
    return True


def missing_inputs(seen, now):
    return [
        topic for topic in ('/scan', '/odometry/filtered', '/imu/data') if now - seen.get(topic, float('-inf')) >= 1.0
    ]
