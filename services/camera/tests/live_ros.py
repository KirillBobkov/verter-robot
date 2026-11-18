"""Passive ROS camera validation. Subscribes only; no device or motion writes."""

import json, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = rclpy.create_node('camera_acceptance_observer')
received = {}
counts = {}
subs = []
for stream in ('color', 'depth'):
    for part, kind in [('image_raw', Image), ('camera_info', CameraInfo)]:
        topic = f'/camera/{stream}/{part}'

        def callback(msg, topic=topic):
            received[topic] = msg
            counts[topic] = counts.get(topic, 0) + 1

        subs.append(node.create_subscription(kind, topic, callback, qos_profile_sensor_data))
buffer = Buffer()
listener = TransformListener(buffer, node)
start = time.monotonic()
while time.monotonic() - start < 12:
    rclpy.spin_once(node, timeout_sec=0.1)
results = {}
for s in ('color', 'depth'):
    image = received[f'/camera/{s}/image_raw']
    info = received[f'/camera/{s}/camera_info']
    assert image.width == info.width and image.height == info.height
    assert image.header.frame_id == info.header.frame_id
    assert info.k[0] > 0 and info.k[4] > 0
    results[s] = {
        'width': image.width,
        'height': image.height,
        'encoding': image.encoding,
        'frames_in_12_s': counts[f'/camera/{s}/image_raw'],
        'frame': image.header.frame_id,
        'fx': info.k[0],
        'fy': info.k[4],
    }
    if s == 'depth':
        d = np.frombuffer(image.data, dtype='<u2')
        valid = d[d > 0]
        assert len(valid) > 0
        results[s].update(
            valid_fraction=float(len(valid) / len(d)),
            median_mm=float(np.median(valid)),
            min_mm=int(valid.min()),
            max_mm=int(valid.max()),
        )
    tf = buffer.lookup_transform('base_footprint', image.header.frame_id, rclpy.time.Time())
    results[s]['tf_translation'] = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
out = Path(__file__).resolve().parents[3] / 'data/diagnostics/camera'
out.mkdir(parents=True, exist_ok=True)
(out / 'ros_acceptance.json').write_text(json.dumps(results, indent=2))
print(json.dumps(results, indent=2))
node.destroy_node()
rclpy.shutdown()
