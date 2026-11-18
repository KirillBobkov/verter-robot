"""One explicit AMCL initial-pose publication, with lifecycle and session guards."""

import math
import time


def publish_initial_pose(pose, guard):
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from lifecycle_msgs.srv import GetState
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    guard()
    context = Context()
    rclpy.init(context=context)
    node = None
    executor = None
    try:
        node = rclpy.create_node(
            'verter_initial_pose_publisher',
            context=context,
            enable_rosout=False,
            start_parameter_services=False,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        publisher = node.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE),
        )
        client = node.create_client(GetState, '/amcl/get_state')
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            guard()
            if client.service_is_ready() and publisher.get_subscription_count() > 0:
                break
            executor.spin_once(timeout_sec=0.05)
        else:
            raise RuntimeError('active_amcl_subscriber_unavailable')
        future = client.call_async(GetState.Request())
        executor.spin_until_future_complete(future, timeout_sec=max(0, deadline - time.monotonic()))
        if not future.done() or future.result() is None or future.result().current_state.id != 3:
            raise RuntimeError('amcl_not_active')
        guard()
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        now = node.get_clock().now()
        message.header.stamp = now.to_msg()
        message.pose.pose.position.x = pose['x']
        message.pose.pose.position.y = pose['y']
        message.pose.pose.orientation.z = math.sin(pose['yaw'] / 2)
        message.pose.pose.orientation.w = math.cos(pose['yaw'] / 2)
        message.pose.covariance[0] = pose['std_xy'] ** 2
        message.pose.covariance[7] = pose['std_xy'] ** 2
        message.pose.covariance[35] = pose['std_yaw'] ** 2
        # A volatile publisher sends once; no retry queue or latched reset for a later AMCL.
        guard()
        publisher.publish(message)
        executor.spin_once(timeout_sec=0.1)
        return {'published_at': now.nanoseconds / 1e9}
    finally:
        if executor:
            executor.shutdown()
        if node:
            node.destroy_node()
        context.shutdown()
