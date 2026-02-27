"""E2E integration tests for waypoint navigation.

Requires a ROS2 Humble environment with Nav2 accessible.
Run with: pytest tests/integration/test_e2e_waypoint_navigation.py -v

Tests:
  test_save_navigate_succeed  — save waypoint → navigate → goal succeeds.
  test_si9_web_server_crash   — SI-9: SIGKILL web_server_node does not stop patrol.

These tests launch sub-processes and require:
  - ros2 sourced in the shell
  - Nav2 action server running (or the mock server from this file used standalone)
  - rosbridge_server installed (ros-humble-rosbridge-suite)

Note: These are system-level tests intended to run on the Jetson or CI with full stack.
      On a dev machine without ROS, this module is skipped automatically.
"""
from __future__ import annotations

import os
import signal
import subprocess
import time

import pytest

# Guard: skip entire module if rclpy is not available
pytest.importorskip('rclpy', reason='rclpy not available — ROS2 environment required')

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
import threading


# ---------------------------------------------------------------------------
# Shared ROS context
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


# ---------------------------------------------------------------------------
# Helper: spin nodes in background
# ---------------------------------------------------------------------------

class _Spinner:
    def __init__(self, *nodes):
        self._exe = MultiThreadedExecutor()
        for n in nodes:
            self._exe.add_node(n)
        self._t = threading.Thread(target=self._exe.spin, daemon=True)
        self._t.start()

    def add(self, node):
        self._exe.add_node(node)

    def stop(self):
        self._exe.shutdown(timeout_sec=2.0)


# ---------------------------------------------------------------------------
# Test: save waypoint → navigate → succeed (basic E2E flow)
# ---------------------------------------------------------------------------

def test_save_navigate_succeed(ros_context, tmp_path):
    """E2E: save a waypoint, navigate to it, confirm success response."""
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionServer
    from verter_admin.waypoints.adapters.ros.waypoint_manager_node import WaypointManagerNode
    from verter_admin_msgs.srv import SaveWaypoint, NavigateToWaypoint

    goal_received = threading.Event()
    result_future_holder = [None]

    def _execute_cb(goal_handle):
        goal_received.set()
        goal_handle.succeed()
        return NavigateToPose.Result()

    mock_node = rclpy.create_node('e2e_mock_nav2')
    ActionServer(mock_node, NavigateToPose, 'navigate_to_pose_e2e', _execute_cb)

    wm_node = WaypointManagerNode()
    wm_node.set_parameters([
        Parameter('waypoints_file', value=str(tmp_path / 'e2e_waypoints.yaml')),
        Parameter('nav2_action_server', value='navigate_to_pose_e2e'),
    ])

    spinner = _Spinner(wm_node, mock_node)
    client_node = rclpy.create_node('e2e_client')
    spinner.add(client_node)

    try:
        wm_node.trigger_configure()
        time.sleep(0.2)
        wm_node.trigger_activate()
        time.sleep(0.2)

        save_client = client_node.create_client(SaveWaypoint, '/save_waypoint')
        nav_client = client_node.create_client(NavigateToWaypoint, '/navigate_to_waypoint')
        assert save_client.wait_for_service(timeout_sec=3.0)
        assert nav_client.wait_for_service(timeout_sec=3.0)

        def _sync_call(client, req, timeout=5.0):
            f = client.call_async(req)
            end = time.time() + timeout
            while not f.done() and time.time() < end:
                time.sleep(0.05)
            assert f.done(), 'Service call timed out'
            return f.result()

        # Save
        save_req = SaveWaypoint.Request()
        save_req.name = 'e2e_point'
        save_req.x = 1.0
        save_req.y = 0.5
        save_req.theta = 0.0
        save_res = _sync_call(save_client, save_req)
        assert save_res.success, f"save failed: {save_res.message}"

        # Navigate
        nav_req = NavigateToWaypoint.Request()
        nav_req.name = 'e2e_point'
        nav_res = _sync_call(nav_client, nav_req, timeout=15.0)
        assert nav_res.success, f"navigate failed: {nav_res.message}"
        assert goal_received.is_set(), 'Mock Nav2 server never received goal'
    finally:
        wm_node.destroy_node()
        mock_node.destroy_node()
        client_node.destroy_node()
        spinner.stop()


# ---------------------------------------------------------------------------
# Test: SI-9 — web_server_node crash does not stop patrol
# ---------------------------------------------------------------------------

def test_si9_web_server_crash(ros_context, tmp_path):
    """SI-9: SIGKILL of web_server_node does not interrupt the patrol loop.

    Verifies that patrol continues through >= 2 waypoints after web_server_node is killed.
    The waypoint_manager is a separate process from the web server (SI-9 HMI isolation).
    """
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionServer
    from verter_admin.waypoints.adapters.ros.waypoint_manager_node import WaypointManagerNode
    from verter_admin_msgs.srv import SaveWaypoint
    from std_srvs.srv import Trigger

    goals_completed = [0]
    goals_lock = threading.Lock()

    def _execute_cb(goal_handle):
        goal_handle.succeed()
        with goals_lock:
            goals_completed[0] += 1
        return NavigateToPose.Result()

    mock_node = rclpy.create_node('si9_mock_nav2')
    ActionServer(mock_node, NavigateToPose, 'navigate_to_pose_si9', _execute_cb)

    wm_node = WaypointManagerNode()
    wm_node.set_parameters([
        Parameter('waypoints_file', value=str(tmp_path / 'si9_waypoints.yaml')),
        Parameter('nav2_action_server', value='navigate_to_pose_si9'),
        Parameter('patrol_warn_threshold', value=10),
        Parameter('patrol_fault_threshold', value=20),
    ])

    spinner = _Spinner(wm_node, mock_node)
    client_node = rclpy.create_node('si9_client')
    spinner.add(client_node)

    try:
        wm_node.trigger_configure()
        time.sleep(0.2)
        wm_node.trigger_activate()
        time.sleep(0.2)

        save_client = client_node.create_client(SaveWaypoint, '/save_waypoint')
        patrol_client = client_node.create_client(Trigger, '/start_patrol')
        assert save_client.wait_for_service(timeout_sec=3.0)
        assert patrol_client.wait_for_service(timeout_sec=3.0)

        def _sync_call(client, req, timeout=5.0):
            f = client.call_async(req)
            end = time.time() + timeout
            while not f.done() and time.time() < end:
                time.sleep(0.05)
            assert f.done()
            return f.result()

        # Save two waypoints
        for i, (x, y) in enumerate([(1.0, 0.0), (2.0, 0.0)]):
            req = SaveWaypoint.Request()
            req.name = f'si9_wp{i}'
            req.x = x
            req.y = y
            req.theta = 0.0
            res = _sync_call(save_client, req)
            assert res.success

        # Start patrol
        patrol_res = _sync_call(patrol_client, Trigger.Request())
        assert patrol_res.success, f"start_patrol failed: {patrol_res.message}"

        # Simulate web_server_node crash by spawning and killing a subprocess.
        # Since WebServerNode is a plain Node (SI-9), its crash does not affect WaypointManager.
        web_proc = subprocess.Popen(
            ['python3', '-c',
             'import rclpy; rclpy.init(); '
             'from verter_admin.web.adapters.ros.web_server_node import WebServerNode; '
             'rclpy.spin(WebServerNode())'],
            env={**os.environ},
        )
        time.sleep(0.5)
        web_proc.send_signal(signal.SIGKILL)
        web_proc.wait(timeout=3.0)

        # Patrol should continue: wait for >= 2 more goals to complete
        goals_before_kill = goals_completed[0]
        deadline = time.time() + 15.0
        while time.time() < deadline:
            with goals_lock:
                delta = goals_completed[0] - goals_before_kill
            if delta >= 2:
                break
            time.sleep(0.1)

        with goals_lock:
            delta = goals_completed[0] - goals_before_kill
        assert delta >= 2, (
            f'SI-9 FAIL: patrol completed only {delta} goals after web_server_node SIGKILL '
            f'(expected >= 2). Patrol was disrupted by web server crash.'
        )
    finally:
        wm_node.destroy_node()
        mock_node.destroy_node()
        client_node.destroy_node()
        spinner.stop()
