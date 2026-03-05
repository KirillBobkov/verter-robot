"""WaypointManagerNode — LifecycleNode for waypoint CRUD and autonomous patrol.

Safety invariants enforced here:
  SI-5: All 6 service callbacks return success=False if called before on_activate().
  SI-6: PatrolPolicy governs PATROLLING → DEGRADED → FAULT (delegated to domain).
  C1:   NavigateToPose action client is the ONLY actuator interface. No direct velocity publishing.
  §6:   /amcl_pose QoS: TRANSIENT_LOCAL, RELIABLE, depth=5 (C3/U7).
        /waypoints/markers QoS: TRANSIENT_LOCAL, RELIABLE, depth=1.

Lifecycle resource allocation:
  on_configure : create action client, publisher, subscriber, all 6 services, load YAML.
  on_activate  : set _active=True, publish initial markers.
  on_deactivate: set _active=False, stop patrol, cancel Nav2 goals.
  on_cleanup   : destroy all ROS resources, reset domain objects.
"""
from __future__ import annotations

import math
import os
import threading
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from verter_admin_msgs.srv import (
    DeleteWaypoint,
    ListWaypoints,
    NavigateToWaypoint,
    SaveWaypoint,
)

from verter_admin.waypoints.application.navigate_to import ActionClientPort, NavigateToUseCase
from verter_admin.waypoints.application.patrol import PatrolUseCase
from verter_admin.waypoints.application.save_waypoint import SaveWaypointUseCase
from verter_admin.waypoints.domain.patrol_policy import PatrolPolicy, PatrolState
from verter_admin.waypoints.domain.waypoint_store import Waypoint, WaypointStore
from verter_admin.waypoints.infrastructure.yaml_store import YamlStore

# ---------------------------------------------------------------------------
# QoS profiles (§6 / Design C3, U7)
# ---------------------------------------------------------------------------
_QOS_TRANSIENT_RELIABLE_5 = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=5,
)
_QOS_TRANSIENT_RELIABLE_1 = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)


def _yaw_from_quaternion(q) -> float:
    """Convert a geometry_msgs Quaternion to yaw in radians (A6)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ---------------------------------------------------------------------------
# ActionClientPort adapter for Nav2 NavigateToPose (C1)
# ---------------------------------------------------------------------------

class _Nav2ActionClientPort(ActionClientPort):
    """Wraps rclpy ActionClient for NavigateToPose.

    send_goal() blocks the calling thread (patrol loop daemon thread) using
    threading.Events. The ROS executor running on the main thread processes
    the callbacks that set those events.
    """

    def __init__(self, action_client: ActionClient, logger) -> None:
        self._client = action_client
        self._logger = logger
        self._goal_handle = None
        self._lock = threading.Lock()

    def send_goal(self, waypoint: Waypoint) -> bool:  # called from daemon thread
        if not self._client.wait_for_server(timeout_sec=5.0):
            self._logger.warning('NavigateToPose action server not available (timeout=5s)')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = waypoint.x
        goal.pose.pose.position.y = waypoint.y
        # Yaw-only quaternion
        goal.pose.pose.orientation.z = math.sin(waypoint.theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(waypoint.theta / 2.0)

        accepted_event = threading.Event()
        result_event = threading.Event()
        success = [False]

        def _goal_response_cb(future):
            handle = future.result()
            if not handle.accepted:
                accepted_event.set()
                return
            with self._lock:
                self._goal_handle = handle
            accepted_event.set()
            handle.get_result_async().add_done_callback(_result_cb)

        def _result_cb(future):
            with self._lock:
                self._goal_handle = None
            try:
                result = future.result()
                success[0] = (result.status == GoalStatus.STATUS_SUCCEEDED)
            except Exception as e:
                self._logger.warning(f'Nav2 result error: {e}')
                success[0] = False
            result_event.set()

        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(_goal_response_cb)

        if not accepted_event.wait(timeout=10.0):
            self._logger.warning('Nav2 goal acceptance timed out (10s)')
            return False
        if not result_event.wait(timeout=120.0):
            self._logger.warning('Nav2 goal result timed out (120s)')
            return False
        return success[0]

    def cancel_goal(self) -> None:
        with self._lock:
            handle = self._goal_handle
        if handle is not None:
            handle.cancel_goal_async()


# ---------------------------------------------------------------------------
# WaypointManagerNode
# ---------------------------------------------------------------------------

class WaypointManagerNode(LifecycleNode):
    """LifecycleNode managing waypoint CRUD and autonomous patrol via Nav2.

    Lifecycle contract:
      UNCONFIGURED → configure → INACTIVE : load YAML, create all ROS resources.
      INACTIVE → activate → ACTIVE        : enable service processing.
      ACTIVE → deactivate → INACTIVE      : disable processing, stop patrol, cancel goals.
      INACTIVE → cleanup → UNCONFIGURED   : destroy all ROS resources, reset domain.
    """

    def __init__(self) -> None:
        super().__init__('waypoint_manager')
        self.declare_parameter(
            'waypoints_file',
            os.path.expanduser('~/verter-robot/verter_admin/waypoints.yaml'),
        )
        self.declare_parameter('nav2_action_server', 'navigate_to_pose')
        self.declare_parameter('patrol_warn_threshold', 3)
        self.declare_parameter('patrol_fault_threshold', 5)

        # Domain objects (set in on_configure, cleared in on_cleanup)
        self._store: Optional[WaypointStore] = None
        self._policy: Optional[PatrolPolicy] = None
        self._yaml_store: Optional[YamlStore] = None

        # ROS resources (set in on_configure, destroyed in on_cleanup)
        self._action_client: Optional[ActionClient] = None
        self._action_port: Optional[_Nav2ActionClientPort] = None
        self._patrol_use_case: Optional[PatrolUseCase] = None
        self._marker_pub = None
        self._amcl_sub = None
        self._services: List = []

        # SI-5: gate controlling whether service callbacks process requests
        self._active: bool = False

        # Patrol loop state
        self._patrol_running: bool = False
        self._patrol_thread: Optional[threading.Thread] = None

        self._current_pose: Optional[Waypoint] = None

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('WaypointManagerNode: configuring')

        waypoints_file = self.get_parameter('waypoints_file').value
        warn = self.get_parameter('patrol_warn_threshold').value
        fault = self.get_parameter('patrol_fault_threshold').value
        action_server = self.get_parameter('nav2_action_server').value

        # Domain
        self._store = WaypointStore()
        self._yaml_store = YamlStore(waypoints_file)
        self._policy = PatrolPolicy(warn_threshold=warn, fault_threshold=fault)

        # Load persisted waypoints
        data = self._yaml_store.load()
        if data:
            self._store.load_from_dict(data)
            self.get_logger().info(
                f'Loaded {len(self._store.list_all())} waypoints from {waypoints_file}'
            )

        # Publisher: /waypoints/markers — TRANSIENT_LOCAL, RELIABLE, depth=1
        self._marker_pub = self.create_publisher(
            MarkerArray, '/waypoints/markers', _QOS_TRANSIENT_RELIABLE_1
        )

        # Subscriber: /amcl_pose — TRANSIENT_LOCAL, RELIABLE, depth=5 (C3/U7)
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose_cb,
            _QOS_TRANSIENT_RELIABLE_5,
        )

        # 6 services (created here so they are accessible in INACTIVE state;
        # SI-5 guard inside each callback ensures requests are rejected until active)
        self._services = [
            self.create_service(SaveWaypoint, '/save_waypoint', self._save_waypoint_cb),
            self.create_service(
                NavigateToWaypoint, '/navigate_to_waypoint', self._navigate_to_waypoint_cb
            ),
            self.create_service(DeleteWaypoint, '/delete_waypoint', self._delete_waypoint_cb),
            self.create_service(ListWaypoints, '/list_waypoints', self._list_waypoints_cb),
            self.create_service(Trigger, '/start_patrol', self._start_patrol_cb),
            self.create_service(Trigger, '/stop_patrol', self._stop_patrol_cb),
        ]

        self._active = False
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('WaypointManagerNode: activating')
        action_server = self.get_parameter('nav2_action_server').value
        self._action_client = ActionClient(self, NavigateToPose, action_server)
        self._action_port = _Nav2ActionClientPort(self._action_client, self.get_logger())
        self._patrol_use_case = PatrolUseCase(
            store=self._store,
            policy=self._policy,
            action_client=self._action_port,
        )
        self._active = True
        self._publish_markers()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('WaypointManagerNode: deactivating')
        self._active = False
        self._stop_patrol_internal()
        if self._action_port is not None:
            self._action_port.cancel_goal()
        if self._action_client is not None:
            self._action_client.destroy()
            self._action_client = None
        self._action_port = None
        self._patrol_use_case = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('WaypointManagerNode: cleanup')
        for srv in self._services:
            self.destroy_service(srv)
        self._services = []
        if self._marker_pub is not None:
            self.destroy_publisher(self._marker_pub)
            self._marker_pub = None
        if self._amcl_sub is not None:
            self.destroy_subscription(self._amcl_sub)
            self._amcl_sub = None
        if self._action_client is not None:
            self._action_client.destroy()
            self._action_client = None
        # Reset domain objects (R7: on_cleanup fully resets PatrolPolicy)
        self._store = None
        self._policy = None
        self._yaml_store = None
        self._patrol_use_case = None
        self._action_port = None
        self._current_pose = None
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Service callbacks — SI-5 guard applied in every callback
    # ------------------------------------------------------------------

    def _guard(self, response) -> bool:
        """Set response.success=False and return True if node is not active (SI-5)."""
        if not self._active:
            response.success = False
            response.message = 'Node not active'
            return True
        return False

    def _save_waypoint_cb(self, request, response):
        if self._guard(response):
            return response
        try:
            use_case = SaveWaypointUseCase(store=self._store, port=self._yaml_store)
            use_case.execute(request.name, request.x, request.y, request.theta)
            self._publish_markers()
            response.success = True
            response.message = f"Waypoint '{request.name}' saved"
        except ValueError as e:
            response.success = False
            response.message = str(e)
        return response

    def _navigate_to_waypoint_cb(self, request, response):
        if self._guard(response):
            return response
        try:
            use_case = NavigateToUseCase(store=self._store, action_client=self._action_port)
            accepted = use_case.execute(request.name)
            response.success = accepted
            response.message = 'Goal accepted' if accepted else 'Goal rejected by Nav2'
        except KeyError as e:
            response.success = False
            response.message = str(e)
        return response

    def _delete_waypoint_cb(self, request, response):
        if self._guard(response):
            return response
        try:
            self._store.delete(request.name)
            self._yaml_store.save(self._store.to_dict())
            self._publish_markers()
            response.success = True
            response.message = f"Waypoint '{request.name}' deleted"
        except KeyError as e:
            response.success = False
            response.message = str(e)
        return response

    def _list_waypoints_cb(self, request, response):
        # ListWaypoints has no success/message fields; return empty lists if not active
        if not self._active:
            return response
        waypoints = self._store.list_all()
        response.names = [wp.name for wp in waypoints]
        response.x = [wp.x for wp in waypoints]
        response.y = [wp.y for wp in waypoints]
        response.theta = [wp.theta for wp in waypoints]
        return response

    def _start_patrol_cb(self, request, response):
        if self._guard(response):
            return response
        if self._patrol_running:
            response.success = False
            response.message = 'Patrol already running'
            return response
        self._start_patrol_internal()
        response.success = True
        response.message = 'Patrol started'
        return response

    def _stop_patrol_cb(self, request, response):
        if self._guard(response):
            return response
        self._stop_patrol_internal()
        response.success = True
        response.message = 'Patrol stopped'
        return response

    # ------------------------------------------------------------------
    # Patrol loop management
    # ------------------------------------------------------------------

    def _start_patrol_internal(self) -> None:
        self._patrol_running = True
        self._patrol_use_case.start()
        self._patrol_thread = threading.Thread(
            target=self._patrol_loop, daemon=True, name='patrol_loop'
        )
        self._patrol_thread.start()

    def _stop_patrol_internal(self) -> None:
        self._patrol_running = False
        if self._patrol_use_case is not None:
            self._patrol_use_case.stop()
        if self._patrol_thread is not None:
            self._patrol_thread.join(timeout=5.0)
            self._patrol_thread = None

    def _patrol_loop(self) -> None:
        while self._patrol_running and rclpy.ok():
            state = self._patrol_use_case.step()
            if state == PatrolState.FAULT:
                self.get_logger().error(
                    'PatrolPolicy entered FAULT state — patrol halted. '
                    'Recovery: deactivate → cleanup → configure → activate.'
                )
                self._patrol_running = False
                break
            if state == PatrolState.DEGRADED:
                self.get_logger().warning('PatrolPolicy in DEGRADED state')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        """Cache latest robot pose for optional use by UI (A6)."""
        theta = _yaw_from_quaternion(msg.pose.pose.orientation)
        self._current_pose = Waypoint(
            name='__current__',
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            theta=theta,
        )

    def _publish_markers(self) -> None:
        """Publish current waypoints as MarkerArray on /waypoints/markers."""
        if self._marker_pub is None or self._store is None:
            return
        array = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, wp in enumerate(self._store.list_all()):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wp.x
            m.pose.position.y = wp.y
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2
            m.color.r = 0.0
            m.color.g = 0.8
            m.color.b = 0.0
            m.color.a = 1.0
            array.markers.append(m)
        self._marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManagerNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
