import math
import queue
import time
import uuid

import rclpy

from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
)
from nav_msgs.msg import (
    OccupancyGrid,
    Odometry,
)
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap
from std_srvs.srv import Empty

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from map_renderer import MapRenderer
from waypoint_storage import WaypointStorage


STATE_WAITING_INITIAL_POSE = "waiting_initial_pose"
STATE_STARTUP_LOCALIZATION = "startup_localization"
STATE_READY = "ready"
STATE_NAVIGATING = "navigating"


class ManagerNode(Node):

    def __init__(self, shared_state, command_queue, event_queue, result_dict, waypoints_path: str, map_image_path: str,):
        super().__init__("verter_manager")

        self.shared_state = shared_state
        self.command_queue = command_queue
        self.event_queue = event_queue
        self.result_dict = result_dict

        # =====================================================
        # Storage
        # =====================================================

        self.storage = WaypointStorage(waypoints_path)

        self.map_renderer = MapRenderer(map_image_path)

        self.map_image_path = map_image_path

        # =====================================================
        # Runtime state
        # =====================================================

        self.current_pose = None
        self.latest_map = None

        self.navigation_goal_handle = None

        # =====================================================
        # Localization thresholds
        # =====================================================

        self.localization_xy_std_max = 0.10
        self.localization_yaw_std_max = math.radians(6.0)

        # =====================================================
        # Motion detection
        # =====================================================

        self.last_motion_time = time.monotonic()

        self.motion_linear_threshold = 0.01
        self.motion_angular_threshold = 0.01

        # =====================================================
        # Startup localization movement
        # =====================================================

        self.startup_localization_active = False

        self.startup_localization_started_at = 0.0
        self.startup_direction_changed_at = 0.0

        self.startup_direction = 1

        self.startup_localization_duration = 30.0
        self.startup_direction_period = 3.0
        self.startup_angular_speed = 0.05

        # =====================================================
        # Publishers
        # =====================================================

        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # =====================================================
        # Subscribers
        # =====================================================

        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.initial_pose_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.amcl_pose_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback,10)

        # -----------------------------------------------------
        # Map QoS
        # -----------------------------------------------------

        map_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)

        # =====================================================
        # Services
        # =====================================================

        self.nomotion_client = self.create_client(Empty, "/request_nomotion_update")
        self.map_save_client = self.create_client(SaveMap, "/map_saver/save_map")

        # =====================================================
        # Nav2
        # =====================================================

        self.navigation_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        # =====================================================
        # Timers
        # =====================================================

        # Flask -> ROS
        self.create_timer(0.05, self.process_commands,)

        # Стартовые повороты
        self.create_timer(0.05, self.startup_localization_timer)

        # Каждые 5 сек проверяем,
        # стоит ли робот достаточно долго.
        self.create_timer(5.0, self.nomotion_timer_callback,)

        # =====================================================

        self.initialize_shared_state()
        self.initialize_start_position()

    # =========================================================
    # Shared state
    # =========================================================

    def initialize_shared_state(self):
        self.shared_state.update({
            "state": STATE_WAITING_INITIAL_POSE,
            "localized": False,
            "position": None,
            "localization": {
                "std_x": None,
                "std_y": None,
                "std_yaw": None,
                "std_yaw_deg": None,
            },
            "navigation": {
                "active": False,
                "waypoint_id": None,
            },
            "waypoints": self.storage.list(),
            "map": {
                "available": False,
                "width": 0,
                "height": 0,
                "resolution": 0.0,
                "origin_x": 0.0,
                "origin_y": 0.0,
            },
            "last_error": None
        })

    # =========================================================
    # Startup
    # =========================================================

    def initialize_start_position(self):
        base = self.storage.get_base()

        if base is None:
            self.shared_state["state"] = STATE_WAITING_INITIAL_POSE
            self.get_logger().warning("Base waypoint not found. Waiting for RViz 2D Pose Estimate.")
            self.publish_event("initial_pose_required", {})
            return

        self.get_logger().info(f"Base found: x={base['x']:.3f}, y={base['y']:.3f}, yaw={base['yaw']:.3f}")

        # Сначала меняем state, чтобы наш собственный
        # /initialpose не воспринимался как первый RViz pose.

        self.shared_state["state"] = STATE_STARTUP_LOCALIZATION
        self.publish_initial_pose(base["x"], base["y"], base["yaw"])
        self.start_startup_localization()

    # =========================================================
    # Initial pose
    # =========================================================

    def initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        # Нас интересует только первый initial pose,
        # когда база ещё отсутствует.

        if self.shared_state.get("state") != STATE_WAITING_INITIAL_POSE:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        self.get_logger().info(f"Initial pose received from RViz: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
        base = self.storage.get_base()

        if base is None:
            base = self.storage.create(waypoint_id="base", name="База", x=x, y=y, yaw=yaw, is_base=True)
            self.shared_state["waypoints"] = self.storage.list()
            self.publish_event("base_created", base,)

        self.start_startup_localization()

    # =========================================================
    # Startup localization movement
    # =========================================================

    def start_startup_localization(self):
        self.stop_robot()

        now = time.monotonic()
        self.startup_localization_active = True
        self.startup_localization_started_at = now
        self.startup_direction_changed_at = now
        self.startup_direction = 1

        self.shared_state["state"] = STATE_STARTUP_LOCALIZATION

        self.publish_event("startup_localization_started", {})

        self.get_logger().info("Startup localization movement started")

    # ---------------------------------------------------------

    def startup_localization_timer(self):
        if not self.startup_localization_active:
            return

        now = time.monotonic()
        elapsed = now - self.startup_localization_started_at

        # -----------------------------------------------------
        # Finish
        # -----------------------------------------------------

        if elapsed >= self.startup_localization_duration:
            self.finish_startup_localization()
            return

        # -----------------------------------------------------
        # Change direction
        # -----------------------------------------------------

        if (now - self.startup_direction_changed_at) >= self.startup_direction_period:
            self.startup_direction *= -1
            self.startup_direction_changed_at = now

        # -----------------------------------------------------
        # Turn
        # -----------------------------------------------------

        msg = Twist()
        msg.angular.z = self.startup_angular_speed * self.startup_direction
        self.cmd_vel_pub.publish(msg)

    # ---------------------------------------------------------

    def finish_startup_localization(self):
        self.startup_localization_active = False
        self.stop_robot()

        self.shared_state["state"] = STATE_READY
        self.publish_event("startup_localization_finished", {"localized": self.shared_state.get("localized", False)})

        self.get_logger().info("Startup localization movement finished")

    # =========================================================
    # AMCL pose
    # =========================================================

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):

        # -----------------------------------------------------
        # Current position
        # -----------------------------------------------------

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        self.current_pose = {
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw)
        }

        self.shared_state["position"] = self.current_pose

        # -----------------------------------------------------
        # Covariance
        # -----------------------------------------------------

        covariance = msg.pose.covariance
        std_x = math.sqrt(max(covariance[0], 0.0))
        std_y = math.sqrt(max(covariance[7], 0.0))
        std_yaw = math.sqrt(max(covariance[35], 0.0))

        self.shared_state["localization"] = {
            "std_x": float(std_x),
            "std_y": float(std_y),
            "std_yaw": float(std_yaw),
            "std_yaw_deg": float(math.degrees(std_yaw))}

        # -----------------------------------------------------
        # Localization status
        # -----------------------------------------------------

        localized = std_x <= self.localization_xy_std_max and std_y <= self.localization_xy_std_max and std_yaw <= self.localization_yaw_std_max
        previous_localized = self.shared_state.get("localized", False)

        # Пересчитываем НА КАЖДОМ /amcl_pose

        self.shared_state["localized"] = localized
        
        self.get_logger().info(f'LOC state: prev {previous_localized} | curr {localized} | 0: {round(covariance[0], 3)} | 7: {round(covariance[7], 3)} | 35: {round(covariance[35], 3)}')

        # -----------------------------------------------------
        # Event only on status change
        # -----------------------------------------------------

        if localized != previous_localized:
            self.get_logger().info(
                f"Localization status changed: "
                f"{previous_localized} -> {localized}. "
                f"x_std={std_x:.3f}, "
                f"y_std={std_y:.3f}, "
                f"yaw_std="
                f"{math.degrees(std_yaw):.2f} deg"
            )

            self.publish_event("localization_changed", {"localized": localized, "position": self.current_pose, "localization": self.shared_state["localization"]})

    # =========================================================
    # Odometry
    # =========================================================

    def odom_callback(self, msg: Odometry,):
        linear_x = abs(msg.twist.twist.linear.x)
        linear_y = abs(msg.twist.twist.linear.y)
        angular_z = abs(msg.twist.twist.angular.z)

        moving = (
            linear_x > self.motion_linear_threshold
            or
            linear_y > self.motion_linear_threshold
            or
            angular_z > self.motion_angular_threshold
        )

        if moving:
            self.last_motion_time = time.monotonic()

    # =========================================================
    # AMCL no-motion update
    # =========================================================

    def nomotion_timer_callback(self):
        # Таймер сам запускается раз в 5 секунд.
        #
        # Если с последнего движения прошло < 5 секунд,
        # ничего не делаем.

        if (time.monotonic() - self.last_motion_time) < 5.0:
            return

        # Во время стартовых поворотов robot физически
        # должен двигаться, поэтому no-motion не вызываем.

        if self.startup_localization_active:
            return

        if not self.nomotion_client.service_is_ready():
            self.get_logger().warning("/request_nomotion_update " "service unavailable")
            return

        future = self.nomotion_client.call_async(Empty.Request())

        future.add_done_callback(self.nomotion_update_callback)

    # ---------------------------------------------------------

    def nomotion_update_callback(self, future,):
        try:
            future.result()
            self.get_logger().debug("AMCL no-motion update requested")
        except Exception as exception:
            self.get_logger().warning("request_nomotion_update failed: " f"{exception}")

    # =========================================================
    # Map
    # =========================================================

    def map_callback(self, msg: OccupancyGrid,):
        self.latest_map = msg
        self.shared_state["map"] = {
            "available": True,
            "width":int(msg.info.width),
            "height":int(msg.info.height),
            "resolution":float(msg.info.resolution),
            "origin_x":float(msg.info.origin.position.x),
            "origin_y":float(msg.info.origin.position.y),
            "origin_yaw":float(self.quaternion_to_yaw(msg.info.origin.orientation))}

        try:
            self.map_renderer.update(msg)
        except Exception as exception:
            self.get_logger().error(f"Map rendering failed: {exception}")

    # =========================================================
    # Flask command processing
    # =========================================================

    def process_commands(self):
        for _ in range(20):
            try:
                request = (self.command_queue.get_nowait())
            except queue.Empty:
                break

            request_id = request["request_id"]
            command = request["command"]

            payload = request.get("payload", {})

            try:
                result = self.execute_command(command, payload)

                self.result_dict[request_id] = {
                    "success": True,
                    "result": result
                }
            except Exception as exception:
                error = f"{type(exception).__name__}: {exception}"
                self.get_logger().error(f"{command}: {error}")
                self.result_dict[request_id] = {
                    "success": False,
                    "error": error
                }

    # ---------------------------------------------------------

    def execute_command(self, command: str, payload: dict):
        match command:
            case "waypoint_create":
                return self.command_waypoint_create(payload)
            case  "waypoint_update":
                return self.command_waypoint_update(payload)
            case  "waypoint_delete":
                return self.command_waypoint_delete(payload)
            case  "waypoint_goal":
                return self.command_waypoint_goal(payload)
            case  "map_save":
                return self.command_map_save(payload)
            case  "localization_restart":
                return self.command_localization_restart()

        raise ValueError(f"Unknown command: {command}")

    # =========================================================
    # Waypoint create
    # =========================================================

    def command_waypoint_create(self, payload: dict):
        waypoint_id = payload["id"]
        name = payload.get("name", waypoint_id)
        is_base = payload.get("is_base", False)

        # -----------------------------------------------------
        # Explicit pose
        # -----------------------------------------------------

        if all(field in payload for field in ("x", "y", "yaw")):
            pose = {
                "x": float(payload["x"]),
                "y": float(payload["y"]),
                "yaw": float(payload["yaw"])
            }
        else:
            if self.current_pose is None:
                raise RuntimeError("Current robot position is unavailable")

            if not self.shared_state.get("localized", False):
                raise RuntimeError("Cannot save current position: robot is not localized")

            pose = dict(self.current_pose)

        waypoint = self.storage.create(waypoint_id=waypoint_id, name=name, x=pose["x"], y=pose["y"], yaw=pose["yaw"], is_base=is_base)
        self.shared_state["waypoints"] = self.storage.list()
        self.publish_event("waypoint_created", waypoint)

        return waypoint

    # =========================================================
    # Waypoint update
    # =========================================================

    def command_waypoint_update(self, payload: dict):
        waypoint_id = payload["id"]
        values = {}

        for field in ("name", "x", "y", "yaw", "is_base"):
            if field in payload:
                values[field] = payload[field]

        if payload.get("use_current_position", False):
            if self.current_pose is None:
                raise RuntimeError("Current robot position is unavailable")

            if not self.shared_state.get("localized", False):
                raise RuntimeError("Cannot use current position: robot is not localized")

            values.update(self.current_pose)

        waypoint = self.storage.update(waypoint_id, values)
        self.shared_state["waypoints"] = self.storage.list()
        self.publish_event("waypoint_updated", waypoint)

        return waypoint

    # =========================================================
    # Waypoint delete
    # =========================================================

    def command_waypoint_delete(self, payload: dict):
        waypoint_id = payload["id"]
        self.storage.delete(waypoint_id)
        self.shared_state["waypoints"] = self.storage.list()
        self.publish_event("waypoint_deleted", {"id": waypoint_id})

        return {"id": waypoint_id}

    # =========================================================
    # Navigation goal
    # =========================================================

    def command_waypoint_goal(self, payload: dict):
        # Главное ограничение.

        if not self.shared_state.get("localized", False):
            raise RuntimeError("Robot is not localized")

        if self.startup_localization_active:
            raise RuntimeError("Startup localization movement is still running")

        waypoint_id = payload["id"]
        waypoint = self.storage.get(waypoint_id)

        if waypoint is None:
            raise KeyError(f"Waypoint '{waypoint_id}' not found")

        if not self.navigation_client.server_is_ready():
            if not self.navigation_client.wait_for_server(timeout_sec=1.0):
                raise RuntimeError("NavigateToPose action is unavailable")

        goal = NavigateToPose.Goal()
        goal.pose = self.create_pose_stamped(waypoint["x"], waypoint["y"], waypoint["yaw"])
        future = self.navigation_client.send_goal_async(goal)
        future.add_done_callback(lambda future: self.navigation_goal_response(future, waypoint_id))

        return {"accepted": True, "waypoint_id": waypoint_id}

    # =========================================================
    # Localization restart
    # =========================================================
    
    def command_localization_restart(self):
        base = self.storage.get_base()

        if base is None:
            raise RuntimeError("Base waypoint is not configured")

        self.get_logger().warning("Restarting localization from base")

        # -------------------------------------------------
        # Если Nav2 сейчас куда-то едет — отменяем goal
        # -------------------------------------------------

        if self.navigation_goal_handle is not None:
            try:
                self.navigation_goal_handle.cancel_goal_async()
            except Exception as exception:
                self.get_logger().warning(f"Cannot cancel navigation goal: {exception}")

            self.navigation_goal_handle = None
            self.shared_state["navigation"] = {"active": False, "waypoint_id": None}

        # -------------------------------------------------
        # Останавливаем робот
        # -------------------------------------------------

        self.stop_robot()

        # -------------------------------------------------
        # До нового /amcl_pose считаем локализацию
        # неизвестной / плохой
        # -------------------------------------------------

        self.shared_state["localized"] = False
        self.shared_state["localization"] = {
            "std_x": None,
            "std_y": None,
            "std_yaw": None,
            "std_yaw_deg": None
        }

        # -------------------------------------------------
        # Устанавливаем base как initial pose
        # -------------------------------------------------

        self.publish_initial_pose(base["x"], base["y"], base["yaw"])

        # -------------------------------------------------
        # Снова запускаем стандартные стартовые повороты
        # -------------------------------------------------

        self.start_startup_localization()

        self.publish_event("localization_restart", {"base": {"x": base["x"], "y": base["y"], "yaw": base["yaw"]}})

        return {"accepted": True, "base": {"x": base["x"], "y": base["y"], "yaw": base["yaw"]}}

    # ---------------------------------------------------------

    def navigation_goal_response(self, future, waypoint_id):
        try:
            goal_handle = future.result()
        except Exception as exception:
            self.get_logger().error("Navigation goal error: {exception}")

            return

        if not goal_handle.accepted:
            self.publish_event("navigation_rejected", {"waypoint_id": waypoint_id})
            return

        self.navigation_goal_handle = goal_handle
        self.shared_state["state"] = STATE_NAVIGATING
        self.shared_state["navigation"] = {
            "active": True,
            "waypoint_id": waypoint_id
        }

        self.publish_event("navigation_started", {"waypoint_id": waypoint_id})

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda future: self.navigation_finished(future, waypoint_id))

    # ---------------------------------------------------------

    def navigation_finished(self, future, waypoint_id):
        try:
            result = future.result()
            status = result.status
        except Exception as exception:
            status = None
            self.get_logger().error(f"Navigation result error: {exception}")

        self.navigation_goal_handle = None
        self.shared_state["navigation"] = {
            "active": False,
            "waypoint_id": None
        }

        self.shared_state["state"] = STATE_READY
        self.publish_event("navigation_finished", {"waypoint_id": waypoint_id, "status": status,})

    # =========================================================
    # Map save
    # =========================================================

    def command_map_save(self, payload: dict):
        map_url = payload.get("path", "/home/jetson/verter-data/maps/current_map")

        if not self.map_save_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("Map saver service unavailable")

        request = SaveMap.Request()

        request.map_topic = "/map"
        request.map_url = map_url
        request.image_format = "png"
        request.map_mode = "trinary"
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65

        future = (self.map_save_client.call_async(request))
        future.add_done_callback(lambda future: self.map_save_finished(future, map_url))

        return {"accepted": True, "path": map_url}

    # ---------------------------------------------------------

    def map_save_finished(self, future, path):
        try:
            response = future.result()
            self.publish_event("map_saved", {"path": path, "result": bool(response.result)})
        except Exception as exception:
            self.publish_event("map_save_failed", {"error": str(exception)})

    # =========================================================
    # Pose helpers
    # =========================================================

    def create_pose_stamped(self, x: float, y: float, yaw: float):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin( yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    # ---------------------------------------------------------

    def publish_initial_pose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Initial covariance

        msg.pose.covariance[0] = 0.25 ** 2
        msg.pose.covariance[7] = 0.25 ** 2
        msg.pose.covariance[35] = math.radians(15.0) ** 2

        self.initial_pose_pub.publish(msg)

    # ---------------------------------------------------------

    @staticmethod
    def quaternion_to_yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # =========================================================
    # Stop
    # =========================================================

    def stop_robot(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    # =========================================================
    # SSE events
    # =========================================================

    def publish_event(self, name: str, data: dict):
        try:
            self.event_queue.put_nowait({"id": uuid.uuid4().hex, "event": name, "data": data, "timestamp": time.time()})
        except Exception as exception:
            self.get_logger().warning(f"Cannot publish event: {exception}")