/**
 * ROS type definitions
 * Types for roslib.js and custom ROS messages
 */

// ============ roslib.js types ============

export interface RosbridgeOptions {
  url?: string;
  transport?: 'websocket' | 'socket.io' | 'tcp';
}

export interface RosMessage {
  name: string;
  msg: RosMessageData;
}

export interface RosMessageData {
  [key: string]: unknown;
}

export interface RosServiceRequest {
  [key: string]: unknown;
}

export interface RosServiceResponse {
  [key: string]: unknown;
}

// ============ ROS Message Types ============

/**
 * geometry_msgs/msg/Twist
 * Linear and angular velocity
 */
export interface Twist {
  linear: {
    x: number;
    y: number;
    z: number;
  };
  angular: {
    x: number;
    y: number;
    z: number;
  };
}

/**
 * geometry_msgs/msg/Pose
 * Position and orientation
 */
export interface Pose {
  position: {
    x: number;
    y: number;
    z: number;
  };
  orientation: {
    x: number;
    y: number;
    z: number;
    w: number;
  };
}

/**
 * geometry_msgs/msg/PoseWithCovarianceStamped
 * Pose with covariance and timestamp (from AMCL)
 */
export interface PoseWithCovarianceStamped {
  header: {
    stamp: {
      sec: number;
      nanosec: number;
    };
    frame_id: string;
  };
  pose: {
    pose: Pose;
    covariance: number[];
  };
}

/**
 * nav2_msgs/action/NavigateToPose
 * Navigation action goal
 */
export interface NavigateToPoseGoal {
  pose: {
    header: {
      stamp: {
        sec: number;
        nanosec: number;
      };
      frame_id: string;
    };
    pose: Pose;
  };
}

// ============ Custom Service Types ============

/**
 * verter_admin_msgs/srv/SaveWaypoint
 */
export interface SaveWaypointRequest {
  name: string;
  x: number;
  y: number;
  theta: number;
}

export interface SaveWaypointResponse {
  success: boolean;
  message: string;
}

/**
 * verter_admin_msgs/srv/NavigateToWaypoint
 */
export interface NavigateToWaypointRequest {
  name: string;
}

export interface NavigateToWaypointResponse {
  success: boolean;
  message: string;
}

/**
 * verter_admin_msgs/srv/DeleteWaypoint
 */
export interface DeleteWaypointRequest {
  name: string;
}

export interface DeleteWaypointResponse {
  success: boolean;
  message: string;
}

/**
 * verter_admin_msgs/srv/ListWaypoints
 */
export interface ListWaypointsRequest {
  // Empty request
}

export interface ListWaypointsResponse {
  names: string[];
  x: number[];
  y: number[];
  theta: number[];
}

/**
 * std_srvs/srv/Trigger
 */
export interface TriggerRequest {
  // Empty request
}

export interface TriggerResponse {
  success: boolean;
  message: string;
}

// ============ Domain Types ============

/**
 * Waypoint - saved location
 */
export interface Waypoint {
  name: string;
  x: number;
  y: number;
  theta: number;
  description?: string;
}

/**
 * Robot pose in 2D (x, y, yaw)
 */
export interface RobotPose2D {
  x: number;
  y: number;
  yaw: number; // in radians
}

/**
 * Navigation status
 */
export type NavigationStatus =
  | 'idle'
  | 'planning'
  | 'navigating'
  | 'arrived'
  | 'failed'
  | 'cancelled';

/**
 * ROS connection status
 */
export type RosStatus = 'disconnected' | 'connecting' | 'connected' | 'error';

/**
 * App mode - setup or active
 */
export type AppMode = 'setup' | 'active';
