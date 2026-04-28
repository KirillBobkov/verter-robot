/**
 * ROS bridge service
 * Abstraction layer over roslib.js
 */

import type {
  RosStatus,
  Twist,
  PoseWithCovarianceStamped,
  SaveWaypointRequest,
  SaveWaypointResponse,
  NavigateToWaypointRequest,
  NavigateToWaypointResponse,
  DeleteWaypointRequest,
  DeleteWaypointResponse,
  ListWaypointsRequest,
  ListWaypointsResponse,
  TriggerRequest,
  TriggerResponse,
} from '../types/ros';
import type { Ros as RosType } from 'roslib';

let ros: RosType | null = null;
let connectedPort: number | null = null;
let connectedHost: string | null = null;

/**
 * Get rosbridge URL from current location or params
 */
export function getRosbridgeUrl(): string {
  // Check URL params first
  const params = new URLSearchParams(window.location.search);
  const portParam = params.get('rosbridge_port');
  const hostParam = params.get('rosbridge_host');

  const port = portParam ? parseInt(portParam, 10) : 9090;
  const host = hostParam || window.location.hostname || 'localhost';

  return `ws://${host}:${port}`;
}

/**
 * Initialize ROS connection
 */
export function initROS(url?: string): Promise<void> {
  return new Promise((resolve, reject) => {
    if (typeof window === 'undefined' || !window.ROSLIB) {
      reject(new Error('ROSLIB not available. Load roslib.js first.'));
      return;
    }

    const rosUrl = url || getRosbridgeUrl();

    // Parse URL to get host and port
    const urlObj = new URL(rosUrl);
    connectedHost = urlObj.hostname;
    connectedPort = parseInt(urlObj.port, 10);

    ros = new window.ROSLIB.Ros({ url: rosUrl }) as unknown as RosType;

    ros.on('connection', () => {
      console.log(`ROS bridge connected to ${rosUrl}`);
      resolve();
    });

    ros.on('error', (err: Error) => {
      console.error('ROS bridge error:', err);
      reject(err);
    });

    ros.on('close', () => {
      console.log('ROS bridge connection closed');
      connectedHost = null;
      connectedPort = null;
    });
  });
}

/**
 * Get ROS instance
 */
export function getROS() {
  return ros;
}

/**
 * Check if ROS is connected
 */
export function isConnected(): boolean {
  return ros !== null;
}

/**
 * Close ROS connection
 */
export function closeROS(): void {
  if (ros) {
    ros.close();
    ros = null;
  }
}

/**
 * Create a topic
 */
export function createTopic<TMsg = any>(
  name: string,
  messageType: string,
  compression?: 'none' | 'cbor' | 'png'
) {
  if (!ros) {
    throw new Error('ROS not initialized');
  }
  return new window.ROSLIB.Topic({
    ros,
    name,
    messageType,
    compression,
  }) as TopicType<TMsg>;
}

/**
 * Publish to a topic
 */
export function publish<TMsg = any>(topicName: string, messageType: string, message: TMsg): void {
  const topic = createTopic<TMsg>(topicName, messageType);
  topic.publish(new window.ROSLIB.Message(message));
}

/**
 * Subscribe to a topic
 */
export function subscribe<TMsg = any>(
  topicName: string,
  messageType: string,
  callback: (message: TMsg) => void
): () => void {
  const topic = createTopic<TMsg>(topicName, messageType);
  topic.subscribe(callback);

  // Return unsubscribe function
  return () => topic.unsubscribe(callback);
}

/**
 * Call a ROS service with timeout
 */
export function callService<T = unknown, U = unknown>(
  serviceName: string,
  serviceType: string,
  request: T,
  timeoutMs: number = 5000
): Promise<U> {
  return new Promise((resolve, reject) => {
    if (!ros) {
      reject(new Error('ROS not initialized'));
      return;
    }

    const service = new window.ROSLIB.Service({
      ros,
      name: serviceName,
      serviceType,
    }) as ServiceType<T, U>;

    const requestObj = new window.ROSLIB.ServiceRequest(request);

    // Set up timeout
    const timeoutId = setTimeout(() => {
      reject(new Error(`Service call timeout: ${serviceName} (${timeoutMs}ms)`));
    }, timeoutMs);

    service.callService(requestObj, (result: U) => {
      clearTimeout(timeoutId);
      resolve(result);
    }, (error: Error) => {
      clearTimeout(timeoutId);
      reject(error);
    });
  });
}

/**
 * Publish cmd_vel (velocity command)
 */
export function publishCmdVel(linear: { x: number; y: number; z: number }, angular: { x: number; y: number; z: number }): void {
  if (!isConnected()) return;
  publish('/cmd_vel', 'geometry_msgs/msg/Twist', { linear, angular });
}

/**
 * Stop the robot (publish zero velocity)
 */
export function stopRobot(): void {
  if (!isConnected()) return;
  publishCmdVel({ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 });
}

/**
 * Subscribe to amcl_pose (current robot position)
 */
export function subscribePose(callback: (pose: PoseWithCovarianceStamped) => void): () => void {
  return subscribe('/amcl_pose', 'geometry_msgs/msg/PoseWithCovarianceStamped', callback);
}

/**
 * Save waypoint
 */
export function saveWaypoint(req: SaveWaypointRequest): Promise<SaveWaypointResponse> {
  return callService<SaveWaypointRequest, SaveWaypointResponse>(
    '/save_waypoint',
    'verter_admin_msgs/srv/SaveWaypoint',
    req
  );
}

/**
 * Navigate to waypoint
 */
export function navigateToWaypoint(req: NavigateToWaypointRequest): Promise<NavigateToWaypointResponse> {
  return callService<NavigateToWaypointRequest, NavigateToWaypointResponse>(
    '/navigate_to_waypoint',
    'verter_admin_msgs/srv/NavigateToWaypoint',
    req
  );
}

/**
 * Delete waypoint
 */
export function deleteWaypoint(req: DeleteWaypointRequest): Promise<DeleteWaypointResponse> {
  return callService<DeleteWaypointRequest, DeleteWaypointResponse>(
    '/delete_waypoint',
    'verter_admin_msgs/srv/DeleteWaypoint',
    req
  );
}

/**
 * List all waypoints
 */
export function listWaypoints(): Promise<ListWaypointsResponse> {
  return callService<ListWaypointsRequest, ListWaypointsResponse>(
    '/list_waypoints',
    'verter_admin_msgs/srv/ListWaypoints',
    {}
  );
}

/**
 * Start patrol
 */
export function startPatrol(): Promise<TriggerResponse> {
  return callService<TriggerRequest, TriggerResponse>(
    '/start_patrol',
    'std_srvs/srv/Trigger',
    {}
  );
}

/**
 * Stop patrol
 */
export function stopPatrol(): Promise<TriggerResponse> {
  return callService<TriggerRequest, TriggerResponse>(
    '/stop_patrol',
    'std_srvs/srv/Trigger',
    {}
  );
}
