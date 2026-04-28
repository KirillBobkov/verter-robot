/**
 * Quaternion utilities
 * Convert between quaternion and Euler angles
 */

/**
 * Convert quaternion to yaw angle (around Z axis)
 * Uses the formula: yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
 *
 * @param qw - quaternion w component
 * @param qx - quaternion x component
 * @param qy - quaternion y component
 * @param qz - quaternion z component
 * @returns yaw angle in radians
 */
export function quaternionToYaw(
  qw: number,
  qx: number,
  qy: number,
  qz: number
): number {
  return Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
}

/**
 * Convert yaw angle to quaternion (around Z axis)
 *
 * @param yaw - angle in radians
 * @returns quaternion {x, y, z, w}
 */
export function yawToQuaternion(yaw: number): { x: number; y: number; z: number; w: number } {
  const halfYaw = yaw / 2;
  const sinHalf = Math.sin(halfYaw);
  const cosHalf = Math.cos(halfYaw);

  return {
    x: 0,
    y: 0,
    z: sinHalf,
    w: cosHalf,
  };
}

/**
 * Normalize angle to [-pi, pi] range
 *
 * @param angle - angle in radians
 * @returns normalized angle in radians
 */
export function normalizeAngle(angle: number): number {
  while (angle > Math.PI) angle -= 2 * Math.PI;
  while (angle < -Math.PI) angle += 2 * Math.PI;
  return angle;
}

/**
 * Convert radians to degrees
 *
 * @param radians - angle in radians
 * @returns angle in degrees
 */
export function radToDeg(radians: number): number {
  return (radians * 180) / Math.PI;
}

/**
 * Convert degrees to radians
 *
 * @param degrees - angle in degrees
 * @returns angle in radians
 */
export function degToRad(degrees: number): number {
  return (degrees * Math.PI) / 180;
}

/**
 * Format angle for display
 * Returns a string like "90.0°" or "-45.0°"
 *
 * @param radians - angle in radians
 * @param decimals - number of decimal places (default: 1)
 * @returns formatted angle string
 */
export function formatAngle(radians: number, decimals: number = 1): string {
  const degrees = radToDeg(radians);
  return `${degrees.toFixed(decimals)}°`;
}
