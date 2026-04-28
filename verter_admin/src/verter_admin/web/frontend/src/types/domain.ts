/**
 * Domain types for Verter Robot Web UI
 * Business logic types independent of ROS
 */

import type { Waypoint, RobotPose2D, AppMode, NavigationStatus } from './ros';

// ============ UI State ============

export interface UIState {
  mode: AppMode;
  idleTimeout: number; // seconds
  isIdle: boolean;
}

// ============ Location Types ============

/**
 * Location type for cabinets/rooms
 */
export interface Location extends Waypoint {
  description?: string;
  floor?: string;
  building?: string;
}

/**
 * Location category for grouping
 */
export type LocationCategory =
  | 'cardiology'
  | 'laboratory'
  | 'reception'
  | 'radiology'
  | 'surgery'
  | 'pharmacy'
  | 'other';

// ============ Menu Items ============

export interface MenuItem {
  id: string;
  label: string;
  icon: string;
  route: string;
  order: number;
}

// ============ Form State ============

export interface LocationFormState {
  name: string;
  description: string;
  useCurrentPose: boolean;
  x: string;
  y: string;
  theta: string;
}

// ============ Log Entry ============

export interface LogEntry {
  timestamp: string;
  level: 'info' | 'warn' | 'error' | 'success';
  message: string;
}

// ============ Key Bindings ============

export interface KeyBinding {
  key: string;
  action: () => void;
  description: string;
}

// ============ Speed Control ============

export interface SpeedControl {
  linear: number;  // m/s
  angular: number; // rad/s
}
