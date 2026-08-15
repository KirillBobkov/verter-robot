/**
 * ROS type definitions
 * Types for roslib.js and dialog messages
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

// ============ ROS Message Types (dialog) ============

/**
 * std_msgs/msg/String — универсальная обёртка для dialog_status,
 * ui_dialog_control, ai_question, text_to_speech.
 */
export interface StringMsg {
  data: string;
}

/**
 * std_msgs/msg/Bool — speech_control / tts_control.
 */
export interface BoolMsg {
  data: boolean;
}

// ============ Connection ============

/**
 * ROS connection status
 */
export type RosStatus = 'disconnected' | 'connecting' | 'connected' | 'error';
