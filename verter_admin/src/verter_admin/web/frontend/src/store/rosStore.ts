/**
 * ROS Store - Zustand
 * Manages ROS connection state
 */

import { create } from 'zustand';
import type { RosStatus } from '../types/ros';

interface ROSState {
  // State
  isConnected: boolean;
  status: RosStatus;
  url: string;
  host: string;
  port: number;
  error: string | null;

  // Actions
  setStatus: (status: RosStatus) => void;
  setConnected: (isConnected: boolean) => void;
  setError: (error: string | null) => void;
  setUrl: (url: string, host: string, port: number) => void;
  reset: () => void;
}

const initialState: Omit<ROSState, 'setStatus' | 'setConnected' | 'setError' | 'setUrl' | 'reset'> = {
  isConnected: false,
  status: 'disconnected',
  url: 'ws://localhost:9090',
  host: 'localhost',
  port: 9090,
  error: null,
};

export const useROSStore = create<ROSState>((set) => ({
  ...initialState,

  setStatus: (status) =>
    set({
      status,
      isConnected: status === 'connected',
      error: status === 'error' ? 'Connection error' : null,
    }),

  setConnected: (isConnected) =>
    set({
      isConnected,
      status: isConnected ? 'connected' : 'disconnected',
      error: null,
    }),

  setError: (error) =>
    set({
      error,
      status: error ? 'error' : 'disconnected',
      isConnected: false,
    }),

  setUrl: (url, host, port) =>
    set({ url, host, port }),

  reset: () => set(initialState),
}));
