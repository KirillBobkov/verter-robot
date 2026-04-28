/**
 * UI Store - Zustand
 * Manages UI state (mode, idle timer)
 */

import { create } from 'zustand';
import type { AppMode } from '../types/ros';

interface UIState {
  // State
  mode: AppMode;
  idleTimeout: number; // seconds
  isIdle: boolean;
  lastActivity: number; // timestamp

  // Actions
  setMode: (mode: AppMode) => void;
  setIdleTimeout: (timeout: number) => void;
  setIdle: (isIdle: boolean) => void;
  recordActivity: () => void;
  reset: () => void;
}

const IDLE_TIMEOUT_DEFAULT = 20; // seconds
const INITIAL_IDLE_DELAY = 60; // seconds before idle timer starts

const initialState: Omit<UIState, 'setMode' | 'setIdleTimeout' | 'setIdle' | 'recordActivity' | 'reset'> = {
  mode: 'setup', // Default to setup mode for development
  idleTimeout: IDLE_TIMEOUT_DEFAULT,
  isIdle: false,
  lastActivity: Date.now(),
};

export const useUIStore = create<UIState>((set) => ({
  ...initialState,

  setMode: (mode) => set({ mode }),

  setIdleTimeout: (timeout) => set({ idleTimeout: timeout }),

  setIdle: (isIdle) => set({ isIdle }),

  recordActivity: () => set({ lastActivity: Date.now(), isIdle: false }),

  reset: () => set(initialState),
}));

/**
 * Hook to check if app should be idle based on inactivity
 * Returns true if idle timeout has elapsed
 */
export function getIsIdleTimedOut(lastActivity: number, timeout: number): boolean {
  const elapsed = (Date.now() - lastActivity) / 1000;
  return elapsed > timeout;
}
