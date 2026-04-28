/**
 * Navigation Store - Zustand
 * Manages navigation state and current goal
 */

import { create } from 'zustand';
import type { NavigationStatus, Waypoint } from '../types/ros';

interface NavigationState {
  // State
  status: NavigationStatus;
  currentGoal: Waypoint | null;
  progress: number; // 0-100
  estimatedTimeRemaining: number; // seconds
  error: string | null;

  // Actions
  setStatus: (status: NavigationStatus) => void;
  setGoal: (goal: Waypoint | null) => void;
  setProgress: (progress: number) => void;
  setEstimatedTimeRemaining: (time: number) => void;
  setError: (error: string | null) => void;
  reset: () => void;
}

const initialState: Omit<NavigationState, 'setStatus' | 'setGoal' | 'setProgress' | 'setEstimatedTimeRemaining' | 'setError' | 'reset'> = {
  status: 'idle',
  currentGoal: null,
  progress: 0,
  estimatedTimeRemaining: 0,
  error: null,
};

export const useNavigationStore = create<NavigationState>((set) => ({
  ...initialState,

  setStatus: (status) => set({ status }),

  setGoal: (goal) =>
    set({
      currentGoal: goal,
      status: goal ? 'planning' : 'idle',
      progress: 0,
      error: null,
    }),

  setProgress: (progress) => set({ progress: Math.max(0, Math.min(100, progress)) }),

  setEstimatedTimeRemaining: (time) => set({ estimatedTimeRemaining: time }),

  setError: (error) =>
    set({
      error,
      status: error ? 'failed' : 'idle',
    }),

  reset: () => set(initialState),
}));
