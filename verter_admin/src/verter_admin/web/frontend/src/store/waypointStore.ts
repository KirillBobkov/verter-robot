/**
 * Waypoint Store - Zustand
 * Manages waypoints cache and operations
 */

import { create } from 'zustand';
import type { Waypoint } from '../types/ros';
import * as rosbridge from '../services/rosbridge';

interface WaypointState {
  // State
  waypoints: Waypoint[];
  loading: boolean;
  error: string | null;

  // Actions
  fetchWaypoints: () => Promise<void>;
  addWaypoint: (waypoint: Waypoint) => Promise<{ success: boolean; message: string }>;
  deleteWaypoint: (name: string) => Promise<{ success: boolean; message: string }>;
  updateWaypoint: (name: string, waypoint: Partial<Waypoint>) => Promise<{ success: boolean; message: string }>;
  setError: (error: string | null) => void;
  clearError: () => void;
}

export const useWaypointStore = create<WaypointState>((set, get) => ({
  // Initial state
  waypoints: [],
  loading: false,
  error: null,

  // Fetch all waypoints from ROS
  fetchWaypoints: async () => {
    set({ loading: true, error: null });
    try {
      const response = await rosbridge.listWaypoints();
      const waypoints: Waypoint[] = [];

      // Zip parallel arrays into Waypoint objects
      for (let i = 0; i < response.names.length; i++) {
        waypoints.push({
          name: response.names[i],
          x: response.x[i],
          y: response.y[i],
          theta: response.theta[i],
        });
      }

      set({ waypoints, loading: false });
    } catch (err) {
      const error = err instanceof Error ? err.message : 'Failed to fetch waypoints';
      set({ error, loading: false });
    }
  },

  // Add a new waypoint
  addWaypoint: async (waypoint) => {
    set({ loading: true, error: null });
    try {
      const response = await rosbridge.saveWaypoint({
        name: waypoint.name,
        x: waypoint.x,
        y: waypoint.y,
        theta: waypoint.theta,
      });

      if (response.success) {
        // Add to local cache
        set((state) => ({
          waypoints: [...state.waypoints, waypoint],
          loading: false,
        }));
      } else {
        set({ error: response.message, loading: false });
      }

      return { success: response.success, message: response.message };
    } catch (err) {
      const error = err instanceof Error ? err.message : 'Failed to add waypoint';
      set({ error, loading: false });
      return { success: false, message: error };
    }
  },

  // Delete a waypoint
  deleteWaypoint: async (name) => {
    set({ loading: true, error: null });
    try {
      const response = await rosbridge.deleteWaypoint({ name });

      if (response.success) {
        // Remove from local cache
        set((state) => ({
          waypoints: state.waypoints.filter((wp) => wp.name !== name),
          loading: false,
        }));
      } else {
        set({ error: response.message, loading: false });
      }

      return { success: response.success, message: response.message };
    } catch (err) {
      const error = err instanceof Error ? err.message : 'Failed to delete waypoint';
      set({ error, loading: false });
      return { success: false, message: error };
    }
  },

  // Update a waypoint (delete + save)
  updateWaypoint: async (name, waypoint) => {
    // First delete the old waypoint
    const deleteResult = await get().deleteWaypoint(name);
    if (!deleteResult.success) {
      return deleteResult;
    }

    // Then save the new waypoint with updated data
    const newWaypoint: Waypoint = {
      name: waypoint.name ?? name,
      x: waypoint.x ?? 0,
      y: waypoint.y ?? 0,
      theta: waypoint.theta ?? 0,
      description: waypoint.description,
    };

    return get().addWaypoint(newWaypoint);
  },

  setError: (error) => set({ error }),
  clearError: () => set({ error: null }),
}));
