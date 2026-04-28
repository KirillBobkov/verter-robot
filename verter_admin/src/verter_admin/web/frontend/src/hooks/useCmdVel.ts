/**
 * useCmdVel hook
 * Manages velocity commands to /cmd_vel
 */

import { useState, useCallback, useRef, useEffect } from 'react';
import * as rosbridge from '../services/rosbridge';
import type { Twist } from '../types/ros';

interface KeyState {
  w: boolean;
  a: boolean;
  s: boolean;
  d: boolean;
}

interface VelocityControl {
  linear: number;  // m/s
  angular: number; // rad/s
}

export function useCmdVel() {
  const [isEnabled, setIsEnabled] = useState(false);
  const [speed, setSpeed] = useState<VelocityControl>({ linear: 0.3, angular: 0.5 });
  const keysRef = useRef<KeyState>({ w: false, a: false, s: false, d: false });
  const intervalRef = useRef<NodeJS.Timeout | null>(null);

  // Publish velocity command
  const publishVelocity = useCallback((linear: number, angular: number) => {
    rosbridge.publishCmdVel(
      { x: linear, y: 0, z: 0 },
      { x: 0, y: 0, z: angular }
    );
  }, []);

  // Stop the robot
  const stop = useCallback(() => {
    rosbridge.stopRobot();

    // Clear all keys
    keysRef.current = { w: false, a: false, s: false, d: false };

    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
  }, []);

  // Enable keyboard control
  const enable = useCallback(() => {
    setIsEnabled(true);
  }, []);

  // Disable keyboard control
  const disable = useCallback(() => {
    setIsEnabled(false);
    stop();
  }, [stop]);

  // Handle key down
  const handleKeyDown = useCallback((e: KeyboardEvent) => {
    if (!isEnabled) return;

    const key = e.key.toLowerCase();
    if (['w', 'a', 's', 'd', ' '].includes(key)) {
      e.preventDefault();

      // Handle space bar for stop
      if (key === ' ') {
        stop();
        return;
      }

      keysRef.current[key as keyof KeyState] = true;
    }
  }, [isEnabled, stop]);

  // Handle key up
  const handleKeyUp = useCallback((e: KeyboardEvent) => {
    if (!isEnabled) return;

    const key = e.key.toLowerCase();
    if (['w', 'a', 's', 'd'].includes(key)) {
      e.preventDefault();
      keysRef.current[key as keyof KeyState] = false;
    }
  }, [isEnabled]);

  // Velocity update loop
  useEffect(() => {
    if (!isEnabled) {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
      return;
    }

    intervalRef.current = setInterval(() => {
      const keys = keysRef.current;
      let linear = 0;
      let angular = 0;

      if (keys.w) linear += speed.linear;
      if (keys.s) linear -= speed.linear;
      if (keys.a) angular += speed.angular;
      if (keys.d) angular -= speed.angular;

      // Publish if non-zero velocity
      if (linear !== 0 || angular !== 0) {
        publishVelocity(linear, angular);
      }
    }, 100); // 10Hz update rate

    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
      }
    };
  }, [isEnabled, speed, publishVelocity]);

  // Register keyboard listeners
  useEffect(() => {
    if (isEnabled) {
      window.addEventListener('keydown', handleKeyDown);
      window.addEventListener('keyup', handleKeyUp);
    }

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [isEnabled, handleKeyDown, handleKeyUp]);

  return {
    isEnabled,
    speed,
    setSpeed,
    enable,
    disable,
    stop,
    publishVelocity,
  };
}
