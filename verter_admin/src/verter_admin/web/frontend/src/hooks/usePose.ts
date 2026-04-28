/**
 * usePose hook
 * Subscribes to robot pose from /amcl_pose
 */

import { useState, useEffect, useCallback, useRef } from 'react';
import { quaternionToYaw } from '../utils/quaternion';
import type { PoseWithCovarianceStamped, RobotPose2D } from '../types/ros';
import * as rosbridge from '../services/rosbridge';

interface UsePoseOptions {
  enabled?: boolean;
}

export function usePose(options: UsePoseOptions = {}) {
  const { enabled = true } = options;

  const [pose, setPose] = useState<RobotPose2D>({ x: 0, y: 0, yaw: 0 });
  const [isSubscribed, setIsSubscribed] = useState(false);
  const unsubscribeRef = useRef<(() => void) | null>(null);

  // Subscribe to pose updates
  useEffect(() => {
    if (!enabled) {
      if (unsubscribeRef.current) {
        unsubscribeRef.current();
        unsubscribeRef.current = null;
      }
      setIsSubscribed(false);
      return;
    }

    try {
      const unsubscribe = rosbridge.subscribePose((msg: PoseWithCovarianceStamped) => {
        const p = msg.pose.pose;
        const q = p.orientation;
        const yaw = quaternionToYaw(q.w, q.x, q.y, q.z);

        setPose({
          x: p.position.x,
          y: p.position.y,
          yaw,
        });
      });

      unsubscribeRef.current = unsubscribe;
      setIsSubscribed(true);
    } catch (err) {
      console.error('Failed to subscribe to pose:', err);
    }

    return () => {
      if (unsubscribeRef.current) {
        unsubscribeRef.current();
        unsubscribeRef.current = null;
      }
    };
  }, [enabled]);

  // Manual refresh (fetch once)
  const refresh = useCallback(() => {
    return new Promise<RobotPose2D>((resolve, reject) => {
      let unsubscribe: (() => void) | null = null;
      let timedOut = false;
      let resolved = false;

      // Cleanup function to clear timers and unsubscribe
      const cleanup = () => {
        if (timeout1) clearTimeout(timeout1);
        if (timeout2) clearTimeout(timeout2);
        if (unsubscribe && !resolved) {
          unsubscribe();
        }
      };

      // Short timeout to receive first message
      const timeout1 = setTimeout(() => {
        if (!resolved && !timedOut) {
          resolved = true;
          cleanup();
          // Return current pose from state as fallback
          resolve(pose);
        }
      }, 100);

      // Long timeout for error
      const timeout2 = setTimeout(() => {
        if (!resolved) {
          timedOut = true;
          cleanup();
          reject(new Error('Pose fetch timeout'));
        }
      }, 5000);

      try {
        unsubscribe = rosbridge.subscribePose((msg: PoseWithCovarianceStamped) => {
          if (resolved) return;

          const p = msg.pose.pose;
          const q = p.orientation;
          const yaw = quaternionToYaw(q.w, q.x, q.y, q.z);

          const pose2D = { x: p.position.x, y: p.position.y, yaw };
          setPose(pose2D);
          resolved = true;
          cleanup();
          resolve(pose2D);
        });
      } catch (err) {
        cleanup();
        reject(err);
      }
    });
  }, [pose]);

  return {
    pose,
    isSubscribed,
    refresh,
  };
}
