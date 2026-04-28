/**
 * useIdleTimer hook
 * Manages idle state and auto-return to Idle screen
 */

import { useEffect, useCallback, useRef } from 'react';
import { useUIStore } from '../store/uiStore';

interface UseIdleTimerOptions {
  enabled?: boolean;
  timeout?: number; // seconds, overrides store value
  onIdle?: () => void;
  onActive?: () => void;
}

export function useIdleTimer(options: UseIdleTimerOptions = {}) {
  const { enabled = true, timeout, onIdle, onActive } = options;

  const isIdle = useUIStore((state) => state.isIdle);
  const idleTimeout = useUIStore((state) => state.idleTimeout);
  const recordActivity = useUIStore((state) => state.recordActivity);
  const setIdle = useUIStore((state) => state.setIdle);

  const timeoutRef = useRef<NodeJS.Timeout | null>(null);
  const actualTimeout = timeout ?? idleTimeout;

  // Reset the idle timer
  const reset = useCallback(() => {
    if (!enabled) return;

    recordActivity();

    if (isIdle) {
      setIdle(false);
      onActive?.();
    }
  }, [enabled, isIdle, recordActivity, setIdle, onActive]);

  // Setup idle timeout
  useEffect(() => {
    if (!enabled) return;

    // Clear existing timeout
    if (timeoutRef.current) {
      clearTimeout(timeoutRef.current);
    }

    // Set new timeout
    timeoutRef.current = setTimeout(() => {
      setIdle(true);
      onIdle?.();
    }, actualTimeout * 1000);

    return () => {
      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
      }
    };
  }, [enabled, actualTimeout, isIdle, setIdle, onIdle]);

  // Activity event listeners
  useEffect(() => {
    if (!enabled) return;

    const activityEvents = [
      'mousedown',
      'mousemove',
      'keydown',
      'scroll',
      'touchstart',
    ];

    activityEvents.forEach((event) => {
      window.addEventListener(event, reset, { passive: true });
    });

    return () => {
      activityEvents.forEach((event) => {
        window.removeEventListener(event, reset);
      });
    };
  }, [enabled, reset]);

  return {
    isIdle,
    reset,
  };
}

/**
 * HOC to wrap a component with idle timer functionality
 * Auto-returns to Idle screen when inactive
 */
import React from 'react';

export function withIdleTimer<P extends object>(
  Component: React.ComponentType<P>,
  options?: UseIdleTimerOptions
) {
  return function WithIdleTimer(props: P) {
    const { isIdle } = useIdleTimer({
      enabled: options?.enabled ?? true,
      timeout: options?.timeout,
      onIdle: options?.onIdle,
    });

    // If idle, don't render the wrapped component
    // The router should handle showing IdlePage instead
    if (isIdle && options?.enabled !== false) {
      return null;
    }

    return React.createElement(Component, props);
  };
}
