/**
 * useROS hook
 * Manages ROS connection lifecycle
 */

import { useEffect, useCallback, useRef } from 'react';
import { useROSStore } from '../store/rosStore';
import * as rosbridge from '../services/rosbridge';

export function useROS(autoConnect: boolean = true) {
  const isConnected = useROSStore((state) => state.isConnected);
  const status = useROSStore((state) => state.status);
  const setStatus = useROSStore((state) => state.setStatus);
  const setConnected = useROSStore((state) => state.setConnected);
  const setError = useROSStore((state) => state.setError);
  const setUrl = useROSStore((state) => state.setUrl);

  const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const isConnectingRef = useRef(false);

  // Connect to ROS
  const connect = useCallback(async () => {
    if (isConnectingRef.current || isConnected) {
      return;
    }

    isConnectingRef.current = true;
    setStatus('connecting');

    try {
      const url = rosbridge.getRosbridgeUrl();
      const urlObj = new URL(url);
      setUrl(url, urlObj.hostname, parseInt(urlObj.port, 10));

      await rosbridge.initROS(url);
      setConnected(true);
      isConnectingRef.current = false;

      // Clear any pending reconnect
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
        reconnectTimeoutRef.current = null;
      }
    } catch (err) {
      const error = err instanceof Error ? err.message : 'Connection failed';
      setError(error);
      isConnectingRef.current = false;

      // Auto-reconnect after 5 seconds
      reconnectTimeoutRef.current = setTimeout(() => {
        isConnectingRef.current = false;
        connect();
      }, 5000);
    }
  }, [isConnected, setStatus, setConnected, setError, setUrl]);

  // Disconnect from ROS
  const disconnect = useCallback(() => {
    rosbridge.closeROS();
    setConnected(false);
    isConnectingRef.current = false;

    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }
  }, [setConnected]);

  // Auto-connect on mount
  useEffect(() => {
    if (autoConnect) {
      connect();
    }

    return () => {
      disconnect();
    };
  }, [autoConnect, connect, disconnect]);

  return {
    isConnected,
    status,
    connect,
    disconnect,
  };
}
