/**
 * useROS — управляет жизненным циклом подключения к rosbridge.
 * Статус приходит через rosbridge.onStatus (включая тихий реконнект).
 */

import { useEffect, useRef } from 'react';
import { useROSStore } from '../store/rosStore';
import * as rosbridge from '../services/rosbridge';

export function useROS(autoConnect: boolean = true) {
  const isConnected = useROSStore((state) => state.isConnected);
  const status = useROSStore((state) => state.status);
  const setStatus = useROSStore((state) => state.setStatus);
  const setUrl = useROSStore((state) => state.setUrl);

  const isConnectingRef = useRef(false);

  useEffect(() => {
    // Проксируем статус из rosbridge (обновляется при реконнекте тоже).
    const offStatus = rosbridge.onStatus((newStatus) => {
      setStatus(newStatus);
    });

    if (!autoConnect) {
      return offStatus;
    }

    if (isConnectingRef.current) {
      return offStatus;
    }
    isConnectingRef.current = true;

    const url = rosbridge.getRosbridgeUrl();
    try {
      const urlObj = new URL(url);
      setUrl(url, urlObj.hostname, parseInt(urlObj.port, 10));
    } catch {
      /* некорректный URL — продолжаем с дефолтом */
    }

    rosbridge.initROS(url).finally(() => {
      isConnectingRef.current = false;
    });

    return () => {
      offStatus();
      rosbridge.closeROS();
      isConnectingRef.current = false;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [autoConnect]);

  return {
    isConnected,
    status,
  };
}
