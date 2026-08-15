/**
 * ROS bridge service — диалоговый kiosk.
 * Тонкая обёртка над npm-бандлом roslib.js: подключение к rosbridge,
 * тихий реконнект, подписки на диалоговые топики и публикация управления.
 *
 * Внимание: rosbridge даёт НЕаутентифицированный доступ к ROS.
 * Допустимо только на localhost для kiosk.
 */

import ROSLIB from 'roslib';
import type { Ros as RosType, Topic as TopicType } from 'roslib';
import type { RosStatus } from '../types/ros';

let ros: RosType | null = null;

// Тихий реконнект: без UI, без выбрасывания ошибок наружу.
const RECONNECT_DELAY_MS = 3000;
let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
let wantConnected = false;
let statusCb: ((status: RosStatus) => void) | null = null;

/**
 * URL rosbridge. По умолчанию ws://localhost:9090 (kiosk на самом роботе).
 * При необходимости переопределяется через ?rosbridge=host:port.
 */
export function getRosbridgeUrl(): string {
  if (typeof window === 'undefined') {
    return 'ws://localhost:9090';
  }
  const param = new URLSearchParams(window.location.search).get('rosbridge');
  if (param) {
    // host:port или полный URL
    if (param.startsWith('ws://') || param.startsWith('wss://')) {
      return param;
    }
    return `ws://${param}`;
  }
  return 'ws://localhost:9090';
}

function setStatus(status: RosStatus): void {
  if (statusCb) {
    statusCb(status);
  }
}

function scheduleReconnect(): void {
  if (reconnectTimer) {
    clearTimeout(reconnectTimer);
  }
  reconnectTimer = setTimeout(() => {
    reconnectTimer = null;
    if (wantConnected) {
      initROS(getRosbridgeUrl());
    }
  }, RECONNECT_DELAY_MS);
}

/**
 * Инициализация подключения к rosbridge. Идемпотентно для одного URL.
 * Тихий реконнект: при разрыве пытается переподключиться каждые RECONNECT_DELAY_MS.
 */
export function initROS(url: string = getRosbridgeUrl()): Promise<void> {
  wantConnected = true;

  // Уже подключено к этому же URL — ничего не делаем.
  if (ros && (ros as unknown as { url?: string }).url === url) {
    return Promise.resolve();
  }

  setStatus('connecting');

  return new Promise((resolve) => {
    try {
      if (ros) {
        try {
          ros.close();
        } catch {
          /* ignore */
        }
        ros = null;
      }

      const newRos = new ROSLIB.Ros({ url });
      ros = newRos as unknown as RosType;

      newRos.on('connection', () => {
        setStatus('connected');
        resolve();
      });

      newRos.on('error', () => {
        setStatus('error');
        resolve();
        scheduleReconnect();
      });

      newRos.on('close', () => {
        setStatus('disconnected');
        resolve();
        if (wantConnected) {
          scheduleReconnect();
        }
      });
    } catch {
      setStatus('error');
      resolve();
      scheduleReconnect();
    }
  });
}

/**
 * Регистрирует колбэк статуса соединения (используется rosStore).
 */
export function onStatus(cb: (status: RosStatus) => void): void {
  statusCb = cb;
}

/**
 * Корректно закрывает подключение и прекращает реконнекты.
 */
export function closeROS(): void {
  wantConnected = false;
  if (reconnectTimer) {
    clearTimeout(reconnectTimer);
    reconnectTimer = null;
  }
  if (ros) {
    try {
      ros.close();
    } catch {
      /* ignore */
    }
    ros = null;
  }
  setStatus('disconnected');
}

/**
 * Текущий экземпляр Ros (для хуков подписки/публикации).
 */
export function getROS(): RosType | null {
  return ros;
}

// ============ Подписки ============

/**
 * Создаёт подписку на std_msgs/String топик и возвращает функцию отписки.
 */
export function subscribeString(
  topicName: string,
  listener: (data: string) => void,
): () => void {
  if (!ros) {
    return () => {};
  }
  const topic: TopicType = new ROSLIB.Topic({
    ros: ros as unknown as ConstructorParameters<typeof ROSLIB.Topic>[0]['ros'],
    name: topicName,
    messageType: 'std_msgs/msg/String',
    throttle_rate: 0,
  });
  topic.subscribe((msg: { data?: unknown }) => {
    if (typeof msg.data === 'string') {
      listener(msg.data);
    }
  });
  return () => {
    try {
      topic.unsubscribe();
    } catch {
      /* ignore */
    }
  };
}

/**
 * Публикует std_msgs/String в топик.
 */
export function publishString(topicName: string, data: string): void {
  if (!ros) {
    return;
  }
  const topic: TopicType = new ROSLIB.Topic({
    ros: ros as unknown as ConstructorParameters<typeof ROSLIB.Topic>[0]['ros'],
    name: topicName,
    messageType: 'std_msgs/msg/String',
  });
  topic.publish(new ROSLIB.Message({ data }));
}

// ============ Диалоговые топики ============

/** /dialog_status — единый источник правды для UI (recognition + ai_assistant). */
export function subscribeDialogStatus(listener: (status: string) => void): () => void {
  return subscribeString('/dialog_status', listener);
}

/** /ai_question — распознанный вопрос пользователя. */
export function subscribeAIQuestion(listener: (question: string) => void): () => void {
  return subscribeString('/ai_question', listener);
}

/** /text_to_speech — текст, который робот озвучивает (ответ/ошибка/farewell). */
export function subscribeTextToSpeech(listener: (text: string) => void): () => void {
  return subscribeString('/text_to_speech', listener);
}

/** /ui_dialog_control — команда от фронтенда ('start' | 'stop'). */
export function publishDialogControl(command: 'start' | 'stop'): void {
  publishString('/ui_dialog_control', command);
}
