/**
 * Типы диалогового kiosk-интерфейса.
 */

/** FSM-статус диалога (топик /dialog_status). */
export type DialogStatus =
  | 'idle'
  | 'listening'
  | 'thinking'
  | 'speaking'
  | 'error';

/** Тип ошибки AI/сети (суффикс статуса error:*). */
export type ErrorType = 'timeout' | 'network' | 'unavailable';

/** Роль текущего сообщения на экране. */
export type MessageRole =
  | 'question'
  | 'answer'
  | 'error'
  | 'farewell'
  | 'welcome'
  | 'thinking'
  | 'listening'
  | 'speaking';

/** Одно текущее сообщение (лента не нужна — отображаем одно). */
export interface DialogMessage {
  role: MessageRole;
  text: string;
}

/**
 * Разбирает значение топика /dialog_status.
 * Возвращает нормализованный статус и (для error:*) тип ошибки.
 */
export function parseDialogStatus(
  raw: string,
): { status: DialogStatus; errorType: ErrorType | null } {
  if (raw.startsWith('error:')) {
    const suffix = raw.slice('error:'.length);
    if (suffix === 'timeout' || suffix === 'network' || suffix === 'unavailable') {
      return { status: 'error', errorType: suffix };
    }
    return { status: 'error', errorType: 'unavailable' };
  }
  if (raw === 'idle' || raw === 'listening' || raw === 'thinking' || raw === 'speaking') {
    return { status: raw, errorType: null };
  }
  // Неизвестное значение — не меняем состояние.
  return { status: 'idle', errorType: null };
}
