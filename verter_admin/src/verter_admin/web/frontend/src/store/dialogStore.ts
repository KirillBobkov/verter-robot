/**
 * dialogStore — Zustand-стор состояния диалога.
 * Единственный источник правды для UI: статус FSM, тип ошибки, текущее сообщение.
 */

import { create } from 'zustand';
import type { DialogStatus, ErrorType, DialogMessage, MessageRole } from '../types/dialog';
import { parseDialogStatus } from '../types/dialog';

interface DialogState {
  // Состояние
  status: DialogStatus;
  errorType: ErrorType | null;
  currentMessage: DialogMessage | null;

  // Производные
  isDialogActive: () => boolean;

  // Действия
  setStatus: (raw: string) => void;
  setMessage: (role: MessageRole, text: string) => void;
  clearMessage: () => void;
  reset: () => void;
}

const WELCOME: DialogMessage = {
  role: 'welcome',
  text: 'Нажми на кнопку чтобы начать диалог',
};

export const useDialogStore = create<DialogState>((set, get) => ({
  status: 'idle',
  errorType: null,
  currentMessage: WELCOME,

  isDialogActive: () => get().status !== 'idle',

  setStatus: (raw: string) => {
    const { status, errorType } = parseDialogStatus(raw);

    // Простая модель: errorType живёт только в статусе error. На любом другом
    // статусе сбрасываем — иначе ошибка «прилипала» к панели/аватару и переключатель
    // состояний не менял подсветку при уходе из ошибки.
    set((state) => {
      if (status === 'error') {
        return { status, errorType };
      }
      // listening — сбрасываем текст ошибки (робот ждёт новый вопрос);
      // welcome/вопрос оставляем — сменятся новым сообщением из топика.
      if (status === 'listening') {
        return {
          status,
          errorType: null,
          currentMessage: state.currentMessage?.role === 'error' ? null : state.currentMessage,
        };
      }
      return { status, errorType: null };
    });

    // Возврат в idle — показываем приветствие в облачке (farewell в currentMessage
    // не попадает — он аудио-only, см. useDialogROS).
    if (status === 'idle') {
      set({ currentMessage: WELCOME });
    }
  },

  setMessage: (role: MessageRole, text: string) => {
    set({ currentMessage: { role, text } });
  },

  clearMessage: () => {
    set({ currentMessage: null });
  },

  reset: () => {
    set({ status: 'idle', errorType: null, currentMessage: WELCOME });
  },
}));
