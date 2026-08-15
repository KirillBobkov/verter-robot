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
  text: 'Привет! Нажми на кнопку чтобы начать диалог!',
};

export const useDialogStore = create<DialogState>((set, get) => ({
  status: 'idle',
  errorType: null,
  currentMessage: WELCOME,

  isDialogActive: () => get().status !== 'idle',

  setStatus: (raw: string) => {
    const { status, errorType } = parseDialogStatus(raw);

    // Ошибка «прилипает» к аватару: сохраняем errorType, пока не придёт
    // listening/idle (новый диалог или возврат в ожидание).
    set((state) => {
      // При переходе в listening/idle — сбрасываем ошибку.
      if (status === 'listening' || status === 'idle') {
        return { status, errorType: null };
      }
      // Для error — обновляем errorType.
      if (status === 'error') {
        return { status, errorType };
      }
      // thinking/speaking — сохраняем накопленную ошибку (если вдруг), статус обновляем.
      return { status, errorType: state.errorType };
    });

    // Возврат в idle — показываем приветствие.
    if (status === 'idle' && get().currentMessage?.role !== 'farewell') {
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
