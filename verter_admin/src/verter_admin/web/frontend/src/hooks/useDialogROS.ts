/**
 * useDialogROS — связывает ROS-подписки диалога с dialogStore.
 *
 * Подписки:
 *  - /dialog_status   → setStatus
 *  - /ai_question     → setMessage('question', text)
 *  - /text_to_speech  → setMessage(роль по текущему статусу)
 *
 * Публикация:
 *  - /ui_dialog_control ('start' | 'stop')
 *
 * Роль для /text_to_speech определяется по текущему статусу/ошибке:
 *  - errorType != null → 'error'
 *  - status === 'idle' → 'farewell' (прощание после стопа/таймаута)
 *  - иначе → 'answer'
 *
 * Важно: подписки переподписываются при каждом восстановлении соединения
 * (по isConnected), т.к. rosbridge при реконнекте создаёт новый экземпляр Ros,
 * а старые Topic-подписки привязаны к мёртвому подключению.
 */

import { useEffect, useRef } from 'react';
import { useDialogStore } from '../store/dialogStore';
import { useROSStore } from '../store/rosStore';
import * as rosbridge from '../services/rosbridge';

export function useDialogROS() {
  const setStatus = useDialogStore((s) => s.setStatus);
  const setMessage = useDialogStore((s) => s.setMessage);
  const isConnected = useROSStore((s) => s.isConnected);
  const statusRef = useRef(useDialogStore.getState().status);
  const errorTypeRef = useRef(useDialogStore.getState().errorType);

  // Держим актуальные значения статуса/ошибки для определения роли text_to_speech.
  useEffect(() => {
    const unsub = useDialogStore.subscribe((state) => {
      statusRef.current = state.status;
      errorTypeRef.current = state.errorType;
    });
    return unsub;
  }, []);

  // Переподписываемся при каждом подключении (включая после реконнекта).
  useEffect(() => {
    if (!isConnected) {
      return;
    }

    const unsubStatus = rosbridge.subscribeDialogStatus((raw) => {
      setStatus(raw);
    });

    const unsubQuestion = rosbridge.subscribeAIQuestion((question) => {
      setMessage('question', question);
    });

    const unsubTts = rosbridge.subscribeTextToSpeech((text) => {
      const role = errorTypeRef.current
        ? 'error'
        : statusRef.current === 'idle'
          ? 'farewell'
          : 'answer';
      setMessage(role, text);
    });

    return () => {
      unsubStatus();
      unsubQuestion();
      unsubTts();
    };
  }, [isConnected, setStatus, setMessage]);

  const startDialog = () => {
    rosbridge.publishDialogControl('start');
  };

  const stopDialog = () => {
    rosbridge.publishDialogControl('stop');
  };

  return { startDialog, stopDialog };
}
