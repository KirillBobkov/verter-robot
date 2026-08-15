/**
 * emulationService — реализация transport-контракта (тот же публичный интерфейс,
 * что у rosbridge.ts) для режима эмуляции. См. config/emulation.ts.
 *
 * Минимальная эмуляция: никаких авто-сценариев и таймингов. Панель состояний
 * (EmulationPanel) вручную эмитит dialog_status / ai_question / text_to_speech;
 * кнопка «Начать диалог» ставит listening, «Остановить» — idle. Хуки
 * (useROS, useDialogROS) работают без изменений — меняется только путь импорта
 * через фасад transport.ts.
 */

import type { RosStatus } from '../types/ros';

type TopicListener = (data: string) => void;
type StatusListener = (status: RosStatus) => void;

class EmulationController {
  private statusListener: TopicListener | null = null;
  private questionListener: TopicListener | null = null;
  private ttsListener: TopicListener | null = null;
  private statusCb: StatusListener | null = null;

  setStatusListener(cb: TopicListener) { this.statusListener = cb; }
  setQuestionListener(cb: TopicListener) { this.questionListener = cb; }
  setTtsListener(cb: TopicListener) { this.ttsListener = cb; }
  setStatusCb(cb: StatusListener) { this.statusCb = cb; }

  /** dialog_status → меняет статус (аватар + облачко). */
  emitStatus(raw: string) {
    this.statusListener?.(raw);
  }

  /** ai_question → распознанный вопрос пользователя. */
  emitQuestion(text: string) {
    this.questionListener?.(text);
  }

  /** text_to_speech → ответ/ошибка/прощание (роль в useDialogROS по статусу). */
  emitTts(text: string) {
    this.ttsListener?.(text);
  }

  notifyConnected() { this.statusCb?.('connected'); }
  notifyDisconnected() { this.statusCb?.('disconnected'); }
}

export const controller = new EmulationController();

// ============ Transport-контракт (совпадает с rosbridge.ts) ============

export function getRosbridgeUrl(): string {
  return 'ws://emulation';
}

export function initROS(_url?: string): Promise<void> {
  setTimeout(() => controller.notifyConnected(), 50);
  return Promise.resolve();
}

export function onStatus(cb: StatusListener): () => void {
  controller.setStatusCb(cb);
  return () => { controller.setStatusCb(() => {}); };
}

export function closeROS(): void {
  controller.notifyDisconnected();
}

export function subscribeDialogStatus(listener: (status: string) => void): () => void {
  controller.setStatusListener(listener);
  return () => { controller.setStatusListener(() => {}); };
}

export function subscribeAIQuestion(listener: (question: string) => void): () => void {
  controller.setQuestionListener(listener);
  return () => { controller.setQuestionListener(() => {}); };
}

export function subscribeTextToSpeech(listener: (text: string) => void): () => void {
  controller.setTtsListener(listener);
  return () => { controller.setTtsListener(() => {}); };
}

/** start → listening, stop → idle (кнопки основного UI). */
export function publishDialogControl(command: 'start' | 'stop'): void {
  controller.emitStatus(command === 'start' ? 'listening' : 'idle');
}

// Не используются хуками диалога напрямую — оставлены для контракта.
export function publishString(_topicName: string, _data: string): void { /* no-op */ }
export function subscribeString(_topicName: string, listener: (data: string) => void): () => void {
  return () => { void listener; };
}
