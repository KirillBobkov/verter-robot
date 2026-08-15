/**
 * EmulationPanel — сервисная панель режима эмуляции.
 * position: fixed bottom-left, мелкий моноширинный шрифт. Не рендерится в обычном
 * режиме (App.tsx гейтит по isEmulation + lazy-импорт → не раздувает прод-бандл).
 *
 * Простая и понятная модель — никаких авто-сценариев:
 *  1. Переключатель состояния — 7 статусов из плана (idle/listening/thinking/
 *     speaking/error:timeout/error:network/error:unavailable). Клик ставит
 *     dialog_status → синхронно меняется и аватар, и облачко (через dialogStore).
 *     Активный статус подсвечен из store — панель всегда отражает реальное
 *     состояние экрана.
 *  2. Поле сообщения — один инпут + кнопка «показать». Топик и роль определяются
 *     текущим статусом автоматически:
 *       thinking   → ai_question   (облачко «ВЫ СПРОСИЛИ» + «Думаю»)
 *       speaking   → text_to_speech(роль answer — «РОБОТ ОТВЕЧАЕТ»)
 *       error:*    → text_to_speech(роль error — «ОШИБКА»)
 *       idle       → text_to_speech(роль farewell — озвучится, не покажется)
 *       listening  → сообщение фиксировано («Говорите, я слушаю…»), поле скрыто
 */

import { useState } from 'react';
import { controller } from '../../services/emulationService';
import { useDialogStore } from '../../store/dialogStore';
import styles from './EmulationPanel.module.css';

/** Состояния из плана (DIALOG_FLOW §3). */
const STATUSES = [
  'idle',
  'listening',
  'thinking',
  'speaking',
  'error:timeout',
  'error:network',
  'error:unavailable',
] as const;

/** Что делает кнопка «показать» для данного статуса. */
function messageActionFor(
  composite: string,
): { kind: 'question' | 'answer' | 'error' | 'farewell' | 'none'; label: string; placeholder: string } {
  if (composite === 'thinking') {
    return { kind: 'question', label: 'Вопрос пользователя', placeholder: 'текст вопроса…' };
  }
  if (composite === 'speaking') {
    return { kind: 'answer', label: 'Ответ робота', placeholder: 'текст ответа…' };
  }
  if (composite.startsWith('error:')) {
    return { kind: 'error', label: 'Текст ошибки', placeholder: 'что говорит робот…' };
  }
  if (composite === 'idle') {
    return { kind: 'farewell', label: 'Прощание (озвучится, без облачка)', placeholder: 'Рад был помочь!' };
  }
  // listening — текст фиксирован, вписывать нечего.
  return { kind: 'none', label: '', placeholder: '' };
}

export default function EmulationPanel() {
  // Реальное состояние экрана — для подсветки активной кнопки (синхронизация).
  const status = useDialogStore((s) => s.status);
  const errorType = useDialogStore((s) => s.errorType);
  const composite = errorType ? `error:${errorType}` : status;

  const [text, setText] = useState('');

  const action = messageActionFor(composite);

  const pickStatus = (value: string) => {
    controller.emitStatus(value);
  };

  const showMessage = () => {
    if (!text || action.kind === 'none') return;
    if (action.kind === 'question') {
      controller.emitQuestion(text);
    } else {
      // answer / error / farewell — всё через text_to_speech,
      // роль в useDialogROS определится по status/errorType.
      controller.emitTts(text);
    }
    setText('');
  };

  return (
    <div className={styles.panel}>
      <div className={styles.title}>EMU · состояния</div>

      <div className={styles.section}>
        <span className={styles.label}>Состояние (аватар + облачко):</span>
        <div className={styles.statusGrid}>
          {STATUSES.map((s) => (
            <button
              key={s}
              className={`${styles.btn} ${composite === s ? styles.btnActive : ''}`}
              onClick={() => pickStatus(s)}
            >
              {s}
            </button>
          ))}
        </div>
      </div>

      {action.kind !== 'none' && (
        <div className={styles.section}>
          <label className={styles.label} htmlFor="emu-msg">
            {action.label}:
          </label>
          <input
            id="emu-msg"
            className={styles.input}
            value={text}
            placeholder={action.placeholder}
            onChange={(e) => setText(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === 'Enter') showMessage();
            }}
          />
          <button className={styles.btn} onClick={showMessage} disabled={!text}>
            показать
          </button>
        </div>
      )}

      {action.kind === 'none' && (
        <div className={styles.hint}>
          listening: сообщение фиксировано — «Говорите, я слушаю…»
        </div>
      )}
    </div>
  );
}
