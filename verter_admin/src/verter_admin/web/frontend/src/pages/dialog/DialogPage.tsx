/**
 * DialogPage — fullscreen диалоговый kiosk-экран.
 *
 * Простая модель: для каждого статуса из плана — своё состояние облачка,
 * аватар берёт тот же статус. Фронтенд «просто рисует статусы».
 *
 *  - idle            → welcome «Нажми на кнопку чтобы начать диалог».
 *  - listening       → «Говорите, я слушаю…».
 *  - thinking        → есть вопрос: вопрос + «Думаю…» ниже; иначе — «Думаю…».
 *  - speaking        → есть ответ/ошибка: ответ; иначе — «Формулирую ответ…».
 *  - error:*         → есть текст ошибки: ошибка; иначе — заглушка ошибки.
 *
 * Кнопка — снизу: «Начать диалог» в idle, «Остановить» в active.
 */

import { useDialogStore } from '../../store/dialogStore';
import { useDialogROS } from '../../hooks/useDialogROS';
import RobotAvatar from '../../components/dialog/RobotAvatar';
import MessageZone from '../../components/dialog/MessageZone';
import DialogControls from '../../components/dialog/DialogControls';
import styles from './DialogPage.module.css';
import type { DialogMessage, DialogStatus } from '../../types/dialog';

// ============ Фиксированные сообщения (заглушки) ============

const WELCOME: DialogMessage = { role: 'welcome', text: 'Нажми на кнопку чтобы начать диалог' };
const LISTENING: DialogMessage = { role: 'listening', text: 'Говорите, я слушаю…' };
const THINKING: DialogMessage = { role: 'thinking', text: 'Думаю…' };
const FORMULATING: DialogMessage = { role: 'speaking', text: 'Формулирую ответ…' };
const ERROR_FALLBACK: DialogMessage = {
  role: 'error',
  text: 'Что-то пошло не так. Попробуйте ещё раз.',
};

/**
 * Что рисовать в облачке для данного статуса. Детерминированно: статус → вид.
 * `currentMessage` используется только если его роль подходит под статус
 * (пришедший ответ/ошибка/вопрос); иначе — фиксированная заглушка. Старый текст
 * чужой роли никогда не «торчит» в новом статусе.
 */
function viewFor(
  status: DialogStatus,
  currentMessage: DialogMessage | null,
): { message: DialogMessage; subMessage?: DialogMessage } {
  switch (status) {
    case 'idle':
      return { message: WELCOME };
    case 'listening':
      return { message: LISTENING };
    case 'thinking':
      // Есть распознанный вопрос — вопрос + «Думаю…» ниже.
      if (currentMessage?.role === 'question') {
        return { message: currentMessage, subMessage: THINKING };
      }
      // Иначе — заглушка. Чужая роль (ответ/ошибка) в thinking не торчит:
      // статус = thinking → рисуем «Думаю…».
      return { message: THINKING };
    case 'speaking':
      // Ответ робота. Чужая роль (ошибка) в speaking не торчит: только answer,
      // иначе — заглушка «Формулирую ответ…».
      return { message: currentMessage?.role === 'answer' ? currentMessage : FORMULATING };
    case 'error':
      // Текст ошибки, если пришёл; иначе — заглушка.
      return { message: currentMessage?.role === 'error' ? currentMessage : ERROR_FALLBACK };
    default:
      return { message: WELCOME };
  }
}

function sourceOf(message: DialogMessage | null | undefined): 'ai' | 'user' | 'neutral' {
  if (!message) return 'neutral';
  if (message.role === 'question') return 'user';
  return 'ai';
}

export default function DialogPage() {
  const status = useDialogStore((s) => s.status);
  const errorType = useDialogStore((s) => s.errorType);
  const currentMessage = useDialogStore((s) => s.currentMessage);
  const isDialogActive = status !== 'idle';
  const { startDialog, stopDialog } = useDialogROS();

  const view = viewFor(status, currentMessage);

  return (
    <main
      className={styles.page}
      data-status={status}
      data-error={errorType ?? undefined}
      data-source={sourceOf(view.message)}
    >
      <section className={styles.stage}>
        <div className={styles.avatarCol}>
          <RobotAvatar status={status} />
          <DialogControls
            isDialogActive={isDialogActive}
            onStart={startDialog}
            onStop={stopDialog}
          />
        </div>
        <div className={styles.messageCol}>
          <MessageZone message={view.message} subMessage={view.subMessage} />
        </div>
      </section>
    </main>
  );
}
