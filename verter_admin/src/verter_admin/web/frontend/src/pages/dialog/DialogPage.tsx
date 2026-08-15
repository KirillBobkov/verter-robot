/**
 * DialogPage — fullscreen диалоговый kiosk-экран.
 * Композиция: аватар / зона сообщения / кнопки управления.
 */

import { useDialogStore } from '../../store/dialogStore';
import { useDialogROS } from '../../hooks/useDialogROS';
import RobotAvatar from '../../components/dialog/RobotAvatar';
import MessageZone from '../../components/dialog/MessageZone';
import DialogControls from '../../components/dialog/DialogControls';
import styles from './DialogPage.module.css';

export default function DialogPage() {
  const status = useDialogStore((s) => s.status);
  const errorType = useDialogStore((s) => s.errorType);
  const currentMessage = useDialogStore((s) => s.currentMessage);
  const isDialogActive = status !== 'idle';
  const { startDialog, stopDialog } = useDialogROS();

  return (
    <main
      className={styles.page}
      data-status={status}
      data-error={errorType ?? undefined}
    >
      <div className={styles.avatarWrap}>
        <RobotAvatar status={status} errorType={errorType} />
      </div>

      <MessageZone message={currentMessage} />

      <div className={styles.controlsWrap}>
        <DialogControls
          isDialogActive={isDialogActive}
          onStart={startDialog}
          onStop={stopDialog}
        />
      </div>
    </main>
  );
}
