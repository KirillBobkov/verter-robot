/**
 * RobotAvatar — CSS-only аватар робота с 5 анимациями.
 * Состояние определяется через data-avatar: idle/listening/thinking/speaking/sad.
 * errorType приоритетнее status → sad сохраняется до listening/idle (логика в store).
 */

import type { DialogStatus, ErrorType } from '../../types/dialog';
import styles from './RobotAvatar.module.css';

interface RobotAvatarProps {
  status: DialogStatus;
  errorType: ErrorType | null;
}

export default function RobotAvatar({ status, errorType }: RobotAvatarProps) {
  const avatarState = errorType ? 'sad' : status;

  return (
    <div className={styles.avatar} data-avatar={avatarState} aria-hidden="true">
      <div className={styles.eyes}>
        <span className={styles.eye} />
        <span className={styles.eye} />
      </div>
    </div>
  );
}
