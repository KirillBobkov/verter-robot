/**
 * DialogControls — крупные touch-кнопки.
 * В idle — «Начать диалог» (accent). В активном диалоге — «Остановить» (danger).
 */

import styles from './DialogControls.module.css';

interface DialogControlsProps {
  isDialogActive: boolean;
  onStart: () => void;
  onStop: () => void;
}

export default function DialogControls({
  isDialogActive,
  onStart,
  onStop,
}: DialogControlsProps) {
  return (
    <div className={styles.controls}>
      {!isDialogActive ? (
        <button
          className={`${styles.btn} ${styles.btnStart}`}
          onClick={onStart}
          aria-label="Начать диалог"
        >
          Начать диалог
        </button>
      ) : (
        <button
          className={`${styles.btn} ${styles.btnStop}`}
          onClick={onStop}
          aria-label="Остановить диалог"
        >
          Остановить
        </button>
      )}
    </div>
  );
}
