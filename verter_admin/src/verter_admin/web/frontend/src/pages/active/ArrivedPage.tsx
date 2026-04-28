/**
 * ArrivedPage - Destination reached
 * Phase 3: Active Mode
 *
 * Shows arrival message and auto-returns to idle
 */

import { useCallback, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import { useNavigationStore } from '../../store/navigationStore';
import styles from './ArrivedPage.module.css';

export function ArrivedPage() {
  const navigate = useNavigate();

  const currentGoal = useNavigationStore((state) => state.currentGoal);
  const reset = useNavigationStore((state) => state.reset);

  // Auto-return to idle after 5 seconds
  useEffect(() => {
    const timer = setTimeout(() => {
      handleReturn();
    }, 5000);

    return () => clearTimeout(timer);
  }, []);

  // Handle return to idle
  const handleReturn = useCallback(() => {
    reset();
    navigate('/active/idle');
  }, [navigate, reset]);

  return (
    <div className={styles.arrivedContainer}>
      {/* Success Icon */}
      <div className={styles.icon}>✓</div>

      {/* Title */}
      <h1 className={styles.title}>Мы на месте!</h1>

      {/* Destination */}
      {currentGoal && (
        <div className={styles.destination}>
          {currentGoal.name}
        </div>
      )}

      {/* Animation */}
      <div className={styles.animation}>
        <div className={styles.checkmark}></div>
      </div>

      {/* Auto-return message */}
      <div className={styles.autoReturn}>
        Автовозврат через 5 секунд...
      </div>

      {/* Manual return button */}
      <button
        className={styles.returnButton}
        onClick={handleReturn}
      >
        Вернуться в начало
      </button>
    </div>
  );
}
