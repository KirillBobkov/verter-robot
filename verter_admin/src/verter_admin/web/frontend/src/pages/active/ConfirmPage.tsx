/**
 * ConfirmPage - Confirmation before navigation
 * Phase 3: Active Mode
 */

import { useCallback, useState } from 'react';
import { useNavigate, useParams } from 'react-router-dom';
import { useIdleTimer } from '../../hooks/useIdleTimer';
import { ActiveLayout } from '../../components/layout/ActiveLayout';
import { Button } from '../../components/common/Button';
import { useNavigationStore } from '../../store/navigationStore';
import { navigateToWaypoint } from '../../services/rosbridge';
import { useUIStore } from '../../store/uiStore';
import styles from './ConfirmPage.module.css';

export function ConfirmPage() {
  const navigate = useNavigate();
  const { name } = useParams<{ name: string }>();
  const recordActivity = useUIStore((state) => state.recordActivity);
  const setGoal = useNavigationStore((state) => state.setGoal);
  const setStatus = useNavigationStore((state) => state.setStatus);

  const [isNavigating, setIsNavigating] = useState(false);

  const decodedName = name ? decodeURIComponent(name) : '';

  // Idle timer
  useIdleTimer({ enabled: true, timeout: 20, onIdle: () => navigate('/active/idle') });

  // Handle start navigation
  const handleStart = useCallback(async () => {
    if (!decodedName) return;

    setIsNavigating(true);
    recordActivity();

    try {
      setGoal({ name: decodedName, x: 0, y: 0, theta: 0 });
      setStatus('planning');

      const result = await navigateToWaypoint({ name: decodedName });

      if (result.success) {
        setStatus('navigating');
        navigate('/active/navigation');
      } else {
        setStatus('failed');
        alert(result.message || 'Ошибка навигации');
      }
    } catch (err) {
      setStatus('failed');
      console.error('Navigation error:', err);
    } finally {
      setIsNavigating(false);
    }
  }, [decodedName, navigate, recordActivity, setGoal, setStatus]);

  // Handle cancel
  const handleCancel = useCallback(() => {
    recordActivity();
    navigate('/active/select');
  }, [navigate, recordActivity]);

  return (
    <ActiveLayout showBackButton onBack={handleCancel}>
      <div className={styles.confirmContainer}>
        {/* Icon */}
        <div className={styles.icon}>🤖</div>

        {/* Message */}
        <div className={styles.message}>
          Я проведу вас, следуйте за мной
        </div>

        {/* Destination */}
        <div className={styles.destination}>
          <div className={styles.destinationLabel}>Цель:</div>
          <div className={styles.destinationName}>
            {decodedName}
          </div>
        </div>

        {/* Buttons */}
        <div className={styles.buttonGroup}>
          <Button
            variant="primary"
            onClick={handleStart}
            disabled={isNavigating || !decodedName}
            fullWidth
            className={styles.startButton}
          >
            {isNavigating ? 'Начинаю...' : 'Начать сопровождение'}
          </Button>
          <Button
            variant="secondary"
            onClick={handleCancel}
            fullWidth
          >
            Отмена
          </Button>
        </div>
      </div>
    </ActiveLayout>
  );
}
