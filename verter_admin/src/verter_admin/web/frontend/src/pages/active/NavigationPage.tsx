/**
 * NavigationPage - Active navigation screen
 * Phase 3: Active Mode
 *
 * Shows robot is moving to destination
 */

import { useCallback, useEffect, useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { useIdleTimer } from '../../hooks/useIdleTimer';
import { ActiveLayout } from '../../components/layout/ActiveLayout';
import { MapPlaceholder } from '../../components/common/MapPlaceholder';
import { useNavigationStore } from '../../store/navigationStore';
import { stopRobot } from '../../services/rosbridge';
import styles from './NavigationPage.module.css';

export function NavigationPage() {
  const navigate = useNavigate();

  const currentGoal = useNavigationStore((state) => state.currentGoal);
  const status = useNavigationStore((state) => state.status);
  const progress = useNavigationStore((state) => state.progress);
  const setProgress = useNavigationStore((state) => state.setProgress);

  const [timeElapsed, setTimeElapsed] = useState(0);

  // Idle timer disabled during navigation
  useIdleTimer({ enabled: false });

  // Simulate progress (in real app, would come from navigation feedback)
  useEffect(() => {
    if (status === 'navigating') {
      const interval = setInterval(() => {
        setTimeElapsed((t) => {
          const newTime = t + 1;
          setProgress(Math.min(100, newTime * 2)); // Demo: 2% per second
          return newTime;
        });
      }, 1000);

      // Simulate arrival after 50 seconds
      const arrivalTimer = setTimeout(() => {
        navigate('/active/arrived');
      }, 50000);

      return () => {
        clearInterval(interval);
        clearTimeout(arrivalTimer);
      };
    }
  }, [status, setProgress, navigate]);

  // Handle emergency stop (in addition to main button)
  const handleStop = useCallback(() => {
    stopRobot();
    navigate('/active/menu');
  }, [navigate]);

  // Handle arrival (manual trigger)
  const handleArrived = useCallback(() => {
    navigate('/active/arrived');
  }, [navigate]);

  return (
    <ActiveLayout>
      <div className={styles.navigationContainer}>
        {/* Title */}
        <h1 className={styles.title}>Провожу вас...</h1>

        {/* Mini Map */}
        <div className={styles.mapWrapper}>
          <MapPlaceholder
            width="100%"
            height="250px"
            showRobot
            showGrid
          />
          {/* Progress overlay */}
          <div className={styles.progressOverlay}>
            <div className={styles.progressText}>
              {Math.round(progress)}%
            </div>
          </div>
        </div>

        {/* Message */}
        <div className={styles.message}>Следуйте за мной</div>

        {/* Destination info */}
        {currentGoal && (
          <div className={styles.destination}>
            → {currentGoal.name}
          </div>
        )}

        {/* Progress bar */}
        <div className={styles.progressBar}>
          <div
            className={styles.progressFill}
            style={{ width: `${progress}%` }}
          />
        </div>

        {/* Manual arrive button (for testing) */}
        <button
          className={styles.arriveButton}
          onClick={handleArrived}
        >
          Мы на месте (тест)
        </button>
      </div>
    </ActiveLayout>
  );
}
