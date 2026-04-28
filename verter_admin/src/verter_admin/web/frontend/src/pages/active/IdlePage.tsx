/**
 * IdlePage - Idle screen for tablet interface
 * Phase 3: Active Mode
 *
 * Shows logo, animation, and "Start" button
 * Auto-transitions to MenuPage on interaction
 */

import { useEffect, useCallback } from 'react';
import { useNavigate } from 'react-router-dom';
import { useIdleTimer } from '../../hooks/useIdleTimer';
import { ActiveLayout } from '../../components/layout/ActiveLayout';
import { useUIStore } from '../../store/uiStore';
import styles from './IdlePage.module.css';

export function IdlePage() {
  const navigate = useNavigate();
  const recordActivity = useUIStore((state) => state.recordActivity);
  const setMode = useUIStore((state) => state.setMode);

  // Idle timer is always enabled on idle page
  const { isIdle } = useIdleTimer({ enabled: true });

  // Set mode to active when entering idle page
  useEffect(() => {
    setMode('active');
  }, [setMode]);

  // Handle start button click
  const handleStart = useCallback(() => {
    recordActivity();
    navigate('/active/menu');
  }, [navigate, recordActivity]);

  // Handle voice activation (simulated - would come from ROS)
  useEffect(() => {
    // TODO: Subscribe to voice activation topic from ROS
    // When "hello" is detected, navigate to menu
  }, []);

  return (
    <ActiveLayout>
      <div className={styles.idleContainer}>
        {/* Logo placeholder */}
        <div className={styles.logo}>
          <div className={styles.logoIcon}>
            <span className={styles.logoText}>V</span>
          </div>
          <div className={styles.logoLabel}>Verter</div>
        </div>

        {/* Eyes animation */}
        <div className={styles.eyes}>
          <div className={`${styles.eye} ${styles.left}`}>
            <div className={styles.pupil}></div>
          </div>
          <div className={`${styles.eye} ${styles.right}`}>
            <div className={styles.pupil}></div>
          </div>
        </div>

        {/* Greeting */}
        <div className={styles.greeting}>
          Здравствуйте! Я могу помочь вам.
        </div>

        {/* Start Button */}
        <button
          className={styles.startButton}
          onClick={handleStart}
        >
          Начать
        </button>

        {/* Voice hint */}
        <div className={styles.voiceHint}>
          Скажите "привет", чтобы начать
        </div>
      </div>
    </ActiveLayout>
  );
}
