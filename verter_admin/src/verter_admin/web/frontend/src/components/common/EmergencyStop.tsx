/**
 * EmergencyStop component - Material Design 3
 * Extended FAB for emergency robot stop - always visible in active mode
 */

import { useCallback } from 'react';
import { useCmdVel } from '../../hooks/useCmdVel';
import styles from './EmergencyStop.module.css';

export function EmergencyStop() {
  const { stop } = useCmdVel();

  const handleEmergencyStop = useCallback(() => {
    // Stop the robot immediately
    stop();

    // Log emergency stop activation
    console.log('[EMERGENCY STOP] Activated at:', new Date().toISOString());
  }, [stop]);

  return (
    <button
      className={styles.emergencyStop}
      onClick={handleEmergencyStop}
      aria-label="Аварийная остановка робота"
      type="button"
    >
      <span className={styles.icon} aria-hidden="true">⬛</span>
      <span className={styles.text}>СТОП</span>
    </button>
  );
}
