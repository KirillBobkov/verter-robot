/**
 * HomePageContent - Content component for HomePage
 * Reusable by both SetupLayout and ServiceLayout
 */

import { useState, useCallback, useEffect } from 'react';
import { usePose } from '../../hooks/usePose';
import { useCmdVel } from '../../hooks/useCmdVel';
import { useWaypointStore } from '../../store/waypointStore';
import { MapPlaceholder } from '../../components/common/MapPlaceholder';
import { Button } from '../../components/common/Button';
import { formatAngle } from '../../utils/quaternion';
import styles from './HomePage.module.css';

export function HomePageContent() {
  const { pose, isSubscribed, refresh } = usePose({ enabled: true });
  const { publishVelocity, stop } = useCmdVel();
  const { addWaypoint } = useWaypointStore();

  const [isRotating, setIsRotating] = useState(false);
  const [rotationDirection, setRotationDirection] = useState<1 | -1 | null>(null);
  const [isSaving, setIsSaving] = useState(false);

  // Rotation control
  useEffect(() => {
    if (!isRotating || rotationDirection === null) {
      stop();
      return;
    }

    const interval = setInterval(() => {
      publishVelocity(0, rotationDirection * 0.3); // 0.3 rad/s rotation
    }, 100);

    return () => {
      clearInterval(interval);
      stop();
    };
  }, [isRotating, rotationDirection, publishVelocity, stop]);

  // Handle rotation button
  const handleRotateStart = useCallback((direction: 1 | -1) => {
    setRotationDirection(direction);
    setIsRotating(true);
  }, []);

  const handleRotateStop = useCallback(() => {
    setIsRotating(false);
    setRotationDirection(null);
  }, []);

  // Set home position
  const handleSetHome = useCallback(async () => {
    setIsSaving(true);
    try {
      await addWaypoint({
        name: 'home',
        x: pose.x,
        y: pose.y,
        theta: pose.yaw,
      });
      // Success will be shown via waypoint store
    } finally {
      setIsSaving(false);
    }
  }, [pose, addWaypoint]);

  // Test return home
  const handleTestReturn = useCallback(async () => {
    // TODO: Implement navigation to 'home' waypoint
    console.log('Navigate to home');
  }, []);

  // Refresh pose
  const handleRefreshPose = useCallback(async () => {
    await refresh();
  }, [refresh]);

  return (
    <div className={styles.content}>
      {/* Map Display */}
      <section className={styles.mapSection}>
        <MapPlaceholder
          width="100%"
          height="350px"
          showRobot
          showGrid
          robotPose={isSubscribed ? pose : undefined}
        />
      </section>

      {/* Current Position Display */}
      <section className={styles.section}>
        <h2 className={styles.sectionTitle}>Текущая позиция</h2>

        <div className={styles.poseDisplay}>
          <div className={styles.poseRow}>
            <span className={styles.poseLabel}>X:</span>
            <span className={styles.poseValue}>{pose.x.toFixed(3)} м</span>
          </div>
          <div className={styles.poseRow}>
            <span className={styles.poseLabel}>Y:</span>
            <span className={styles.poseValue}>{pose.y.toFixed(3)} м</span>
          </div>
          <div className={styles.poseRow}>
            <span className={styles.poseLabel}>Yaw:</span>
            <span className={styles.poseValue}>{formatAngle(pose.yaw)}</span>
          </div>
        </div>

        {!isSubscribed && (
          <Button
            variant="secondary"
            onClick={handleRefreshPose}
            className={styles.refreshButton}
          >
            Загрузка...
          </Button>
        )}
      </section>

      {/* Orientation Adjust */}
      <section className={styles.section}>
        <h2 className={styles.sectionTitle}>Ориентация</h2>

        <div className={styles.rotationControls}>
          <Button
            variant="secondary"
            onMouseDown={() => handleRotateStart(1)}
            onMouseUp={handleRotateStop}
            onMouseLeave={handleRotateStop}
            onTouchStart={(e) => { e.preventDefault(); handleRotateStart(1); }}
            onTouchEnd={handleRotateStop}
            className={styles.rotateButton}
          >
            ← Влево
          </Button>
          <Button
            variant="secondary"
            onMouseDown={() => handleRotateStart(-1)}
            onMouseUp={handleRotateStop}
            onMouseLeave={handleRotateStop}
            onTouchStart={(e) => { e.preventDefault(); handleRotateStart(-1); }}
            onTouchEnd={handleRotateStop}
            className={styles.rotateButton}
          >
            Вправо →
          </Button>
        </div>
        <p className={styles.rotateHint}>
          Удерживайте кнопку для вращения
        </p>
      </section>

      {/* Home Actions */}
      <section className={styles.section}>
        <div className={styles.buttonGroup}>
          <Button
            variant="primary"
            onClick={handleSetHome}
            disabled={isSaving || !isSubscribed}
          >
            {isSaving ? 'Сохранение...' : 'Установить позицию'}
          </Button>
          <Button
            variant="secondary"
            onClick={handleTestReturn}
          >
            Тестовый возврат
          </Button>
        </div>
      </section>
    </div>
  );
}
