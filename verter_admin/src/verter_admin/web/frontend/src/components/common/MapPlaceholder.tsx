/**
 * MapPlaceholder - Placeholder for map visualization
 * Will be replaced with actual map component in future iteration
 */

import styles from './MapPlaceholder.module.css';

export interface MapPlaceholderProps {
  width?: string | number;
  height?: string | number;
  showRobot?: boolean;
  robotPose?: { x: number; y: number; yaw: number };
  showGrid?: boolean;
  className?: string;
}

export function MapPlaceholder({
  width = '100%',
  height = '400px',
  showRobot = false,
  robotPose,
  showGrid = true,
  className,
}: MapPlaceholderProps) {
  return (
    <div
      className={`${styles.placeholder} ${className || ''}`}
      style={{ width, height }}
    >
      {showGrid && <div className={styles.grid}></div>}

      {showRobot && robotPose && (
        <div
          className={styles.robot}
          style={{
            left: `${50 + robotPose.x * 10}%`, // Simple scale for demo
            top: `${50 - robotPose.y * 10}%`,
            transform: `translate(-50%, -50%) rotate(${-robotPose.yaw}rad)`,
          }}
        >
          <div className={styles.robotArrow}></div>
        </div>
      )}

      <div className={styles.label}>
        {showRobot ? 'Map (coming soon)' : 'Map Placeholder'}
      </div>
    </div>
  );
}
