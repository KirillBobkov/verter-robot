/**
 * LocationList - List of saved waypoints with edit/delete actions
 */

import { useCallback } from 'react';
import type { Waypoint } from '../../types/ros';
import { Button } from '../common/Button';
import { formatAngle } from '../../utils/quaternion';
import styles from './LocationList.module.css';

export interface LocationListProps {
  waypoints: Waypoint[];
  onEdit?: (waypoint: Waypoint) => void;
  onDelete?: (name: string) => Promise<void>;
  loading?: boolean;
}

export function LocationList({ waypoints, onEdit, onDelete, loading }: LocationListProps) {
  const handleDelete = useCallback(async (name: string) => {
    if (confirm(`Удалить точку "${name}"?`)) {
      await onDelete?.(name);
    }
  }, [onDelete]);

  if (loading) {
    return (
      <div className={styles.empty}>Загрузка...</div>
    );
  }

  if (waypoints.length === 0) {
    return (
      <div className={styles.empty}>Нет сохранённых точек</div>
    );
  }

  return (
    <div className={styles.list}>
      {waypoints.map((waypoint) => (
        <div key={waypoint.name} className={styles.item}>
          <div className={styles.info}>
            <div className={styles.name}>
              {waypoint.description ? '📍 ' : ''}
              {waypoint.name}
            </div>
            <div className={styles.details}>
              X: {waypoint.x.toFixed(2)}, Y: {waypoint.y.toFixed(2)}, Поворот: {formatAngle(waypoint.theta)}
            </div>
            {waypoint.description && (
              <div className={styles.description}>{waypoint.description}</div>
            )}
          </div>
          <div className={styles.actions}>
            {onEdit && (
              <Button
                variant="secondary"
                size="sm"
                onClick={() => onEdit(waypoint)}
              >
                Изменить
              </Button>
            )}
            {onDelete && (
              <Button
                variant="danger"
                size="sm"
                onClick={() => handleDelete(waypoint.name)}
              >
                Удалить
              </Button>
            )}
          </div>
        </div>
      ))}
    </div>
  );
}
