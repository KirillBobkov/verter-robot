/**
 * LocationForm - Form for adding/editing waypoints
 */

import { useState, useEffect, useCallback } from 'react';
import { usePose } from '../../hooks/usePose';
import { Button } from '../common/Button';
import { formatAngle } from '../../utils/quaternion';
import type { Waypoint } from '../../types/ros';
import styles from './LocationForm.module.css';

export interface LocationFormProps {
  onSubmit: (waypoint: Waypoint) => Promise<void>;
  onCancel?: () => void;
  initialData?: Partial<Waypoint>;
  submitLabel?: string;
}

export function LocationForm({
  onSubmit,
  onCancel,
  initialData,
  submitLabel,
}: LocationFormProps) {
  const { pose, isSubscribed } = usePose({ enabled: true });

  const [name, setName] = useState(initialData?.name || '');
  const [description, setDescription] = useState(initialData?.description || '');
  const [useCurrentPose, setUseCurrentPose] = useState(true);
  const [x, setX] = useState(initialData?.x?.toString() || '');
  const [y, setY] = useState(initialData?.y?.toString() || '');
  const [theta, setTheta] = useState(initialData?.theta?.toString() || '');
  const [isSubmitting, setIsSubmitting] = useState(false);

  // Update form fields when current pose changes
  // Only update if user is not currently editing the inputs
  useEffect(() => {
    if (useCurrentPose && isSubscribed && document.activeElement) {
      const activeId = document.activeElement.id;
      // Check if any input is currently focused
      if (activeId !== 'x-input' && activeId !== 'y-input' && activeId !== 'theta-input') {
        setX(pose.x.toFixed(3));
        setY(pose.y.toFixed(3));
        setTheta(pose.yaw.toFixed(3));
      }
    }
  }, [pose, isSubscribed, useCurrentPose]);

  // Handle use current pose toggle
  const handleUseCurrentPose = useCallback(() => {
    if (!useCurrentPose && isSubscribed) {
      setX(pose.x.toFixed(3));
      setY(pose.y.toFixed(3));
      setTheta(pose.yaw.toFixed(3));
    }
    setUseCurrentPose(!useCurrentPose);
  }, [useCurrentPose, isSubscribed, pose]);

  // Handle form submit
  const handleSubmit = useCallback(async (e: React.FormEvent) => {
    e.preventDefault();

    if (!name.trim()) {
      return;
    }

    setIsSubmitting(true);
    try {
      await onSubmit({
        name: name.trim(),
        x: parseFloat(x) || 0,
        y: parseFloat(y) || 0,
        theta: parseFloat(theta) || 0,
        description: description.trim() || undefined,
      });

      // Reset form on success
      setName('');
      setDescription('');
      setX('');
      setY('');
      setTheta('');
    } finally {
      setIsSubmitting(false);
    }
  }, [name, x, y, theta, description, onSubmit]);

  return (
    <form className={styles.form} onSubmit={handleSubmit}>
      {/* Name */}
      <div className={styles.field}>
        <label className={styles.label}>Название</label>
        <input
          type="text"
          value={name}
          onChange={(e) => setName(e.target.value)}
          placeholder="Например: Кардиология 214"
          className={styles.input}
          required
        />
      </div>

      {/* Description */}
      <div className={styles.field}>
        <label className={styles.label}>Описание</label>
        <input
          type="text"
          value={description}
          onChange={(e) => setDescription(e.target.value)}
          placeholder="2 этаж, главное здание"
          className={styles.input}
        />
      </div>

      {/* Use Current Pose */}
      <div className={styles.field}>
        <label className={styles.checkbox}>
          <input
            type="checkbox"
            checked={useCurrentPose}
            onChange={handleUseCurrentPose}
          />
          <span>Использовать текущую позицию</span>
        </label>
      </div>

      {/* Pose Display */}
      <div className={styles.poseGrid}>
        <div className={styles.poseField}>
          <label className={styles.poseLabel}>X (м)</label>
          <input
            id="x-input"
            type="number"
            step="0.001"
            value={x}
            onChange={(e) => setX(e.target.value)}
            className={styles.input}
            disabled={useCurrentPose}
            required
          />
        </div>
        <div className={styles.poseField}>
          <label className={styles.poseLabel}>Y (м)</label>
          <input
            id="y-input"
            type="number"
            step="0.001"
            value={y}
            onChange={(e) => setY(e.target.value)}
            className={styles.input}
            disabled={useCurrentPose}
            required
          />
        </div>
        <div className={styles.poseField}>
          <label className={styles.poseLabel}>Поворот (°)</label>
          <input
            id="theta-input"
            type="number"
            step="0.1"
            value={theta}
            onChange={(e) => setTheta(e.target.value)}
            className={styles.input}
            disabled={useCurrentPose}
            required
          />
        </div>
      </div>

      {/* Current Pose Preview */}
      {isSubscribed && (
        <div className={styles.currentPose}>
          <span className={styles.currentPoseLabel}>Текущая позиция:</span>
          <span className={styles.currentPoseValue}>
            X: {pose.x.toFixed(2)}, Y: {pose.y.toFixed(2)}, Поворот: {formatAngle(pose.yaw)}
          </span>
        </div>
      )}

      {/* Actions */}
      <div className={styles.actions}>
        <Button
          type="submit"
          variant="primary"
          disabled={isSubmitting || !name.trim()}
        >
          {isSubmitting ? 'Сохранение...' : (submitLabel || 'Сохранить точку')}
        </Button>
        {onCancel && (
          <Button
            type="button"
            variant="secondary"
            onClick={onCancel}
          >
            Отмена
          </Button>
        )}
      </div>
    </form>
  );
}
