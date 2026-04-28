/**
 * SelectPage - Select destination (cabinet/office)
 * Phase 3: Active Mode
 */

import { useCallback, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import { useIdleTimer } from '../../hooks/useIdleTimer';
import { ActiveLayout } from '../../components/layout/ActiveLayout';
import { BigButton } from '../../components/common/BigButton';
import { useWaypointStore } from '../../store/waypointStore';
import { useUIStore } from '../../store/uiStore';
import styles from './SelectPage.module.css';

interface WaypointWithIcon {
  name: string;
  x: number;
  y: number;
  theta: number;
  icon: string;
}

// Icon mapping based on name keywords
function getIconForWaypoint(name: string): string {
  const lowerName = name.toLowerCase();

  if (lowerName.includes('кардиолог') || lowerName.includes('cardio')) return '🏥';
  if (lowerName.includes('лаборатор') || lowerName.includes('lab')) return '🔬';
  if (lowerName.includes('регистратур') || lowerName.includes('reception')) return '🫂';
  if (lowerName.includes('рентген') || lowerName.includes('x-ray') || lowerName.includes('радиолог')) return '🩻';
  if (lowerName.includes('хирург') || lowerName.includes('surgery')) return '🏥';
  if (lowerName.includes('аптек') || lowerName.includes('pharm')) return '💊';
  if (lowerName.includes('процедур') || lowerName.includes('procedure')) return '💉';
  if (lowerName.includes('врач') || lowerName.includes('doctor')) return '👨‍⚕️';

  return '📍'; // Default icon
}

export function SelectPage() {
  const navigate = useNavigate();
  const recordActivity = useUIStore((state) => state.recordActivity);

  const { waypoints, loading, fetchWaypoints } = useWaypointStore();

  // Idle timer
  useIdleTimer({ enabled: true, timeout: 15, onIdle: () => navigate('/active/idle') });

  // Load waypoints on mount
  useEffect(() => {
    fetchWaypoints();
  }, [fetchWaypoints]);

  // Handle waypoint selection
  const handleSelect = useCallback((name: string) => {
    recordActivity();
    navigate(`/active/confirm/${encodeURIComponent(name)}`);
  }, [navigate, recordActivity]);

  // Handle back
  const handleBack = useCallback(() => {
    recordActivity();
    navigate('/active/menu');
  }, [navigate, recordActivity]);

  // Map waypoints to display items
  const displayItems: WaypointWithIcon[] = waypoints.map((wp) => ({
    ...wp,
    icon: getIconForWaypoint(wp.name),
  }));

  return (
    <ActiveLayout showBackButton onBack={handleBack}>
      <div className={styles.selectContainer}>
        {/* Title */}
        <h1 className={styles.title}>Выберите кабинет</h1>

        {/* Waypoints List */}
        {loading ? (
          <div className={styles.loading}>Загрузка...</div>
        ) : displayItems.length === 0 ? (
          <div className={styles.empty}>Нет доступных кабинетов</div>
        ) : (
          <div className={styles.waypointGrid}>
            {displayItems.map((item) => (
              <BigButton
                key={item.name}
                icon={item.icon}
                label={item.name}
                variant="secondary"
                onClick={() => handleSelect(item.name)}
              />
            ))}
          </div>
        )}
      </div>
    </ActiveLayout>
  );
}
