/**
 * FindPage - Find office location (non-navigational)
 * Phase 4: Active Mode
 */

import { useCallback, useEffect, useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { useIdleTimer } from '../../hooks/useIdleTimer';
import { ActiveLayout } from '../../components/layout/ActiveLayout';
import { BigButton } from '../../components/common/BigButton';
import { Button } from '../../components/common/Button';
import { useWaypointStore } from '../../store/waypointStore';
import { useUIStore } from '../../store/uiStore';
import styles from './FindPage.module.css';

interface WaypointWithDescription {
  name: string;
  x: number;
  y: number;
  theta: number;
  description?: string;
  floor?: string;
  building?: string;
}

// Enhanced waypoints with location info
const WAYPOINT_INFO: Record<string, { floor?: string; building?: string; description?: string }> = {
  'кардио': { floor: '2', building: 'Главное', description: '2 этаж, главное здание, коридор B' },
  'лаборатор': { floor: '1', building: 'Главное', description: '1 этаж, главное здание' },
  'регистратур': { floor: '1', building: 'Главное', description: '1 этаж, главный вход' },
};

export function FindPage() {
  const navigate = useNavigate();
  const recordActivity = useUIStore((state) => state.recordActivity);

  const { waypoints, fetchWaypoints } = useWaypointStore();
  const [selectedWaypoint, setSelectedWaypoint] = useState<WaypointWithDescription | null>(null);

  // Idle timer
  useIdleTimer({ enabled: true, timeout: 15, onIdle: () => navigate('/active/idle') });

  // Load waypoints on mount
  useEffect(() => {
    fetchWaypoints();
  }, [fetchWaypoints]);

  // Handle waypoint selection
  const handleSelect = useCallback((waypoint: WaypointWithDescription) => {
    recordActivity();
    setSelectedWaypoint(waypoint);
  }, [recordActivity]);

  // Handle back to list
  const handleBack = useCallback(() => {
    recordActivity();
    setSelectedWaypoint(null);
  }, [recordActivity]);

  // Handle guide me
  const handleGuideMe = useCallback((name: string) => {
    recordActivity();
    navigate(`/active/confirm/${encodeURIComponent(name)}`);
  }, [navigate, recordActivity]);

  // Handle menu back
  const handleMenuBack = useCallback(() => {
    recordActivity();
    navigate('/active/menu');
  }, [navigate, recordActivity]);

  // Get waypoint info
  const getWaypointInfo = useCallback((name: string) => {
    const lowerName = name.toLowerCase();
    for (const [key, info] of Object.entries(WAYPOINT_INFO)) {
      if (lowerName.includes(key)) {
        return info;
      }
    }
    return null;
  }, []);

  // Enhance waypoints with info
  const enhancedWaypoints: WaypointWithDescription[] = waypoints.map((wp) => {
    const info = getWaypointInfo(wp.name);
    return {
      ...wp,
      ...info,
    };
  });

  return (
    <ActiveLayout showBackButton onBack={selectedWaypoint ? handleBack : handleMenuBack}>
      <div className={styles.findContainer}>
        {!selectedWaypoint ? (
          <>
            {/* Title */}
            <h1 className={styles.title}>Где находится кабинет?</h1>

            {/* Waypoints List */}
            <div className={styles.waypointList}>
              {waypoints.map((waypoint) => {
                const info = getWaypointInfo(waypoint.name);
                return (
                  <button
                    key={waypoint.name}
                    className={styles.waypointItem}
                    onClick={() => handleSelect({ ...waypoint, ...info })}
                  >
                    <span className={styles.waypointIcon}>📍</span>
                    <span className={styles.waypointName}>{waypoint.name}</span>
                    <span className={styles.waypointArrow}>→</span>
                  </button>
                );
              })}
            </div>
          </>
        ) : (
          <>
            {/* Title */}
            <h1 className={styles.title}>
              {selectedWaypoint.name}
            </h1>

            {/* Info Card */}
            <div className={styles.infoCard}>
              <div className={styles.infoIcon}>📍</div>
              <div className={styles.infoContent}>
                {selectedWaypoint.description && (
                  <div className={styles.infoDescription}>
                    {selectedWaypoint.description}
                  </div>
                )}
                {(selectedWaypoint.floor || selectedWaypoint.building) && (
                  <div className={styles.infoDetails}>
                    {selectedWaypoint.building && (
                      <span className={styles.infoDetail}>
                        Здание: {selectedWaypoint.building}
                      </span>
                    )}
                    {selectedWaypoint.floor && (
                      <span className={styles.infoDetail}>
                        Этаж: {selectedWaypoint.floor}
                      </span>
                    )}
                  </div>
                )}
              </div>
            </div>

            {/* Guide Me Button */}
            <BigButton
              icon="🚶"
              label="Провести меня"
              variant="primary"
              onClick={() => handleGuideMe(selectedWaypoint.name)}
            />
          </>
        )}
      </div>
    </ActiveLayout>
  );
}
