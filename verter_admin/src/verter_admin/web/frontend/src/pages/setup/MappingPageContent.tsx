/**
 * MappingPageContent - Content component for MappingPage
 * Reusable by both SetupLayout and ServiceLayout
 */

import { useState, useCallback, useEffect } from 'react';
import { useCmdVel } from '../../hooks/useCmdVel';
import { StatusLog, useStatusLog } from '../../components/common/StatusLog';
import { MapPlaceholder } from '../../components/common/MapPlaceholder';
import { Button } from '../../components/common/Button';
import styles from './MappingPage.module.css';

interface MappingState {
  isMapping: boolean;
  isSaving: boolean;
}

export function MappingPageContent() {
  const { isEnabled, speed, setSpeed, enable, disable, stop } = useCmdVel();
  const { entries, info, warn, error, success, getEntries } = useStatusLog();

  const [mappingState, setMappingState] = useState<MappingState>({
    isMapping: false,
    isSaving: false,
  });

  // Handle key press for manual control info
  const handleKeyDown = useCallback((e: KeyboardEvent) => {
    const key = e.key.toLowerCase();
    if (['w', 'a', 's', 'd', ' '].includes(key)) {
      // Log is handled in useCmdVel
    }
  }, []);

  useEffect(() => {
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [handleKeyDown]);

  // Start mapping (stub)
  const handleStartMapping = useCallback(() => {
    setMappingState((prev) => ({ ...prev, isMapping: true }));
    info('Создание карты начато');
    // TODO: Call SLAM start service when available
  }, [info]);

  // Stop mapping (stub)
  const handleStopMapping = useCallback(() => {
    setMappingState((prev) => ({ ...prev, isMapping: false }));
    warn('Создание карты остановлено');
    // TODO: Call SLAM stop service when available
  }, [warn]);

  // Save map (stub)
  const handleSaveMap = useCallback(async () => {
    setMappingState((prev) => ({ ...prev, isSaving: true }));
    info('Сохранение карты...');

    // Simulate save delay
    await new Promise((resolve) => setTimeout(resolve, 1000));

    setMappingState((prev) => ({ ...prev, isSaving: false }));
    success('Карта успешно сохранена');
    // TODO: Call SLAM save service when available
  }, [info, success]);

  // Reset map (stub)
  const handleResetMap = useCallback(() => {
    if (confirm('Сбросить карту?')) {
      setMappingState((prev) => ({ ...prev, isMapping: false }));
      warn('Карта сброшена');
      // TODO: Call SLAM reset service when available
    }
  }, [warn]);

  // Toggle keyboard control
  const handleToggleControl = useCallback(() => {
    if (isEnabled) {
      disable();
      info('Управление с клавиатуры отключено');
    } else {
      enable();
      info('Управление с клавиатуры включено - Используйте WASD для движения, ПРОБЕЛ для остановки');
    }
  }, [isEnabled, enable, disable, info]);

  // Emergency stop
  const handleEmergencyStop = useCallback(() => {
    stop();
    disable();
    error('АВАРИЙНАЯ ОСТАНОВКА активирована');
  }, [stop, disable, error]);

  return (
    <div className={styles.content}>
      {/* Map Display */}
      <section className={styles.mapSection}>
        <MapPlaceholder
          width="100%"
          height="400px"
          showRobot
          showGrid
        />
      </section>

      {/* Mapping Controls */}
      <section className={styles.section}>
        <h2 className={styles.sectionTitle}>Создание карты</h2>
        <div className={styles.buttonGroup}>
          <Button
            variant={mappingState.isMapping ? 'danger' : 'primary'}
            onClick={mappingState.isMapping ? handleStopMapping : handleStartMapping}
          >
            {mappingState.isMapping ? 'Остановить' : 'Начать'}
          </Button>
          <Button
            variant="secondary"
            onClick={handleSaveMap}
            disabled={mappingState.isSaving || !mappingState.isMapping}
          >
            {mappingState.isSaving ? 'Сохранение...' : 'Сохранить'}
          </Button>
          <Button
            variant="danger"
            onClick={handleResetMap}
          >
            Сбросить
          </Button>
        </div>
        <div className={styles.status}>
          <span className={styles.statusLabel}>Статус:</span>
          <span className={mappingState.isMapping ? styles.active : styles.inactive}>
            {mappingState.isMapping ? 'Активно' : 'Неактивно'}
          </span>
        </div>
      </section>

      {/* Manual Control */}
      <section className={styles.section}>
        <h2 className={styles.sectionTitle}>Ручное управление</h2>

        <div className={styles.instructions}>
          <p>Используйте WASD для движения, ПРОБЕЛ для экстренной остановки</p>
        </div>

        <div className={styles.controlPanel}>
          <Button
            variant={isEnabled ? 'danger' : 'primary'}
            onClick={handleToggleControl}
            fullWidth
          >
            {isEnabled ? 'Остановить' : 'Включить управление'}
          </Button>

          <div className={styles.speedControl}>
            <label className={styles.speedLabel}>
              Скорость: {speed.linear.toFixed(1)} м/с
            </label>
            <input
              type="range"
              min="0.1"
              max="0.5"
              step="0.05"
              value={speed.linear}
              onChange={(e) => setSpeed({ ...speed, linear: parseFloat(e.target.value) })}
              className={styles.speedSlider}
              disabled={!isEnabled}
            />
          </div>
        </div>
      </section>

      {/* Log */}
      <section className={styles.section}>
        <StatusLog entries={getEntries()} />
      </section>
    </div>
  );
}
