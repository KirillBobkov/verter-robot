/**
 * InfoPage - Information about hospital/clinic
 * Phase 4: Active Mode
 */

import { useCallback } from 'react';
import { useNavigate } from 'react-router-dom';
import { useIdleTimer } from '../../hooks/useIdleTimer';
import { ActiveLayout } from '../../components/layout/ActiveLayout';
import { useUIStore } from '../../store/uiStore';
import styles from './InfoPage.module.css';

export function InfoPage() {
  const navigate = useNavigate();
  const recordActivity = useUIStore((state) => state.recordActivity);

  // Idle timer
  useIdleTimer({ enabled: true, timeout: 20, onIdle: () => navigate('/active/idle') });

  // Handle back
  const handleBack = useCallback(() => {
    recordActivity();
    navigate('/active/menu');
  }, [navigate, recordActivity]);

  const info = {
    title: 'Информация',
    workingHours: 'Режим работы',
    schedule: [
      { days: 'Пн-Пт', hours: '8:00 - 20:00' },
      { days: 'Суббота', hours: '9:00 - 15:00' },
      { days: 'Воскресенье', hours: 'Закрыто' },
    ],
    rules: 'Правила посещения',
    rulesList: [
      'Регистрация в приёмной обязательна',
      'Наличие направления от врача',
      'Соблюдайте очередь',
      'Верхняя одежда в гардероб',
    ],
  };

  return (
    <ActiveLayout showBackButton onBack={handleBack}>
      <div className={styles.infoContainer}>
        {/* Title */}
        <h1 className={styles.title}>{info.title}</h1>

        {/* Working Hours */}
        <section className={styles.section}>
          <h2 className={styles.sectionTitle}>{info.workingHours}</h2>
          <div className={styles.schedule}>
            {info.schedule.map((item, index) => (
              <div key={index} className={styles.scheduleRow}>
                <span className={styles.scheduleDays}>{item.days}</span>
                <span className={styles.scheduleHours}>{item.hours}</span>
              </div>
            ))}
          </div>
        </section>

        {/* Rules */}
        <section className={styles.section}>
          <h2 className={styles.sectionTitle}>{info.rules}</h2>
          <ul className={styles.rulesList}>
            {info.rulesList.map((rule, index) => (
              <li key={index} className={styles.ruleItem}>{rule}</li>
            ))}
          </ul>
        </section>

        {/* Help Section */}
        <section className={styles.section}>
          <h2 className={styles.sectionTitle}>Нужна помощь?</h2>
          <button
            className={styles.helpButton}
            onClick={() => navigate('/active/menu')}
          >
            Вернуться в меню
          </button>
        </section>
      </div>
    </ActiveLayout>
  );
}
