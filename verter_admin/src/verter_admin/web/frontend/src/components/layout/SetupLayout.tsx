/**
 * SetupLayout - Layout wrapper for setup mode pages
 * Material Design 3 with tabs (no NavigationRail)
 */

import { ReactNode } from 'react';
import { useNavigate, useLocation } from 'react-router-dom';
import { useROSStore } from '../../store/rosStore';
import { useUIStore } from '../../store/uiStore';
import styles from './SetupLayout.module.css';

interface SetupLayoutProps {
  children: ReactNode;
  title?: string;
}

export function SetupLayout({ children, title }: SetupLayoutProps) {
  const isConnected = useROSStore((state) => state.isConnected);
  const status = useROSStore((state) => state.status);
  const navigate = useNavigate();
  const location = useLocation();
  const setMode = useUIStore((state) => state.setMode);

  const getStatusText = (status: string): string => {
    switch (status) {
      case 'connected': return 'Подключено';
      case 'connecting': return 'Подключение...';
      default: return 'Отключено';
    }
  };

  const tabs = [
    { id: 'mapping', label: 'Карта', path: '/setup/mapping' },
    { id: 'home', label: 'Дом', path: '/setup/home' },
    { id: 'points', label: 'Точки', path: '/setup/points' },
  ];

  const handleBackToActive = () => {
    setMode('active');
    navigate('/active/menu');
  };

  return (
    <div className={styles.layout}>
      {/* Content area - no NavigationRail */}
      <div className={styles.contentArea}>
        {/* Header with Tabs */}
        <header className={styles.header}>
          <div className={styles.headerTop}>
            <button className={styles.backButton} onClick={handleBackToActive}>
              ← В меню
            </button>

            {/* ROS Status */}
            <div className={styles.rosStatus}>
              <span className={`${styles.statusDot} ${styles[status]}`}></span>
              <span>{getStatusText(status)}</span>
            </div>
          </div>

          {/* Tabs */}
          <div className={styles.tabs}>
            {tabs.map((tab) => {
              const isActive = location.pathname === tab.path;
              return (
                <button
                  key={tab.id}
                  className={`${styles.tab} ${isActive ? styles.tabActive : ''}`}
                  onClick={() => navigate(tab.path)}
                >
                  {tab.label}
                </button>
              );
            })}
          </div>
        </header>

        {/* Main Content */}
        <main className={styles.main}>{children}</main>

        {/* Footer */}
        <footer className={styles.footer}>
          <p>Verter Robot Web UI v1.0.0</p>
        </footer>
      </div>
    </div>
  );
}
