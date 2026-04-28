/**
 * ActiveLayout - Layout wrapper for active mode pages (tablet interface)
 * Material Design 3 without NavigationRail
 */

import { ReactNode } from 'react';
import { Outlet, useLocation } from 'react-router-dom';
import { useNavigate } from 'react-router-dom';
import { useUIStore } from '../../store/uiStore';
import { EmergencyStop } from '../common/EmergencyStop';
import { MaterialIcon } from '../common/MaterialIcon';
import styles from './ActiveLayout.module.css';

interface ActiveLayoutProps {
  children?: ReactNode;
  showBackButton?: boolean;
  onBack?: () => void;
}

export function ActiveLayout({ children, showBackButton, onBack }: ActiveLayoutProps) {
  const navigate = useNavigate();
  const location = useLocation();
  const setMode = useUIStore((state) => state.setMode);

  // Show home button on all pages except menu and idle
  const showHomeButton = location.pathname !== '/active/menu' &&
                        location.pathname !== '/active/idle';

  const handleHome = () => {
    navigate('/active/menu');
  };

  const goToSetup = () => {
    setMode('setup');
    navigate('/setup/mapping');
  };

  return (
    <div className={styles.layout}>
      {/* Main Content Area */}
      <div className={styles.contentArea}>
        {/* Header (optional, for pages with back button) */}
        {(showBackButton || onBack || showHomeButton) && (
          <header className={styles.header}>
            <div className={styles.headerContent}>
              {showBackButton && onBack && (
                <button className={styles.backButton} onClick={onBack}>
                  ← Назад
                </button>
              )}
              {showHomeButton && !showBackButton && (
                <button className={styles.homeButton} onClick={handleHome}>
                  <MaterialIcon icon="home" size={24} className={styles.homeIcon} />
                  <span>На главную</span>
                </button>
              )}
              <button className={styles.setupButton} onClick={goToSetup}>
                <MaterialIcon icon="settings" size={20} />
                <span>Настройка</span>
              </button>
            </div>
          </header>
        )}

        {/* Main Content */}
        <main className={styles.main}>
          {children || <Outlet />}
        </main>
      </div>

      {/* Emergency Stop Button - fixed position FAB */}
      <EmergencyStop />
    </div>
  );
}
