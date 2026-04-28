/**
 * MenuPage - Main menu for tablet interface
 * Phase 3: Active Mode
 *
 * Shows large buttons for main actions
 */

import { useCallback } from 'react';
import { useNavigate } from 'react-router-dom';
import { useIdleTimer } from '../../hooks/useIdleTimer';
import { ActiveLayout } from '../../components/layout/ActiveLayout';
import { ActionCard } from '../../components/common/ActionCard';
import { MaterialIcon } from '../../components/common/MaterialIcon';
import { useUIStore } from '../../store/uiStore';
import { useWaypointStore } from '../../store/waypointStore';
import styles from './MenuPage.module.css';

interface MenuItem {
  id: string;
  icon: string; // Material Symbol name
  label: string;
  description?: string;
  route: string;
  variant: 'primary' | 'secondary';
}

const MENU_ITEMS: MenuItem[] = [
  {
    id: 'guide',
    icon: 'directions_walk',
    label: 'Провести до кабинета',
    description: 'Робот проведёт вас к нужному кабинету',
    route: '/active/select',
    variant: 'primary',
  },
  {
    id: 'find',
    icon: 'search',
    label: 'Найти кабинет',
    route: '/active/find',
    variant: 'secondary',
  },
  {
    id: 'ask',
    icon: 'support_agent',
    label: 'Задать вопрос',
    description: 'Задайте вопрос голосом',
    route: '/active/ask',
    variant: 'secondary',
  },
  {
    id: 'info',
    icon: 'info',
    label: 'Информация',
    description: 'Часы работы и контакты',
    route: '/active/info',
    variant: 'secondary',
  },
];

export function MenuPage() {
  const navigate = useNavigate();
  const recordActivity = useUIStore((state) => state.recordActivity);

  // Idle timer with 15 second timeout
  useIdleTimer({ enabled: true, timeout: 15, onIdle: () => navigate('/active/idle') });

  // Load waypoints for menu display
  const { waypoints, fetchWaypoints } = useWaypointStore();

  // Handle menu item click
  const handleMenuClick = useCallback((route: string) => {
    recordActivity();
    navigate(route);
  }, [navigate, recordActivity]);

  // Handle back button
  const handleBack = useCallback(() => {
    recordActivity();
    navigate('/active/idle');
  }, [navigate, recordActivity]);

  return (
    <ActiveLayout showBackButton onBack={handleBack}>
      <div className={styles.menuContainer}>
        {/* Title */}
        <h1 className={styles.title}>Меню</h1>

        {/* Menu Items */}
        <div className={styles.cardGrid}>
          {MENU_ITEMS.map((item) => (
            <ActionCard
              key={item.id}
              icon={item.icon}
              title={item.label}
              description={item.description}
              variant={item.variant}
              onClick={() => handleMenuClick(item.route)}
            />
          ))}
        </div>
      </div>
    </ActiveLayout>
  );
}
