/**
 * NavigationRail - Material Design 3 navigation rail component
 * Vertical navigation bar for tablet landscape interface
 */

import { useCallback } from 'react';
import { useNavigate, useLocation } from 'react-router-dom';
import { useUIStore } from '../../store/uiStore';
import { MaterialIcon } from './MaterialIcon';
import styles from './NavigationRail.module.css';

export interface NavItem {
  id: string;
  icon: string;
  label: string;
  path: string;
}

// Main navigation items - only primary screens
const ACTIVE_NAV_ITEMS: NavItem[] = [
  { id: 'guide', icon: 'directions_walk', label: 'Провести', path: '/active/menu' },
  { id: 'find', icon: 'search', label: 'Найти', path: '/active/find' },
  { id: 'ask', icon: 'support_agent', label: 'Вопрос', path: '/active/ask' },
  { id: 'more', icon: 'more_horiz', label: 'Ещё', path: '/active/info' },
];

const SETUP_NAV_ITEMS: NavItem[] = [
  { id: 'mapping', icon: 'map', label: 'Карта', path: '/setup/mapping' },
  { id: 'home', icon: 'home', label: 'Главная', path: '/setup/home' },
  { id: 'points', icon: 'place', label: 'Точки', path: '/setup/points' },
];

interface NavigationRailProps {
  mode?: 'active' | 'setup';
}

export function NavigationRail({ mode = 'active' }: NavigationRailProps) {
  const navigate = useNavigate();
  const location = useLocation();
  const recordActivity = useUIStore((state) => state.recordActivity);

  const navItems = mode === 'active' ? ACTIVE_NAV_ITEMS : SETUP_NAV_ITEMS;

  // Check if a path is currently active
  const isActive = useCallback((itemPath: string, currentPath: string) => {
    // Exact match for most items
    if (itemPath === currentPath) return true;
    // For menu page, also show active on select page (sub-flow)
    if (mode === 'active' && itemPath === '/active/menu') {
      return currentPath.startsWith('/active/menu') ||
             currentPath.startsWith('/active/select') ||
             currentPath.startsWith('/active/confirm');
    }
    return false;
  }, [mode]);

  const handleNavClick = useCallback((path: string) => {
    recordActivity();
    navigate(path);
  }, [navigate, recordActivity]);

  const currentPath = location.pathname;

  return (
    <nav
      className={styles.navigationRail}
      role="navigation"
      aria-label={mode === 'active' ? 'Main navigation' : 'Setup navigation'}
    >
      <div className={styles.navigationRailContainer}>
        {navItems.map((item) => {
          const active = isActive(item.path, currentPath);
          return (
            <button
              key={item.id}
              className={`${styles.navigationRailItem} ${active ? styles.navigationRailItemActive : ''}`}
              onClick={() => handleNavClick(item.path)}
              aria-current={active ? 'page' : undefined}
              title={item.label}
            >
              <span className={styles.navigationRailIcon} aria-hidden="true">
                <MaterialIcon icon={item.icon} size={24} />
              </span>
              <span className={styles.navigationRailLabel}>{item.label}</span>
            </button>
          );
        })}
      </div>
    </nav>
  );
}
