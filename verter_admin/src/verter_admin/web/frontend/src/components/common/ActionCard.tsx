/**
 * ActionCard - Large touch-friendly card for main menu actions
 * Designed for elderly users with clear visual hierarchy
 */

import { ButtonHTMLAttributes, forwardRef } from 'react';
import { MaterialIcon } from './MaterialIcon';
import styles from './ActionCard.module.css';

export interface ActionCardProps extends ButtonHTMLAttributes<HTMLButtonElement> {
  icon: string;
  title: string;
  description?: string;
  variant?: 'primary' | 'secondary';
  children?: never;
}

export const ActionCard = forwardRef<HTMLButtonElement, ActionCardProps>(
  (
    {
      icon,
      title,
      description,
      variant = 'secondary',
      className,
      ...props
    },
    ref
  ) => {
    return (
      <button
        ref={ref}
        className={`${styles.actionCard} ${styles[variant]} ${className || ''}`}
        {...props}
      >
        <span className={styles.icon} aria-hidden="true">
          <MaterialIcon icon={icon} size={48} />
        </span>
        <span className={styles.content}>
          <span className={styles.title}>{title}</span>
          {description && (
            <span className={styles.description}>{description}</span>
          )}
        </span>
      </button>
    );
  }
);

ActionCard.displayName = 'ActionCard';
