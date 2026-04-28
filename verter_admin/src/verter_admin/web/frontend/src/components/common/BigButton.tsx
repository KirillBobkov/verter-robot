/**
 * BigButton - Large touch-friendly button for tablet interface
 */

import { ButtonHTMLAttributes, forwardRef } from 'react';
import styles from './BigButton.module.css';

export interface BigButtonProps extends ButtonHTMLAttributes<HTMLButtonElement> {
  variant?: 'primary' | 'secondary' | 'danger';
  icon?: string;
  label?: string;
  children?: React.ReactNode;
}

export const BigButton = forwardRef<HTMLButtonElement, BigButtonProps>(
  (
    {
      variant = 'primary',
      icon,
      label,
      children,
      className,
      ...props
    },
    ref
  ) => {
    return (
      <button
        ref={ref}
        className={`${styles.bigButton} ${styles[variant]} ${className || ''}`}
        {...props}
      >
        {icon && <span className={styles.icon}>{icon}</span>}
        <span className={styles.content}>
          {label || children}
        </span>
      </button>
    );
  }
);

BigButton.displayName = 'BigButton';
