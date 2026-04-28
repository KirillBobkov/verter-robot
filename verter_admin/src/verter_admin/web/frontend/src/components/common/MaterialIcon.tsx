/**
 * MaterialIcon - Material Symbols Outlined icon component
 * Using Google Fonts Material Symbols
 */

import { HTMLAttributes } from 'react';
import styles from './MaterialIcon.module.css';

export interface MaterialIconProps extends HTMLAttributes<HTMLSpanElement> {
  icon: string;
  size?: number;
}

export function MaterialIcon({ icon, size = 24, className = '', ...props }: MaterialIconProps) {
  return (
    <span
      className={`material-symbols-outlined ${styles.materialIcon} ${className}`}
      style={{ fontSize: size }}
      {...props}
    >
      {icon}
    </span>
  );
}
