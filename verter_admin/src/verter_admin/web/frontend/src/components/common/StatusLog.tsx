/**
 * StatusLog - Log display component
 */

import { useEffect, useRef } from 'react';
import type { LogEntry } from '../../types/domain';
import styles from './StatusLog.module.css';

export interface StatusLogProps {
  entries: LogEntry[];
  maxEntries?: number;
  className?: string;
}

export function StatusLog({ entries, maxEntries = 100, className }: StatusLogProps) {
  const scrollRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new entries are added
  useEffect(() => {
    if (scrollRef.current) {
      scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
    }
  }, [entries]);

  const displayEntries = entries.slice(-maxEntries);

  const getLevelClass = (level: LogEntry['level']) => {
    switch (level) {
      case 'error':
        return styles.error;
      case 'warn':
        return styles.warn;
      case 'success':
        return styles.success;
      default:
        return styles.info;
    }
  };

  return (
    <div className={`${styles.log} ${className || ''}`}>
      <div className={styles.header}>Log</div>
      <div ref={scrollRef} className={styles.entries}>
        {displayEntries.length === 0 ? (
          <div className={styles.empty}>No entries</div>
        ) : (
          displayEntries.map((entry, index) => (
            <div key={index} className={`${styles.entry} ${getLevelClass(entry.level)}`}>
              <span className={styles.timestamp}>[{entry.timestamp}]</span>
              <span className={styles.message}>{entry.message}</span>
            </div>
          ))
        )}
      </div>
    </div>
  );
}

/**
 * Hook for managing log entries
 */
export function useStatusLog() {
  const entriesRef = useRef<LogEntry[]>([]);

  const addEntry = (level: LogEntry['level'], message: string) => {
    const timestamp = new Date().toISOString().slice(11, 23); // HH:mm:ss.sss
    const entry: LogEntry = { timestamp, level, message };
    entriesRef.current.push(entry);
    return entry;
  };

  const info = (message: string) => addEntry('info', message);
  const warn = (message: string) => addEntry('warn', message);
  const error = (message: string) => addEntry('error', message);
  const success = (message: string) => addEntry('success', message);

  const clear = () => {
    entriesRef.current = [];
  };

  const getEntries = () => entriesRef.current;

  return {
    entries: entriesRef.current,
    info,
    warn,
    error,
    success,
    clear,
    getEntries,
  };
}
