/**
 * MessageZone — отображает одно текущее сообщение.
 * Появление: typewriter для ответов/вопросов/ошибок, fade — для welcome/farewell.
 * При смене сообщения (key={text}) анимация перезапускается.
 */

import { useMemo } from 'react';
import type { DialogMessage } from '../../types/dialog';
import styles from './MessageZone.module.css';

interface MessageZoneProps {
  message: DialogMessage | null;
}

export default function MessageZone({ message }: MessageZoneProps) {
  // Длительность typewriter масштабируется под длину текста (~40 мс/символ).
  const twDuration = useMemo(() => {
    if (!message) return '2s';
    const ms = Math.min(Math.max(message.text.length * 45, 800), 6000);
    return `${ms}ms`;
  }, [message]);

  if (!message) {
    return <div className={styles.zone} />;
  }

  const useTypewriter =
    message.role === 'answer' ||
    message.role === 'question' ||
    message.role === 'error';

  return (
    <div className={styles.zone}>
      <p
        key={message.text}
        className={styles.message}
        data-role={message.role}
        data-typewriter={useTypewriter ? 'true' : 'false'}
        style={useTypewriter ? ({ ['--tw-duration' as string]: twDuration } as React.CSSProperties) : undefined}
      >
        {message.text}
      </p>
    </div>
  );
}
