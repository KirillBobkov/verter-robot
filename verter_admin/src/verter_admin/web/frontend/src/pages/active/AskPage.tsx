/**
 * AskPage - Voice question interface
 * Phase 4: Active Mode
 */

import { useCallback, useEffect, useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { useIdleTimer } from '../../hooks/useIdleTimer';
import { ActiveLayout } from '../../components/layout/ActiveLayout';
import { Button } from '../../components/common/Button';
import { useUIStore } from '../../store/uiStore';
import styles from './AskPage.module.css';

export function AskPage() {
  const navigate = useNavigate();
  const recordActivity = useUIStore((state) => state.recordActivity);

  const [isListening, setIsListening] = useState(false);
  const [lastQuestion, setLastQuestion] = useState('');

  // Idle timer - extended for voice interaction
  useIdleTimer({ enabled: true, timeout: 30, onIdle: () => navigate('/active/idle') });

  // Handle back
  const handleBack = useCallback(() => {
    recordActivity();
    navigate('/active/menu');
  }, [navigate, recordActivity]);

  // Handle start listening
  const handleStartListening = useCallback(() => {
    setIsListening(true);
    // TODO: Trigger ROS STT service
    // For now, simulate listening
    setTimeout(() => {
      setIsListening(false);
    }, 5000);
  }, []);

  // Handle repeat
  const handleRepeat = useCallback(() => {
    recordActivity();
    // TODO: Trigger ROS TTS to repeat last phrase
    console.log('Repeat:', lastQuestion);
  }, [lastQuestion, recordActivity]);

  // Handle finish
  const handleFinish = useCallback(() => {
    recordActivity();
    navigate('/active/menu');
  }, [navigate, recordActivity]);

  return (
    <ActiveLayout showBackButton onBack={handleBack}>
      <div className={styles.askContainer}>
        {/* Voice Icon */}
        <div className={`${styles.voiceIcon} ${isListening ? styles.listening : ''}`}>
          🎤
        </div>

        {/* Status */}
        <div className={styles.status}>
          {isListening ? 'Слушаю вас...' : ''}
        </div>

        {/* Listening Animation */}
        {isListening && (
          <div className={styles.waves}>
            <div className={styles.wave}></div>
            <div className={styles.wave}></div>
            <div className={styles.wave}></div>
          </div>
        )}



        {/* Actions */}
        <div className={styles.actions}>
                  {/* Listen Button */}
        {!isListening && (
          <button
            className={styles.listenButton}
            onClick={handleStartListening}
          >
            Нажмите и говорите
          </button>
        )}
          <Button
            variant="secondary"
            onClick={handleRepeat}
            disabled={!lastQuestion}
          >
            Повторить вопрос
          </Button>
          <Button
            variant="primary"
            onClick={handleFinish}
          >
            Завершить
          </Button>
        </div>
      </div>
    </ActiveLayout>
  );
}
