/**
 * App — диалоговый kiosk. Без роутинга: один fullscreen-экран.
 *
 * В режиме эмуляции (npm run dev:emu) поверх экрана монтируется EmulationPanel
 * (lazy-импорт → не попадает в прод-бандл обычной сборки).
 */

import { lazy, Suspense } from 'react';
import { useROS } from './hooks/useROS';
import { ErrorBoundary } from './components/common/ErrorBoundary';
import DialogPage from './pages/dialog/DialogPage';
import { isEmulation } from './config/emulation';

const EmulationPanel = isEmulation
  ? lazy(() => import('./components/emulation/EmulationPanel'))
  : null;

export default function App() {
  useROS(true);

  return (
    <ErrorBoundary>
      <DialogPage />
      {EmulationPanel && (
        <Suspense fallback={null}>
          <EmulationPanel />
        </Suspense>
      )}
    </ErrorBoundary>
  );
}
