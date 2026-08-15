/**
 * App — диалоговый kiosk. Без роутинга: один fullscreen-экран.
 */

import { useROS } from './hooks/useROS';
import { ErrorBoundary } from './components/common/ErrorBoundary';
import DialogPage from './pages/dialog/DialogPage';

export default function App() {
  useROS(true);

  return (
    <ErrorBoundary>
      <DialogPage />
    </ErrorBoundary>
  );
}
