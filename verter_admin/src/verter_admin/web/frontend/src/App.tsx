/**
 * App.tsx - Main application component with routing
 */

import { BrowserRouter, Routes, Route, Navigate } from 'react-router-dom';
import { useEffect, useCallback } from 'react';
import { useUIStore } from './store/uiStore';
import { useROS } from './hooks/useROS';
import { ErrorBoundary } from './components/common/ErrorBoundary';

// Setup pages
import { MappingPage } from './pages/setup/MappingPage';
import { HomePage } from './pages/setup/HomePage';
import { PointsPage } from './pages/setup/PointsPage';

// Active pages
import { IdlePage } from './pages/active/IdlePage';
import { MenuPage } from './pages/active/MenuPage';
import { SelectPage } from './pages/active/SelectPage';
import { ConfirmPage } from './pages/active/ConfirmPage';
import { NavigationPage } from './pages/active/NavigationPage';
import { ArrivedPage } from './pages/active/ArrivedPage';
import { FindPage } from './pages/active/FindPage';
import { InfoPage } from './pages/active/InfoPage';
import { AskPage } from './pages/active/AskPage';

const SetupPages = {
  Mapping: MappingPage,
  Home: HomePage,
  Points: PointsPage,
};

// Active pages
const ActivePages = {
  Idle: IdlePage,
  Menu: MenuPage,
  Select: SelectPage,
  Confirm: ConfirmPage,
  Navigation: NavigationPage,
  Arrived: ArrivedPage,
  Find: FindPage,
  Info: InfoPage,
  Ask: AskPage,
};

function App() {
  const mode = useUIStore((state) => state.mode);
  const isIdle = useUIStore((state) => state.isIdle);
  const recordActivity = useUIStore((state) => state.recordActivity);

  // Initialize ROS connection
  useROS(true);

  // Record activity on user interaction
  const handleActivity = useCallback(() => {
    recordActivity();
  }, [recordActivity]);

  useEffect(() => {
    const events = ['mousedown', 'keydown', 'scroll', 'touchstart'];
    events.forEach((event) => {
      window.addEventListener(event, handleActivity, { passive: true });
    });

    return () => {
      events.forEach((event) => {
        window.removeEventListener(event, handleActivity);
      });
    };
  }, [handleActivity]);

  return (
    <ErrorBoundary>
      <BrowserRouter>
        <Routes>
        {/* Setup mode routes */}
        <Route path="/setup" element={<Navigate to="/setup/mapping" replace />} />
        <Route path="/setup/mapping" element={<SetupPages.Mapping />} />
        <Route path="/setup/home" element={<SetupPages.Home />} />
        <Route path="/setup/points" element={<SetupPages.Points />} />

        {/* Active mode routes */}
        <Route path="/active" element={<Navigate to="/active/idle" replace />} />
        <Route path="/active/idle" element={<ActivePages.Idle />} />
        <Route path="/active/menu" element={<ActivePages.Menu />} />
        <Route path="/active/select" element={<ActivePages.Select />} />
        <Route path="/active/confirm/:name" element={<ActivePages.Confirm />} />
        <Route path="/active/navigation" element={<ActivePages.Navigation />} />
        <Route path="/active/arrived" element={<ActivePages.Arrived />} />
        <Route path="/active/find" element={<ActivePages.Find />} />
        <Route path="/active/info" element={<ActivePages.Info />} />
        <Route path="/active/ask" element={<ActivePages.Ask />} />

        {/* Default route - redirect based on mode or idle state */}
        <Route
          path="/"
          element={
            <Navigate
              to={isIdle ? '/active/idle' : mode === 'setup' ? '/setup' : '/active'}
              replace
            />
          }
        />

        {/* Fallback */}
        <Route path="*" element={<Navigate to="/" replace />} />
      </Routes>
    </BrowserRouter>
    </ErrorBoundary>
  );
}

export default App;
