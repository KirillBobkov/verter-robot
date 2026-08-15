/**
 * main.tsx — точка входа диалогового kiosk-фронтенда.
 * roslib подключается как npm-бандл (не CDN).
 */

import ReactDOM from 'react-dom/client';
import App from './App';
import './theme/index.css';
import './vite-env.d.ts';

ReactDOM.createRoot(document.getElementById('root')!).render(
  // <StrictMode>  // Временно отключен - вызывает реконнекты rosbridge в dev режиме
    <App />
  // </StrictMode>
);
