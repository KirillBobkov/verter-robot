/**
 * main.tsx - Application entry point
 */

import { StrictMode } from 'react';
import ReactDOM from 'react-dom/client';
import App from './App';
import './theme/index.css';
import './vite-env.d.ts';

// Load roslib.js from CDN
const script = document.createElement('script');
script.src = 'https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js';
script.async = false;
document.head.appendChild(script);

// Initialize React app
ReactDOM.createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <App />
  </StrictMode>
);
