// src/main.jsx
import React from 'react';
import ReactDOM from 'react-dom/client';
import App from './App';
import { BrowserRouter, Routes, Route } from 'react-router-dom';
import GizmoPage from './components/GizmoPage';
import UrdfViewer from './components/UrdfViewer';
import Visor3D from './components/visor3d';

ReactDOM.createRoot(document.getElementById('root')).render(
  <React.StrictMode>
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<App />} />
        <Route path="/gizmo" element={<GizmoPage />} />
        <Route path="/visor" element={<Visor3D />} />
      </Routes>
    </BrowserRouter>
  </React.StrictMode>
);
