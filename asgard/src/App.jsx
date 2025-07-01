// src/App.jsx
import React from 'react';
import './App.css';
import { RosProvider } from './RosContext';
import JointStateViewer from './components/JointStateViewer';
import UrdfViewer from './components/UrdfViewer';
import JointSliders from './components/JointSliders';

function App() {
  return (
    <RosProvider>
      <div style={{
        display: 'flex',
        height: '100vh',
        backgroundColor: '#1e1e1e',
        color: '#ffffff',
        fontFamily: 'Arial, sans-serif'
      }}>
        <div style={{
          flex: 1,
          padding: '1rem',
          borderRight: '1px solid #444',
          overflowY: 'auto'
        }}>
          <h2>📊 Estado de las articulaciones</h2>
          <JointStateViewer />
          <JointSliders />
        </div>
        <div style={{
          flex: 2,
          padding: '1rem'
        }}>
          <h2>🤖 Visor 3D del robot</h2>
          <UrdfViewer />
        </div>
      </div>
    </RosProvider>
  );
}

export default App;
