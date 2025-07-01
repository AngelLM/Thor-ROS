import React, { useState } from 'react';
import './App.css';
import { RosProvider } from './RosContext';
import JointStateViewer from './components/JointStateViewer';
import UrdfViewer from './components/UrdfViewer';
import JointSliders from './components/JointSliders';

function App() {
  const [activeTab, setActiveTab] = useState('forward');

  return (
    <RosProvider>
      <div className="app-layout">
        <div className="sidebar">
          <div className="tabs">
            <button
              className={`tab-btn${activeTab === 'forward' ? ' active' : ''}`}
              onClick={() => setActiveTab('forward')}
            >
              Forward Kinematics
            </button>
            <button
              className={`tab-btn${activeTab === 'inverse' ? ' active' : ''}`}
              onClick={() => setActiveTab('inverse')}
            >
              Inverse Kinematics
            </button>
          </div>
          <div className="tab-content">
            {activeTab === 'forward' && (
              <>
                <h2>Forward Kinematics</h2>
                <JointStateViewer />
                <JointSliders />
              </>
            )}
            {activeTab === 'inverse' && (
              <>
                <h2>Inverse Kinematics</h2>
                {
                  
                }
              </>
            )}
          </div>
        </div>
        <div className="main-content">
          <h2>🤖 Visor 3D del robot</h2>
          <UrdfViewer />
        </div>
      </div>
    </RosProvider>
  );
}

export default App;