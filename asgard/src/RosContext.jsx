// src/RosContext.jsx
import React, { createContext, useContext, useEffect, useState } from 'react';
import ROSLIB from 'roslib';

const RosContext = createContext();

export const RosProvider = ({ children }) => {
  const [ros, setRos] = useState(null);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    rosInstance.on('connection', () => {
      console.log('[ROS] ✅ Conectado a rosbridge');
      setConnected(true);
    });

    rosInstance.on('error', (error) => {
      console.error('[ROS] ❌ Error de conexión:', error);
    });

    rosInstance.on('close', () => {
      console.warn('[ROS] 🔌 Conexión cerrada');
      setConnected(false);
    });

    setRos(rosInstance);

    return () => {
      rosInstance.close();
    };
  }, []);

  return (
    <RosContext.Provider value={{ ros, connected }}>
      {children}
    </RosContext.Provider>
  );
};

export const useROS = () => useContext(RosContext);
