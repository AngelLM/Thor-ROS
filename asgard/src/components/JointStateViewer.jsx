// src/JointStateViewer.jsx
import React, { useEffect, useState } from 'react';
import { useROS } from '../RosContext';
import ROSLIB from 'roslib';

export default function JointStateViewer() {
  const { ros, connected } = useROS();
  const [joints, setJoints] = useState([]);

  useEffect(() => {
    if (!ros || !connected) return;

    const listener = new ROSLIB.Topic({
      ros: ros,
      name: '/joint_states',
      messageType: 'sensor_msgs/msg/JointState'
    });

    const callback = (message) => {
      const result = message.name.map((name, idx) => ({
        name,
        position: message.position[idx]?.toFixed(3) ?? 'n/a'
      }));
      setJoints(result);
    };

    listener.subscribe(callback);

    return () => {
      listener.unsubscribe();
    };
  }, [ros, connected]);

  return (
    <div>
      <h2>🤖 Estado de las articulaciones</h2>
      {joints.length === 0 ? (
        <p>Esperando datos de /joint_states...</p>
      ) : (
        <ul>
          {joints.map((joint) => (
            <li key={joint.name}>
              {joint.name}: {joint.position} rad
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}
