// src/components/JointSliders.jsx
import React, { useState } from 'react';
import { useROS } from '../RosContext';
import ROSLIB from 'roslib';

const jointNames = [
  'joint_1',
  'joint_2',
  'joint_3',
  'joint_4',
  'joint_5',
  'joint_6'
];

function JointSliders() {
  const { ros, connected } = useROS();

  const [sliderValues, setSliderValues] = useState(
    jointNames.reduce((acc, name) => {
      acc[name] = 0;
      return acc;
    }, {})
  );

  const handleSliderChange = (name, value) => {
    setSliderValues(prev => ({
      ...prev,
      [name]: parseFloat(value)
    }));
  };

  const sendJointCommand = () => {
    if (!connected || !ros) {
      console.warn('ROS no está conectado.');
      return;
    }

    const topic = new ROSLIB.Topic({
      ros,
      name: '/joint_group_position_controller/command',
      messageType: 'std_msgs/Float64MultiArray',
    });

    const message = new ROSLIB.Message({
      data: jointNames.map(name => (sliderValues[name] * Math.PI) / 180), // convertir grados a radianes
    });

    topic.publish(message);
  };

  return (
    <div style={{ marginTop: '2rem' }}>
      <h3>🎛️ Control de articulaciones</h3>
      {jointNames.map(name => (
        <div key={name} style={{ marginBottom: '1rem' }}>
          <label htmlFor={name}>
            {name}: {sliderValues[name].toFixed(1)}°
          </label>
          <input
            type="range"
            id={name}
            min={-180}
            max={180}
            step={1}
            value={sliderValues[name]}
            onChange={e => handleSliderChange(name, e.target.value)}
            style={{ width: '100%' }}
          />
        </div>
      ))}
      <button
        onClick={sendJointCommand}
        style={{
          marginTop: '1rem',
          padding: '0.5rem 1rem',
          backgroundColor: '#007bff',
          color: '#fff',
          border: 'none',
          borderRadius: '5px',
          cursor: 'pointer'
        }}
      >
        Enviar posiciones
      </button>
    </div>
  );
}

export default JointSliders;
