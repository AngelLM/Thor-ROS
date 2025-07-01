import React, { useState } from 'react';
import { useROS } from '../RosContext';
import ROSLIB from 'roslib';

const jointConfigs = [
  { name: 'joint_1', label: 'Art 1', min: -170, max: 170 },
  { name: 'joint_2', label: 'Art 2', min: -90, max: 90 },
  { name: 'joint_3', label: 'Art 3', min: -90, max: 90 },
  { name: 'joint_4', label: 'Art 4', min: -170, max: 170 },
  { name: 'joint_5', label: 'Art 5', min: -90, max: 90 },
  { name: 'joint_6', label: 'Art 6', min: -170, max: 170 }
];

function JointSliders() {
  const { ros, connected } = useROS();

  const [sliderValues, setSliderValues] = useState(
    jointConfigs.reduce((acc, joint) => {
      acc[joint.name] = 0;
      return acc;
    }, {})
  );

  const handleSliderChange = (name, value) => {
    setSliderValues(prev => ({
      ...prev,
      [name]: parseFloat(value)
    }));
  };

  const handleInputChange = (name, value, min, max) => {
    let num = parseFloat(value);
    if (isNaN(num)) num = 0;
    if (num > max) num = max;
    if (num < min) num = min;
    num = Math.round(num * 100) / 100;
    setSliderValues(prev => ({
      ...prev,
      [name]: num
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
      data: jointConfigs.map(joint => (sliderValues[joint.name] * Math.PI) / 180),
    });

    topic.publish(message);
  };

  return (
    <div style={{ marginTop: '2rem' }}>
      {jointConfigs.map(joint => (
        <div
          key={joint.name}
          style={{
            marginBottom: '0.7rem',
            display: 'flex',
            alignItems: 'center',
            gap: '0.5rem'
          }}
        >
          <label htmlFor={joint.name} style={{ minWidth: '90px' }}>
            {joint.label}:
          </label>
          <input
            type="range"
            id={joint.name}
            min={joint.min}
            max={joint.max}
            step={0.01}
            value={sliderValues[joint.name]}
            onChange={e => handleSliderChange(joint.name, e.target.value)}
            style={{ flex: 1 }}
          />
          <input
            type="number"
            min={joint.min}
            max={joint.max}
            step={0.01}
            value={sliderValues[joint.name]}
            onChange={e => handleInputChange(joint.name, e.target.value, joint.min, joint.max)}
            style={{ width: '60px' }}
          />
          <span>°</span>
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
        Move
      </button>
    </div>
  );
}

export default JointSliders;