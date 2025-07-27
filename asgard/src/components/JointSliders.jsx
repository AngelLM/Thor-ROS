import React, { useState, useEffect } from 'react';
import { useROS } from '../RosContext';
import ROSLIB from 'roslib';
import Slider from '@mui/material/Slider';
import Button from '@mui/material/Button';
import Input from '@mui/material/Input';

const jointConfigs = [
  { name: 'joint_1', label: 'Art 1', min: -170, max: 170 },
  { name: 'joint_2', label: 'Art 2', min: -90, max: 90 },
  { name: 'joint_3', label: 'Art 3', min: -90, max: 90 },
  { name: 'joint_4', label: 'Art 4', min: -170, max: 170 },
  { name: 'joint_5', label: 'Art 5', min: -90, max: 90 },
  { name: 'joint_6', label: 'Art 6', min: -170, max: 170 },
  { name: 'gripperbase_to_armgearright', label: 'Tool', min: 0, max: 89.9 }
];

function JointSliders({ onPreviewJointsChange, initialJoints }) {
  const { ros, connected } = useROS();
  const [sliderValues, setSliderValues] = useState(
    jointConfigs.reduce((acc, joint) => {
      acc[joint.name] = 0;
      return acc;
    }, {})
  );

  // Inicializa los sliders SOLO cuando cambia initialJoints (sin lógica extra)
  useEffect(() => {
    if (initialJoints) {
      setSliderValues(prev => {
        const newVals = { ...prev };
        jointConfigs.forEach(j => {
          if (initialJoints[j.name] !== undefined) {
            newVals[j.name] = Math.round((initialJoints[j.name] * 180) / Math.PI * 100) / 100;
          }
          // Invertimos el valor del gripper
          if (j.name === 'gripperbase_to_armgearright') {
            newVals[j.name] = -newVals[j.name];
          }
        });
        return newVals;
      });
    }
  }, [initialJoints]);

  // Notifica cambios al padre para el ghost robot
  useEffect(() => {
    if (onPreviewJointsChange) {
      // Pasa los valores articulares en radianes
      const joints = {};
      jointConfigs.forEach(j => {
        joints[j.name] = (sliderValues[j.name] * Math.PI) / 180;
      });
      // Invertimos el valor del gripper
      joints['gripperbase_to_armgearright'] = -joints['gripperbase_to_armgearright'];
      // Llama a la función de callback
      onPreviewJointsChange(joints);
    }
  }, [sliderValues]);

  const handleSliderChange = (name, value) => {
    setSliderValues(prev => ({
      ...prev,
      [name]: parseFloat(value.toFixed(1)) // Limit to 1 decimal place
    }));
  };

  const handleInputChange = (name, value, min, max) => {
    let num = parseFloat(value);
    if (isNaN(num)) num = 0;
    if (num > max) num = max;
    if (num < min) num = min;
    num = parseFloat(num.toFixed(1)); // Limit to 1 decimal place
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
    // Invertimos el valor del gripper
    message.data[jointConfigs.findIndex(j => j.name === 'gripperbase_to_armgearright')] = -message.data[jointConfigs.findIndex(j => j.name === 'gripperbase_to_armgearright')];

    // print log for debugging
    console.log('Sending joint command:', message.data);
    topic.publish(message);
  };

  return (
    <div style={{ marginTop: '0' }}> {/* Removed top margin */}
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
          <label htmlFor={joint.name} style={{ minWidth: '90px', fontWeight: 'bold' }}> {/* Added fontWeight: bold */}
            {joint.label}
          </label>
          <Slider
            id={joint.name}
            min={joint.min}
            max={joint.max}
            step={0.1} // Adjust step to match 1 decimal place
            value={sliderValues[joint.name]}
            onChange={(e, value) => handleSliderChange(joint.name, value)}
            style={{ flex: 1, marginLeft: '-30px' }} // Move slider closer to the label
          />
          <Input
            value={sliderValues[joint.name]}
            onChange={e => handleInputChange(joint.name, e.target.value, joint.min, joint.max)}
            inputProps={{
              step: 0.1, // Adjust step to match 1 decimal place
              min: joint.min,
              max: joint.max,
              type: 'number',
              'aria-labelledby': joint.name
            }}
            style={{ width: '80px' }} // Increased width from 60px to 80px
          />
          <span>°</span> {/* Add degree symbol */}
        </div>
      ))}
      <Button
        variant="contained"
        color="primary"
        onClick={sendJointCommand}
        style={{ marginTop: '1rem', display: 'block', marginLeft: 'auto', marginRight: 'auto', fontWeight: 'bold' }} // Centered and bold text
      >
        Move
      </Button>
    </div>
  );
}

export default JointSliders;