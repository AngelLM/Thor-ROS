import React, { useState, useEffect, useRef } from 'react';
import { useROS } from '../RosContext';
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

function JointSliders({ onPreviewJointsChange, initialJoints, urdfApi, active = true }) {
  const { connected } = useROS();
  const suppressNextApplyRef = useRef(false);
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
          if (j.name === 'gripperbase_to_armgearright') {
            newVals[j.name] = -newVals[j.name];
          }
        });
        return newVals;
      });
      suppressNextApplyRef.current = true; // don't push back to ghost for this programmatic sync
    }
  }, [initialJoints]);

  // Actualiza ghost vía API solo en cambios del usuario
  useEffect(() => {
    if (suppressNextApplyRef.current) {
      suppressNextApplyRef.current = false;
      return;
    }
    const joints = {};
    jointConfigs.forEach(j => { joints[j.name] = (sliderValues[j.name] * Math.PI) / 180; });
    joints['gripperbase_to_armgearright'] = -joints['gripperbase_to_armgearright'];
    if (active && urdfApi) {
      urdfApi.setGhostJoints(joints);
      if (urdfApi.syncTargetToTCP) urdfApi.syncTargetToTCP();
    }
    // if (onPreviewJointsChange) onPreviewJointsChange(joints);
  }, [sliderValues, urdfApi, active]);

  const handleSliderChange = (name, value) => {
    setSliderValues(prev => ({ ...prev, [name]: parseFloat(value.toFixed(1)) }));
  };

  const handleInputChange = (name, value, min, max) => {
    let num = parseFloat(value);
    if (isNaN(num)) num = 0;
    if (num > max) num = max;
    if (num < min) num = min;
    num = parseFloat(num.toFixed(1));
    setSliderValues(prev => ({ ...prev, [name]: num }));
  };

  const sendJointCommand = () => {
    if (!connected || !urdfApi) return;
    urdfApi.publishGhostToController();
  };

  return (
    <div style={{ marginTop: '0' }}>
      {jointConfigs.map(joint => (
        <div key={joint.name} style={{ marginBottom: '0.7rem', display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
          <label htmlFor={joint.name} style={{ minWidth: '90px', fontWeight: 'bold' }}>
            {joint.label}
          </label>
          <Slider id={joint.name} min={joint.min} max={joint.max} step={0.1} value={sliderValues[joint.name]} onChange={(e, value) => handleSliderChange(joint.name, value)} style={{ flex: 1, marginLeft: '-30px' }} />
          <Input value={sliderValues[joint.name]} onChange={e => handleInputChange(joint.name, e.target.value, joint.min, joint.max)} inputProps={{ step: 0.1, min: joint.min, max: joint.max, type: 'number', 'aria-labelledby': joint.name }} style={{ width: '80px' }} />
          <span>°</span>
        </div>
      ))}
      <Button variant="contained" color="primary" onClick={sendJointCommand} style={{ marginTop: '1rem', display: 'block', marginLeft: 'auto', marginRight: 'auto', fontWeight: 'bold' }}>
        Move
      </Button>
    </div>
  );
}

export default JointSliders;