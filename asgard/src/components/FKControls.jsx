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

function FKControls({ initialJoints, urdfApi, active = true }) {
  const { connected } = useROS();
  const suppressSyncRef = useRef(false);
  const [jointValuesDeg, setJointValuesDeg] = useState(
    jointConfigs.reduce((acc, joint) => {
      acc[joint.name] = 0;
      return acc;
    }, {})
  );

  // Initialize joint values from initialJoints prop (in radians), converting to degrees
  useEffect(() => {
    if (initialJoints) {
      setJointValuesDeg(prev => {
        const newValuesDeg = { ...prev };
        jointConfigs.forEach(cfg => {
          if (initialJoints[cfg.name] !== undefined) {
            newValuesDeg[cfg.name] = Math.round((initialJoints[cfg.name] * 180) / Math.PI * 100) / 100;
          }
          if (cfg.name === 'gripperbase_to_armgearright') {
            newValuesDeg[cfg.name] = -newValuesDeg[cfg.name];
          }
        });
        return newValuesDeg;
      });
    suppressSyncRef.current = true;
    }
  }, [initialJoints]);

  // Update the ghost via the API only on user-initiated changes
  useEffect(() => {
    if (suppressSyncRef.current) {
      suppressSyncRef.current = false;
      return;
    }
     const jointsRad = {};
    jointConfigs.forEach(cfg => { jointsRad[cfg.name] = (jointValuesDeg[cfg.name] * Math.PI) / 180; });
    jointsRad['gripperbase_to_armgearright'] = -jointsRad['gripperbase_to_armgearright'];
    if (active && urdfApi) {
      urdfApi.setGhostJoints(jointsRad);
      if (urdfApi.syncTargetToTCP) urdfApi.syncTargetToTCP();
    }
  }, [jointValuesDeg, urdfApi, active]);

  // Handle slider change
  const handleJointSliderChange = (name, value) => {
    setJointValuesDeg(prev => ({ ...prev, [name]: parseFloat(value.toFixed(1)) }));
  };

  // Ensure input is numeric and within bounds
  const handleJointInputChange = (name, value, min, max) => {
    let valueNum = parseFloat(value);
    if (isNaN(valueNum)) valueNum = 0;
    if (valueNum > max) valueNum = max;
    if (valueNum < min) valueNum = min;
    valueNum = parseFloat(valueNum.toFixed(1));
    setJointValuesDeg(prev => ({ ...prev, [name]: valueNum }));
  };

  // Publish the ghost to the robot controller
  const publishGhostToController = () => {
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
          <Slider id={joint.name} min={joint.min} max={joint.max} step={0.1} value={jointValuesDeg[joint.name]} onChange={(e, value) => handleJointSliderChange(joint.name, value)} style={{ flex: 1, marginLeft: '-30px' }} />
          <Input value={jointValuesDeg[joint.name]} onChange={e => handleJointInputChange(joint.name, e.target.value, joint.min, joint.max)} inputProps={{ step: 0.1, min: joint.min, max: joint.max, type: 'number', 'aria-labelledby': joint.name }} style={{ width: '80px' }} />
          <span>°</span>
        </div>
      ))}
      <Button variant="contained" color="primary" onClick={publishGhostToController} style={{ marginTop: '1rem', display: 'block', marginLeft: 'auto', marginRight: 'auto', fontWeight: 'bold' }}>
        Move
      </Button>
    </div>
  );
}

export default FKControls;