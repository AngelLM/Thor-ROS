import React from 'react';
import FormControlLabel from '@mui/material/FormControlLabel';
import Checkbox from '@mui/material/Checkbox';

export default function Settings({ showRealRobot, setShowRealRobot, showGhostRobot, setShowGhostRobot, showFPS, setShowFPS, showGhostRobotCoordinates, setShowGhostRobotCoordinates }) {
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: '1rem' }}>
      <FormControlLabel
        control={<Checkbox checked={showRealRobot} onChange={(e) => setShowRealRobot(e.target.checked)} />}
        label="Show Real Robot"
      />
      <FormControlLabel
        control={<Checkbox checked={showGhostRobot} onChange={(e) => setShowGhostRobot(e.target.checked)} />}
        label="Show Ghost Robot"
      />
      <FormControlLabel
        control={<Checkbox checked={showGhostRobotCoordinates} onChange={(e) => setShowGhostRobotCoordinates(e.target.checked)} />}
        label="Show Ghost Robot TCP Gizmo"
      />
      <FormControlLabel
        control={<Checkbox checked={showFPS} onChange={(e) => setShowFPS(e.target.checked)} />}
        label="Show FPS"
      />
    </div>
  );
}
