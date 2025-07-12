import React from 'react';
import FormControlLabel from '@mui/material/FormControlLabel';
import Checkbox from '@mui/material/Checkbox';

export default function Settings({ showRealRobot, setShowRealRobot, showGhostRobot, setShowGhostRobot }) {
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
    </div>
  );
}
