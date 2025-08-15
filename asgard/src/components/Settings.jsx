import React, { useEffect } from 'react';
import FormControlLabel from '@mui/material/FormControlLabel';
import Checkbox from '@mui/material/Checkbox';

export default function Settings({ showRealRobot, setShowRealRobot, showGhostRobot, setShowGhostRobot, showFPS, setShowFPS, showGhostRobotCoordinates, setShowGhostRobotCoordinates, showOverlay, setShowOverlay }) {
  // If the parent 'Show Ghost Robot' is turned off, ensure the dependent gizmo flag is also off
  useEffect(() => {
    if (!showGhostRobot && showGhostRobotCoordinates) {
      setShowGhostRobotCoordinates(false);
    }
  }, [showGhostRobot, showGhostRobotCoordinates, setShowGhostRobotCoordinates]);

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
      <div style={{ marginLeft: '1rem' }}>
        <FormControlLabel
          control={<Checkbox checked={showGhostRobotCoordinates} onChange={(e) => setShowGhostRobotCoordinates(e.target.checked)} disabled={!showGhostRobot} />}
          label="Show TCP Gizmo"
        />
      </div>
      <FormControlLabel
        control={<Checkbox checked={showFPS} onChange={(e) => setShowFPS(e.target.checked)} />}
        label="Show FPS"
      />
      <FormControlLabel
        control={<Checkbox checked={showOverlay} onChange={(e) => setShowOverlay(e.target.checked)} />}
        label="Show Robot State"
      />
    </div>
  );
}
