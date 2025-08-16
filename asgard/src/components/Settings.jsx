import React, { useEffect } from 'react';
import FormControlLabel from '@mui/material/FormControlLabel';
import Checkbox from '@mui/material/Checkbox';

export default function Settings({
  showRealRobot,
  setShowRealRobot,
  showGhostRobot,
  setShowGhostRobot,
  showFPS,
  setShowFPS,
  showTcpGizmo,
  setShowTcpGizmo,
  showRobotState,
  setShowRobotState,
}) {
  // If the parent 'Show Ghost Robot' is off, also turn off the dependent TCP gizmo flag
  useEffect(() => {
    if (!showGhostRobot && showTcpGizmo) {
      setShowTcpGizmo(false);
    }
  }, [showGhostRobot, showTcpGizmo, setShowTcpGizmo]);

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
          control={<Checkbox checked={showTcpGizmo} onChange={(e) => setShowTcpGizmo(e.target.checked)} disabled={!showGhostRobot} />}
          label="Show TCP Gizmo"
        />
      </div>
      <FormControlLabel
        control={<Checkbox checked={showFPS} onChange={(e) => setShowFPS(e.target.checked)} />}
        label="Show FPS"
      />
      <FormControlLabel
        control={<Checkbox checked={showRobotState} onChange={(e) => setShowRobotState(e.target.checked)} />}
        label="Show Robot State"
      />
    </div>
  );
}
