import React, { useEffect, useState } from 'react';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogContentText from '@mui/material/DialogContentText';
import DialogTitle from '@mui/material/DialogTitle';
import Button from '@mui/material/Button';
import Snackbar from '@mui/material/Snackbar';
import Alert from '@mui/material/Alert';
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

  const [importDialogOpen, setImportDialogOpen] = useState(false);
  const [importCandidate, setImportCandidate] = useState(null);
  const [importFileName, setImportFileName] = useState('');
  const [importPosesDialogOpen, setImportPosesDialogOpen] = useState(false);
  const [importPosesCandidate, setImportPosesCandidate] = useState(null);
  const [importPosesFileName, setImportPosesFileName] = useState('');
  const [snackbarOpen, setSnackbarOpen] = useState(false);
  const [snackbarMsg, setSnackbarMsg] = useState('');
  const [snackbarSeverity, setSnackbarSeverity] = useState('info');

  const showSnackbar = (msg, severity = 'info') => {
    setSnackbarMsg(msg);
    setSnackbarSeverity(severity);
    setSnackbarOpen(true);
  };

  const handleImportConfirm = () => {
    if (!importCandidate) {
      setImportDialogOpen(false);
      return;
    }
    try {
      localStorage.setItem('program', JSON.stringify(importCandidate));
      try { window.dispatchEvent(new CustomEvent('programImported', { detail: importCandidate })); } catch (e) {}
      // notify user
      showSnackbar(`Program "${importFileName}" imported successfully.`, 'success');
    } catch (err) {
      console.error('Failed to write imported program', err);
      showSnackbar('Failed to save imported program', 'error');
    }
    setImportCandidate(null);
    setImportFileName('');
    setImportDialogOpen(false);
  };

  const handleImportCancel = () => {
  setImportCandidate(null);
  setImportFileName('');
  setImportDialogOpen(false);
  showSnackbar('Import cancelled', 'info');
  };

  const handleImportPosesConfirm = () => {
    if (!importPosesCandidate) {
      setImportPosesDialogOpen(false);
      return;
    }
    try {
      localStorage.setItem('savedPoses', JSON.stringify(importPosesCandidate));
      try { window.dispatchEvent(new CustomEvent('posesImported', { detail: importPosesCandidate })); } catch (e) {}
      showSnackbar(`Poses "${importPosesFileName}" imported successfully.`, 'success');
    } catch (err) {
      console.error('Failed to write imported poses', err);
      showSnackbar('Failed to save imported poses', 'error');
    }
    setImportPosesCandidate(null);
    setImportPosesFileName('');
    setImportPosesDialogOpen(false);
  };

  const handleImportPosesCancel = () => {
    setImportPosesCandidate(null);
    setImportPosesFileName('');
    setImportPosesDialogOpen(false);
    showSnackbar('Import cancelled', 'info');
  };

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
      <div style={{ marginTop: '0.5rem', display: 'flex', gap: '0.5rem' }}>
        <button
          onClick={() => {
            try {
              const program = localStorage.getItem('program') || '[]';
              const blob = new Blob([program], { type: 'application/json' });
              const url = URL.createObjectURL(blob);
              const a = document.createElement('a');
              a.href = url;
              a.download = `thor_program_${new Date().toISOString().replace(/[:.]/g, '-')}.json`;
              document.body.appendChild(a);
              a.click();
              a.remove();
              URL.revokeObjectURL(url);
            } catch (e) { console.error('Export failed', e); }
          }}
          style={{ padding: '0.6rem 1.2rem', background: '#1976d2', color: 'white', border: 'none', borderRadius: 6, cursor: 'pointer', fontSize: '0.95rem' }}
        >
          Export Program
        </button>
        <div>
          <input
            id="import-program-input"
            type="file"
            accept="application/json"
            style={{ display: 'none' }}
            onChange={(e) => {
              const f = e.target.files && e.target.files[0];
              if (!f) return;
              const reader = new FileReader();
              reader.onload = (evt) => {
                try {
                  const text = evt.target.result;
                  const parsed = JSON.parse(text);
                  if (!Array.isArray(parsed)) {
                    showSnackbar('Invalid program file: expected a JSON array', 'error');
                    return;
                  }
                  // Validar estructura de cada movimiento
                  const isValidMovement = (m) => {
                    return m && typeof m.type === 'string' && typeof m.pose === 'string';
                  };
                  if (!parsed.every(isValidMovement)) {
                    showSnackbar('Invalid program file: each item must have "type" (string) and "pose" (string)', 'error');
                    return;
                  }
                  // check existing
                  let hasExisting = false;
                  try {
                    const existingRaw = localStorage.getItem('program');
                    const existing = existingRaw ? JSON.parse(existingRaw) : [];
                    hasExisting = Array.isArray(existing) && existing.length > 0;
                  } catch (e) { hasExisting = false; }

                  if (hasExisting) {
                    // store candidate and open dialog
                    setImportCandidate(parsed);
                    setImportFileName(f.name || 'program.json');
                    setImportDialogOpen(true);
                  } else {
                    // no existing program, import directly
                    localStorage.setItem('program', JSON.stringify(parsed));
                    try { window.dispatchEvent(new CustomEvent('programImported', { detail: parsed })); } catch (e) {}
                    alert('Program imported successfully');
                  }
                } catch (err) {
                  console.error('Import failed', err);
                  alert('Failed to import program: invalid JSON');
                }
              };
              reader.readAsText(f);
              // clear input so same file can be reselected later
              e.target.value = '';
            }}
          />
          <button
            onClick={() => { const inp = document.getElementById('import-program-input'); if (inp) inp.click(); }}
            style={{ padding: '0.6rem 1.2rem', background: '#f57c00', color: 'white', border: 'none', borderRadius: 6, cursor: 'pointer', fontSize: '0.95rem' }}
          >
            Import Program
          </button>

          <Dialog
            open={importDialogOpen}
            onClose={handleImportCancel}
            aria-labelledby="import-dialog-title"
            aria-describedby="import-dialog-description"
          >
            <DialogTitle id="import-dialog-title">Import program</DialogTitle>
            <DialogContent>
              <DialogContentText id="import-dialog-description">
                You are about to overwrite the existing program with the contents of "{importFileName}". This cannot be undone.
                Do you want to continue?
              </DialogContentText>
            </DialogContent>
            <DialogActions>
              <Button onClick={handleImportCancel}>Cancel</Button>
              <Button onClick={handleImportConfirm} color="primary" autoFocus>Import</Button>
            </DialogActions>
          </Dialog>
        </div>
        <button
          onClick={() => {
            try {
              const poses = localStorage.getItem('savedPoses') || '[]';
              const blob = new Blob([poses], { type: 'application/json' });
              const url = URL.createObjectURL(blob);
              const a = document.createElement('a');
              a.href = url;
              a.download = `thor_poses_${new Date().toISOString().replace(/[:.]/g, '-')}.json`;
              document.body.appendChild(a);
              a.click();
              a.remove();
              URL.revokeObjectURL(url);
            } catch (e) { console.error('Export poses failed', e); }
          }}
          style={{ padding: '0.6rem 1.2rem', background: '#388e3c', color: 'white', border: 'none', borderRadius: 6, cursor: 'pointer', fontSize: '0.95rem' }}
        >
          Export Poses
        </button>
          <div style={{ marginLeft: '0.5rem' }}>
            <input
              id="import-poses-input"
              type="file"
              accept="application/json"
              style={{ display: 'none' }}
              onChange={(e) => {
                const f = e.target.files && e.target.files[0];
                if (!f) return;
                const reader = new FileReader();
                reader.onload = (evt) => {
                  try {
                    const text = evt.target.result;
                    const parsed = JSON.parse(text);
                    if (!Array.isArray(parsed)) {
                      return;
                    }
                    // Validar estructura de cada pose
                    const isValidPose = (pose) => {
                      return pose && typeof pose.name === 'string' && pose.name.length > 0 && typeof pose.joints === 'object' && pose.joints !== null;
                    };
                    if (!parsed.every(isValidPose)) {
                      showSnackbar('Invalid poses file: each item must have "name" (string) and "joints" (object)', 'error');
                      return;
                    }
                    // check existing poses
                    let hasExisting = false;
                    try {
                      const existingRaw = localStorage.getItem('savedPoses');
                      const existing = existingRaw ? JSON.parse(existingRaw) : [];
                      hasExisting = Array.isArray(existing) && existing.length > 0;
                    } catch (err) { hasExisting = false; }

                    if (hasExisting) {
                      setImportPosesCandidate(parsed);
                      setImportPosesFileName(f.name || 'poses.json');
                      setImportPosesDialogOpen(true);
                    } else {
                      localStorage.setItem('savedPoses', JSON.stringify(parsed));
                      try { window.dispatchEvent(new CustomEvent('posesImported', { detail: parsed })); } catch (e) {}
                      showSnackbar('Poses imported successfully', 'success');
                    }
                  } catch (err) {
                    console.error('Import poses failed', err);
                    showSnackbar('Failed to import poses: invalid JSON', 'error');
                  }
                };
                reader.readAsText(f);
                e.target.value = '';
              }}
            />
            <button
              onClick={() => { const inp = document.getElementById('import-poses-input'); if (inp) inp.click(); }}
              style={{ padding: '0.6rem 1.2rem', background: '#8bc34a', color: 'white', border: 'none', borderRadius: 6, cursor: 'pointer', fontSize: '0.95rem' }}
            >
              Import Poses
            </button>

            <Dialog
              open={importPosesDialogOpen}
              onClose={handleImportPosesCancel}
              aria-labelledby="import-poses-dialog-title"
              aria-describedby="import-poses-dialog-description"
            >
              <DialogTitle id="import-poses-dialog-title">Import poses</DialogTitle>
              <DialogContent>
                <DialogContentText id="import-poses-dialog-description">
                  You are about to overwrite the existing saved poses with the contents of "{importPosesFileName}". This cannot be undone.
                  Do you want to continue?
                </DialogContentText>
              </DialogContent>
              <DialogActions>
                <Button onClick={handleImportPosesCancel}>Cancel</Button>
                <Button onClick={handleImportPosesConfirm} color="primary" autoFocus>Import</Button>
              </DialogActions>
            </Dialog>
          </div>
      </div>
      <Snackbar
        open={snackbarOpen}
        autoHideDuration={4000}
        onClose={() => setSnackbarOpen(false)}
        anchorOrigin={{ vertical: 'bottom', horizontal: 'center' }}
      >
        <Alert onClose={() => setSnackbarOpen(false)} severity={snackbarSeverity} sx={{ width: '100%' }}>
          {snackbarMsg}
        </Alert>
      </Snackbar>
    </div>
  );
}
