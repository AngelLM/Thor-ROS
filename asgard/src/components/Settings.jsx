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
import Box from '@mui/material/Box';

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

  // Ensure TCP gizmo is off if ghost robot is off
  useEffect(() => {
    if (!showGhostRobot && showTcpGizmo) setShowTcpGizmo(false);
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

  // Show a snackbar message
  const showSnackbar = (msg, severity = 'info') => {
    setSnackbarMsg(msg);
    setSnackbarSeverity(severity);
    setSnackbarOpen(true);
  };

  // Export helper using File System Access API when available
  const exportToFile = async (suggestedName, content) => {
    if (typeof window.showSaveFilePicker === 'function') {
      try {
        const handle = await window.showSaveFilePicker({
          suggestedName,
          types: [{ description: 'JSON', accept: { 'application/json': ['.json'] } }],
        });
        const writable = await handle.createWritable();
        await writable.write(new Blob([content], { type: 'application/json' }));
        await writable.close();
        const name = (handle && handle.name) ? handle.name : suggestedName;
        showSnackbar(`Export done: ${name}`, 'success');
      } catch (err) {
        if (err && (err.name === 'AbortError' || (err.message || '').toLowerCase().includes('cancel'))) showSnackbar('Export cancelled', 'info');
        else { console.error('Export failed', err); showSnackbar('Export failed', 'error'); }
      }
      return;
    }

    // Fallback anchor download (cannot detect cancel)
    try {
      const blob = new Blob([content], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = suggestedName;
      document.body.appendChild(a);
      a.click();
      a.remove();
      URL.revokeObjectURL(url);
      showSnackbar(`Export started: ${suggestedName}`, 'info');
    } catch (e) {
      console.error('Fallback export failed', e);
      showSnackbar('Export failed', 'error');
    }
  };

  // Confirm import and overwrite existing program
  const handleImportConfirm = () => {
    if (!importCandidate) {
      setImportDialogOpen(false);
      return;
    }
    try {
      localStorage.setItem('program', JSON.stringify(importCandidate));
      try { window.dispatchEvent(new CustomEvent('programImported', { detail: importCandidate })); } catch (e) {}
      showSnackbar(`Program "${importFileName}" imported successfully.`, 'success');
    } catch (err) {
      console.error('Failed to write imported program', err);
      showSnackbar('Failed to save imported program', 'error');
    }
    setImportCandidate(null);
    setImportFileName('');
    setImportDialogOpen(false);
  };

  // Cancel import
  const handleImportCancel = () => {
    setImportCandidate(null);
    setImportFileName('');
    setImportDialogOpen(false);
    showSnackbar('Import cancelled', 'info');
  };

  // Confirm import and overwrite existing poses
  const handleImportPosesConfirm = () => {
    if (!importPosesCandidate) { setImportPosesDialogOpen(false); return; }
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

  /// Cancel import
  const handleImportPosesCancel = () => {
    setImportPosesCandidate(null);
    setImportPosesFileName('');
    setImportPosesDialogOpen(false);
    showSnackbar('Import cancelled', 'info');
  };

  // Validate pose
  const isValidPose = (pose) => {
    return pose && typeof pose.name === 'string' && pose.name.length > 0 && typeof pose.joints === 'object' && pose.joints !== null;
  };

  // Validate movement
  const isValidMovement = (m) => {
    return m && typeof m.type === 'string' && typeof m.pose === 'string';
  };

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: '1rem' }}>
      <Box component="fieldset" style={{ display: 'flex', flexDirection: 'column'}}>
        <legend>Visuals</legend>
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
      </Box>

      <Box component="fieldset" style={{ display: 'flex', flexDirection: 'column'}}>
        <legend>Debug</legend>
        <FormControlLabel
          control={<Checkbox checked={showFPS} onChange={(e) => setShowFPS(e.target.checked)} />}
          label="Show FPS"
        />
        <FormControlLabel
          control={<Checkbox checked={showRobotState} onChange={(e) => setShowRobotState(e.target.checked)} />}
          label="Show Robot State"
        />
      </Box>

      <Box component="fieldset">
        <legend>Import & Export</legend>
        <div style={{ marginTop: '0.5rem', display: 'flex', gap: '0.75rem', alignItems: 'center' }}>
          <Button
            variant="contained"
            color="primary"
            style={{ fontWeight: 'bold', width: '180px' }}
            onClick={() => {
              const poses = localStorage.getItem('savedPoses') || '[]';
              const filename = `thor_poses_${new Date().toISOString().replace(/[:.]/g, '-')}.json`;
              exportToFile(filename, poses);
            }}
          >
            Export Poses
          </Button>

          <input id="import-poses-input" type="file" accept="application/json" style={{ display: 'none' }}
            onChange={(e) => {
              const f = e.target.files && e.target.files[0]; if (!f) return;
              const reader = new FileReader();
              reader.onload = (evt) => {
                try {
                  const text = evt.target.result;
                  const parsed = JSON.parse(text);
                  if (!Array.isArray(parsed)) { showSnackbar('Archivo de poses inválido: se esperaba un array JSON', 'error'); return; }
                  if (!parsed.every(isValidPose)) { showSnackbar('Archivo de poses inválido: cada elemento debe tener "name" (string) y "joints" (objeto)', 'error'); return; }
                  let hasExisting = false; try { const existingRaw = localStorage.getItem('savedPoses'); const existing = existingRaw ? JSON.parse(existingRaw) : []; hasExisting = Array.isArray(existing) && existing.length > 0; } catch (_) { hasExisting = false; }
                  if (hasExisting) { setImportPosesCandidate(parsed); setImportPosesFileName(f.name || 'poses.json'); setImportPosesDialogOpen(true); }
                  else { localStorage.setItem('savedPoses', JSON.stringify(parsed)); try { window.dispatchEvent(new CustomEvent('posesImported', { detail: parsed })); } catch (e) {} showSnackbar('Poses imported successfully', 'success'); }
                } catch (err) { console.error('Import poses failed', err); showSnackbar('Failed to import poses: invalid JSON', 'error'); }
              };
              reader.readAsText(f);
              e.target.value = '';
            }}
          />

          <Button 
            variant="contained"
            color="primary"
            style={{ fontWeight: 'bold', width: '180px'}}
            onClick={() => {
              const inp = document.getElementById('import-poses-input'); if (inp) inp.click();
            }}
          >
            Import Poses
          </Button>

          <Dialog open={importPosesDialogOpen} onClose={handleImportPosesCancel} aria-labelledby="import-poses-dialog-title" aria-describedby="import-poses-dialog-description">
            <DialogTitle id="import-poses-dialog-title">Import poses</DialogTitle>
            <DialogContent>
              <DialogContentText id="import-poses-dialog-description">You are about to overwrite the existing saved poses with the contents of "{importPosesFileName}". This cannot be undone. Do you want to continue?</DialogContentText>
            </DialogContent>
            <DialogActions>
              <Button onClick={handleImportPosesCancel}>Cancel</Button>
              <Button onClick={handleImportPosesConfirm} color="primary" autoFocus>Import</Button>
            </DialogActions>
          </Dialog>

          <Dialog open={importDialogOpen} onClose={handleImportCancel} aria-labelledby="import-dialog-title" aria-describedby="import-dialog-description">
            <DialogTitle id="import-dialog-title">Import program</DialogTitle>
            <DialogContent>
              <DialogContentText id="import-dialog-description">You are about to overwrite the existing program with the contents of "{importFileName}". This cannot be undone. Do you want to continue?</DialogContentText>
            </DialogContent>
            <DialogActions>
              <Button onClick={handleImportCancel}>Cancel</Button>
              <Button onClick={handleImportConfirm} color="primary" autoFocus>Import</Button>
            </DialogActions>
          </Dialog>
        </div>

        <div style={{ marginTop: '1.0rem', display: 'flex', gap: '0.75rem', alignItems: 'center' }}>
          <Button
            variant="contained"
            color="warning"
            style={{ fontWeight: 'bold', width: '180px' }}
            onClick={() => {
              const program = localStorage.getItem('program') || '[]';
              const filename = `thor_program_${new Date().toISOString().replace(/[:.]/g, '-')}.json`;
              exportToFile(filename, program);
            }}
          >
            Export Program
          </Button>

          <input id="import-program-input" type="file" accept="application/json" style={{ display: 'none' }}
            onChange={(e) => {
              const f = e.target.files && e.target.files[0]; if (!f) return;
              const reader = new FileReader();
              reader.onload = (evt) => {
                try {
                  const text = evt.target.result;
                  const parsed = JSON.parse(text);
                  if (!Array.isArray(parsed)) { showSnackbar('Invalid program file: expected a JSON array', 'error'); return; }
                  if (!parsed.every(isValidMovement)) { showSnackbar('Invalid program file: each item must have "type" (string) and "pose" (string)', 'error'); return; }
                  let hasExisting = false; try { const existingRaw = localStorage.getItem('program'); const existing = existingRaw ? JSON.parse(existingRaw) : []; hasExisting = Array.isArray(existing) && existing.length > 0; } catch (_) { hasExisting = false; }
                  if (hasExisting) { setImportCandidate(parsed); setImportFileName(f.name || 'program.json'); setImportDialogOpen(true); }
                  else { localStorage.setItem('program', JSON.stringify(parsed)); try { window.dispatchEvent(new CustomEvent('programImported', { detail: parsed })); } catch (e) {} showSnackbar('Program imported successfully', 'success'); }
                } catch (err) { console.error('Import failed', err); showSnackbar('Failed to import program: invalid JSON', 'error'); }
              };
              reader.readAsText(f);
              e.target.value = '';
            }}
          />

          <Button 
            variant="contained"
            color="warning"
            style={{ fontWeight: 'bold', width: '180px' }}
            onClick={() => {
              const inp = document.getElementById('import-program-input'); if (inp) inp.click();
            }}
          >
            Import Program
          </Button>
        </div>

      </Box>

      <Snackbar open={snackbarOpen} autoHideDuration={4000} onClose={() => setSnackbarOpen(false)} anchorOrigin={{ vertical: 'bottom', horizontal: 'center' }}>
        <Alert onClose={() => setSnackbarOpen(false)} severity={snackbarSeverity} sx={{ width: '100%' }}>{snackbarMsg}</Alert>
      </Snackbar>
    </div>
  );
}
