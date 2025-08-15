import React, { useState, useEffect, useRef } from 'react';
import './App.css';
import { RosProvider } from './RosContext';
import UrdfViewer from './components/UrdfViewer';
import JointSliders from './components/JointSliders';
import InverseKinematicsControls from './components/InverseKinematicsControls';
import ROSLIB from 'roslib';
import Accordion from '@mui/material/Accordion';
import AccordionSummary from '@mui/material/AccordionSummary';
import AccordionDetails from '@mui/material/AccordionDetails';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import Settings from './components/Settings';
import Fab from '@mui/material/Fab';
import SchoolIcon from '@mui/icons-material/School';
import FlipCameraAndroidIcon from '@mui/icons-material/FlipCameraAndroid';
import PrecisionManufacturingIcon from '@mui/icons-material/PrecisionManufacturing';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogContentText from '@mui/material/DialogContentText';
import DialogTitle from '@mui/material/DialogTitle';
import TextField from '@mui/material/TextField';
import Button from '@mui/material/Button';
import Poses from './components/Poses';
import Program from './components/Program';

function App() {
  const defaultSettings = {
    showRealRobot: true,
    showGhostRobot: true,
    showFPS: false,
    showGhostRobotCoordinates: true,
  };

  const [activeTab, setActiveTab] = useState('forward');
  const [initialIkPose] = useState(null);
  const [ikPreviewJoints, setIkPreviewJoints] = useState(null);
  const [fkJoints, setFkJoints] = useState(null);
  const [currentJoints, setCurrentJoints] = useState(null);
  const [ghostJoints, setGhostJoints] = useState(null);
  const [showRealRobot, setShowRealRobot] = useState(defaultSettings.showRealRobot);
  const [showGhostRobot, setShowGhostRobot] = useState(defaultSettings.showGhostRobot);
  const [showGhostRobotCoordinates, setShowGhostRobotCoordinates] = useState(defaultSettings.showGhostRobotCoordinates);
  const [showFPS, setShowFPS] = useState(defaultSettings.showFPS);
  const [ikStatus] = useState('reachable');
  const [isDialogOpen, setIsDialogOpen] = useState(false);
  const [poseName, setPoseName] = useState('');
  const [isMoving, setIsMoving] = useState(false);
  const [poses, setPoses] = useState(() => {
    const savedPoses = JSON.parse(localStorage.getItem('savedPoses')) || [];
    return savedPoses;
  });
  const ghostRobotRef = useRef(null); // Reference to the ghost robot
  const posesRef = useRef(null); // Reference to the Poses component
  const urdfApiRef = useRef(null);

  const [showOverlay, setShowOverlay] = useState(false);
  const [overlayRealTCP, setOverlayRealTCP] = useState(null);
  const [overlayGhostTCP, setOverlayGhostTCP] = useState(null);

  const jointOrder = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'];
  const radToDeg = (r) => (r * 180 / Math.PI);
  const formatArmJoints = (joints) => {
    if (!joints) return 'N/A';
    try {
      const vals = jointOrder.map((n) => {
        const v = joints[n];
        if (v === undefined || v === null) return 'N/A';
        return `${radToDeg(v).toFixed(1)}\u00b0`;
      });
      return vals.join(', ');
    } catch (e) { return 'N/A'; }
  };
  const formatGripper = (joints) => {
    if (!joints) return 'N/A';
    const g = joints['gripperbase_to_armgearright'];
    if (g === undefined || g === null) return 'N/A';
    return `${Math.round(Math.abs(radToDeg(g)))}\u00b0`;
  };
  const formatPosition = (tcp) => {
    if (!tcp) return 'N/A';
    const { x, y, z } = tcp;
    const factor = (Math.abs(x) < 10 && Math.abs(y) < 10 && Math.abs(z) < 10) ? 1000 : 1;
    return `${(x*factor).toFixed(1)}mm, ${(y*factor).toFixed(1)}mm, ${(z*factor).toFixed(1)}mm`;
  };
  const formatOrientation = (tcp) => {
    if (!tcp) return 'N/A';
    const { qx, qy, qz, qw } = tcp;
    return `${(qx||0).toFixed(1)}, ${(qy||0).toFixed(1)}, ${(qz||0).toFixed(1)}, ${(qw||0).toFixed(1)}`;
  };

  // Subscribe to /joint_states to obtain current joint positions
  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
    const jointStateListener = new ROSLIB.Topic({
      ros,
      name: '/joint_states',
      messageType: 'sensor_msgs/msg/JointState'
    });
    let previousJoints = null;
    jointStateListener.subscribe((msg) => {
      const joints = {};
      msg.name.forEach((name, i) => {
        joints[name] = parseFloat(msg.position[i].toFixed(4));
      });

      if (JSON.stringify(previousJoints) !== JSON.stringify(joints)) {
        setCurrentJoints(joints);
        previousJoints = joints;
        setIsMoving(true);
      } else {
        setIsMoving(false);
      }
    });
    return () => {
      jointStateListener.unsubscribe();
      ros.close();
    };
  }, []);

  const openSavePoseDialog = () => {
    setIsDialogOpen(true);
  };

  const handleDialogClose = () => {
    setIsDialogOpen(false);
  };

  const handleSavePose = () => {
    if (poseName.trim() === '') {
      alert('Pose name is required!');
      return;
    }

    // Check for duplicate pose name
    const isDuplicate = poses.some(pose => pose.name === poseName.trim());
    if (isDuplicate) {
      alert('A pose with this name already exists. Please choose a different name.');
      return;
    }

    const poseData = {
      name: poseName,
      joints: Object.fromEntries(
        ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'gripperbase_to_armgearright'].map(jointName => [jointName, currentJoints[jointName]])
      ),
      gripperBase: initialIkPose
    };

    const updatedPoses = [...poses, poseData];
    setPoses(updatedPoses);
    localStorage.setItem('savedPoses', JSON.stringify(updatedPoses));

    if (posesRef.current) {
      posesRef.current.updatePoses();
    }

    setIsDialogOpen(false);
    setPoseName('');
  };

  // Determine which preview joints to show (FK vs IK)
  const effectivePreviewJoints = activeTab === 'forward' ? fkJoints : ikPreviewJoints;

  const memoizedOnPreviewJointsChange = React.useCallback((joints) => setIkPreviewJoints(joints), []);

  // Keep overlay TCP values in sync. Try to use urdfApi methods when available.
  useEffect(() => {
    let mounted = true;
    const update = async () => {
      const api = urdfApiRef.current;
      if (!api) return;
      try {
        // Real TCP: prefer computing from currentJoints
        if (currentJoints && typeof api.getTCPFromJoints === 'function') {
          const t = api.getTCPFromJoints(currentJoints);
          if (mounted) setOverlayRealTCP(t || null);
        }
        // Ghost TCP: prefer explicit ghost state or compute from ghost joints
        if (ghostJoints && typeof api.getTCPFromJoints === 'function') {
          const tg = api.getTCPFromJoints(ghostJoints);
          if (mounted) setOverlayGhostTCP(tg || null);
        } else if (typeof api.getGhostState === 'function') {
          const s = api.getGhostState && api.getGhostState();
          if (s && s.tcp && mounted) setOverlayGhostTCP(s.tcp);
        }
      } catch (e) {
        // ignore
      }
    };
    update();
    const id = setInterval(update, 300);
    return () => { mounted = false; clearInterval(id); };
  }, [currentJoints, ghostJoints]);

  return (
    <RosProvider>
      <div className="app-container" style={{ display: 'flex', height: '100vh' }}>
        {/* Left sidebar */}
        <div className="sidebar">
          <div style={{ textAlign: 'center', padding: '0.5rem' }}>
            <img src="/images/thor_logo.png" alt="Thor Logo" style={{ maxWidth: '80%', height: 'auto' }} />
          </div>
          {/* Accordions */}
          <Accordion
            expanded={activeTab === 'forward'}
            onChange={() => setActiveTab(activeTab === 'forward' ? '' : 'forward')}
            className="accordion forward-kinematics"
          >
            <AccordionSummary className='accordion-summary' expandIcon={<ExpandMoreIcon className="expand-icon" />}> 
              <span className="accordion-title">Forward Kinematics</span>
            </AccordionSummary>
            <AccordionDetails>
              <JointSliders onPreviewJointsChange={setFkJoints} initialJoints={ghostJoints} urdfApi={urdfApiRef.current} active={activeTab === 'forward'} />
            </AccordionDetails>
          </Accordion>

          <Accordion
            expanded={activeTab === 'inverse'}
            onChange={() => {
              const newTab = activeTab === 'inverse' ? '' : 'inverse';
              setActiveTab(newTab);
            }}
            className="accordion inverse-kinematics"
          >
            <AccordionSummary className='accordion-summary' expandIcon={<ExpandMoreIcon className="expand-icon" />}> 
              <span className="accordion-title">Inverse Kinematics</span>
            </AccordionSummary>
            <AccordionDetails>
              <InverseKinematicsControls
                onPreviewJointsChange={setIkPreviewJoints}
                initialPose={initialIkPose}
                ghostJoints={ghostJoints}
                urdfApi={urdfApiRef.current}
                active={activeTab === 'inverse'}
              />
            </AccordionDetails>
          </Accordion>

          <Accordion
            expanded={activeTab === 'settings'}
            onChange={() => setActiveTab(activeTab === 'settings' ? '' : 'settings')}
            className="accordion settings"
          >
            <AccordionSummary
              className='accordion-summary'
              expandIcon={<ExpandMoreIcon className="expand-icon" />}
            >
              <span className="accordion-title">Settings</span>
            </AccordionSummary>
            <AccordionDetails>
              <Settings
                showRealRobot={showRealRobot}
                setShowRealRobot={setShowRealRobot}
                showFPS={showFPS} // pass showFPS state
                setShowFPS={setShowFPS} // pass showFPS setter
                showGhostRobot={showGhostRobot}
                setShowGhostRobot={setShowGhostRobot}
                showGhostRobotCoordinates={showGhostRobotCoordinates} // pass ghost robot coordinates flag
                setShowGhostRobotCoordinates={setShowGhostRobotCoordinates} // pass ghost robot coordinates setter
                showOverlay={showOverlay}
                setShowOverlay={setShowOverlay}
              />
            </AccordionDetails>
          </Accordion>

          <Accordion
            expanded={activeTab === 'poses'}
            onChange={() => setActiveTab(activeTab === 'poses' ? '' : 'poses')}
            className="accordion poses"
          >
            <AccordionSummary
              className='accordion-summary'
              expandIcon={<ExpandMoreIcon className="expand-icon" />}
            >
              <span className="accordion-title">Poses</span>
            </AccordionSummary>
            <AccordionDetails>
              <Poses ref={posesRef} ghostRef={ghostRobotRef} onPreviewJointsChange={memoizedOnPreviewJointsChange} poses={poses} setPoses={setPoses} />
            </AccordionDetails>
          </Accordion>

          <Accordion
            expanded={activeTab === 'program'}
            onChange={() => setActiveTab(activeTab === 'program' ? '' : 'program')}
            className="accordion program"
          >
            <AccordionSummary
              className='accordion-summary'
              expandIcon={<ExpandMoreIcon className="expand-icon" />}
            >
              <span className="accordion-title">Program</span>
            </AccordionSummary>
            <AccordionDetails>
              <Program isMoving={isMoving} poses={poses} />
            </AccordionDetails>
          </Accordion>
        </div>

        <div style={{ flex: 1, display: 'flex', flexDirection: 'column', position: 'relative', minHeight: 0 }}>
          <div style={{ flex: '1 1 auto', minHeight: 0, overflow: 'hidden' }}>
            <UrdfViewer
              ref={urdfApiRef}
              previewJoints={effectivePreviewJoints}
              showRealRobot={showRealRobot}
              showGhostRobot={showGhostRobot}
              ikStatus={ikStatus}
              onGhostJointsChange={setGhostJoints}
              className="urdf-viewer"
              showFPS={showFPS}
              showGhostRobotCoordinates={showGhostRobotCoordinates}
            />
          </div>

          {showOverlay && (
            <div style={{ position: 'absolute', bottom: '12px', left: 0, right: 0, zIndex: 11, display: 'flex', justifyContent: 'center' }}>
              <div id="robot-state-overlay" style={{ background: 'transparent', borderRadius: 8, padding: '0.5rem 0.9rem', boxShadow: 'none', fontFamily: 'monospace', fontSize: '0.85rem', width: 'calc(100% - 48px)', maxWidth: '1400px' }}>
                  <div style={{ marginBottom: '0.18rem' }}>
                    <strong style={{ fontSize: '1rem' }}>Real Robot</strong> - <strong><em>Joints</em></strong>: {formatArmJoints(currentJoints)} - <strong><em>Gripper</em></strong>: {formatGripper(currentJoints)} - <strong><em>Position</em></strong>: {formatPosition(overlayRealTCP)} - <strong><em>Orientation</em></strong>: {formatOrientation(overlayRealTCP)}
                  </div>
                <div>
                  <strong style={{ fontSize: '1rem' }}>Ghost Robot</strong> - <strong><em>Joints</em></strong>: {formatArmJoints(ghostJoints)} - <strong><em>Gripper</em></strong>: {formatGripper(ghostJoints)} - <strong><em>Position</em></strong>: {formatPosition(overlayGhostTCP)} - <strong><em>Orientation</em></strong>: {formatOrientation(overlayGhostTCP)}
                </div>
              </div>
            </div>
          )}
        </div>

        {/* TCP Gizmo Frame FAB */}
        {showGhostRobotCoordinates && (
          <Fab variant="extended"
            color="primary"
            aria-label="toggle-tcp-gizmo"
            style={{ position: 'absolute', bottom: '225px', right: '35px', zIndex: 10, fontWeight: 'bold', fontSize: '0.95rem', textTransform: 'none', display: 'flex', alignItems: 'center' }}
            onClick={() => { if (urdfApiRef.current && urdfApiRef.current.toggleTCPGizmoFrame) urdfApiRef.current.toggleTCPGizmoFrame(); }}
          >
            Toggle TCP Gizmo Frame
            <FlipCameraAndroidIcon style={{ marginLeft: '8px' }} />
          </Fab>
        )}

        {/* Move Ghost to Real FAB */}
        {showGhostRobot && (
          <Fab variant="extended"
            color="primary"
            aria-label="move-ghost-to-real"
            style={{ position: 'absolute', bottom: '155px', right: '35px', zIndex: 10, fontWeight: 'bold', fontSize: '0.95rem', textTransform: 'none', display: 'flex', alignItems: 'center' }}
            onClick={() => { if (urdfApiRef.current && urdfApiRef.current.copyRealToGhost) urdfApiRef.current.copyRealToGhost(); }}
          >
            Move Ghost to Real
            <PrecisionManufacturingIcon style={{ marginLeft: '8px' }} />
          </Fab>
        )}

        {/* Teach Real Pose FAB */}
        <Fab variant="extended"
          color="primary"
          aria-label="teach-pose"
          style={{ position: 'absolute', bottom: '85px', right: '35px', zIndex: 10, fontWeight: 'bold', fontSize: '0.95rem', textTransform: 'none', display: 'flex', alignItems: 'center' }}
          onClick={openSavePoseDialog}
        >
          Teach Real Pose
          <SchoolIcon style={{ marginLeft: '8px' }} />
        </Fab>

        {/* Form Dialog */}
        <Dialog open={isDialogOpen} onClose={handleDialogClose}>
          <DialogTitle>Save Pose</DialogTitle>
          <DialogContent>
            <DialogContentText>
              Enter a name for the pose:
            </DialogContentText>
            <TextField
              autoFocus
              margin="dense"
              label="Pose Name"
              type="text"
              fullWidth
              inputProps={{ maxLength: 12 }}
              value={poseName}
              onChange={(e) => setPoseName(e.target.value)}
            />
          </DialogContent>
          <DialogActions>
            <Button onClick={handleDialogClose} color="secondary">
              Cancel
            </Button>
            <Button onClick={handleSavePose} color="primary">
              Save
            </Button>
          </DialogActions>
        </Dialog>
      </div>
    </RosProvider>
  );
}

export default App;