import React, { useState, useEffect, useRef } from 'react';
import './App.css';
import { RosProvider } from './RosContext';
import UrdfViewer from './components/UrdfViewer';
import JointSliders from './components/JointSliders';
import IKSliders from './components/IkSliders';
import ROSLIB from 'roslib';
import Accordion from '@mui/material/Accordion';
import AccordionSummary from '@mui/material/AccordionSummary';
import AccordionDetails from '@mui/material/AccordionDetails';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import Settings from './components/Settings';
import Fab from '@mui/material/Fab';
import AddIcon from '@mui/icons-material/Add';
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
  const [activeTab, setActiveTab] = useState('forward');
  const [ikPose, setIkPose] = useState(null); // Estado compartido para la pose IK
  const [previewJoints, setPreviewJoints] = useState(null); // Estado para las articulaciones objetivo (IK)
  const [fkJoints, setFkJoints] = useState(null); // Estado para las articulaciones objetivo (FK)
  const [currentJoints, setCurrentJoints] = useState(null); // Estado articular actual del robot
  const [ghostJoints, setGhostJoints] = useState(null); // Estado para las articulaciones del ghost robot
  const [showRealRobot, setShowRealRobot] = useState(true);
  const [showGhostRobot, setShowGhostRobot] = useState(true);
  const [ikStatus, setIkStatus] = useState('reachable');
  const [isDialogOpen, setIsDialogOpen] = useState(false);
  const [poseName, setPoseName] = useState('');
  const [isMoving, setIsMoving] = useState(false);
  const [showFPS, setShowFPS] = useState(true); // Estado para controlar la visibilidad de los FPS
  const [showGhostRobotCoordinates, setShowGhostRobotCoordinates] = useState(true); // Estado para controlar la visibilidad de las flechas
  const [poses, setPoses] = useState(() => {
    const savedPoses = JSON.parse(localStorage.getItem('savedPoses')) || [];
    return savedPoses;
  });
  const rosRef = useRef(null);
  const lastJointsOnTabChange = useRef(null);
  const lastPoseOnTabChange = useRef(null);
  const ghostRef = useRef(null); // Referencia para el robot fantasma
  const poseRef = useRef(null); // Referencia para el componente Poses
  const randomRef = useRef(); // Referencia para el componente Random

  // Suscribirse a /joint_states para obtener la posición actual
  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
    rosRef.current = ros;
    const jointStateListener = new ROSLIB.Topic({
      ros,
      name: '/joint_states',
      messageType: 'sensor_msgs/msg/JointState'
    });
    let previousJoints = null; // Variable para almacenar el valor anterior de joints
    jointStateListener.subscribe((msg) => {
      // Mapear a objeto {joint_1: val, ...} con precisión de 4 decimales
      const joints = {};
      msg.name.forEach((name, i) => {
        joints[name] = parseFloat(msg.position[i].toFixed(4));
      });

      // Comprobar si los valores de joints son distintos a los anteriores antes de asignar
      if (JSON.stringify(previousJoints) !== JSON.stringify(joints)) {
        setCurrentJoints(joints);
        previousJoints = joints; // Actualizar el valor anterior
        setIsMoving(true); // Set isMoving to true when joints change
      } else {
        setIsMoving(false); // Set isMoving to false when joints stop changing
      }
    });
    return () => {
      jointStateListener.unsubscribe();
      ros.close();
    };
  }, []);

  // Solo inicializar sliders al cambiar de tab
  useEffect(() => {
    if (activeTab === 'forward' && currentJoints) {
      lastJointsOnTabChange.current = { ...currentJoints };
      setFkJoints({ ...currentJoints });
    }
    if (activeTab === 'inverse' && currentJoints) {
      // Llamar a servicio FK para obtener la pose cartesiana actual
      const ros = rosRef.current;
      if (!ros) return;
      const service = new ROSLIB.Service({
        ros,
        name: '/compute_fk',
        serviceType: 'moveit_msgs/srv/GetPositionFK'
      });
      const req = {
        header: { frame_id: 'base_link' },
        fk_link_names: ['gripper_base'],
        robot_state: {
          joint_state: {
            name: Object.keys(currentJoints),
            position: Object.values(currentJoints)
          }
        }
      };
      service.callService(req, (res) => {
        if (res && res.pose_stamped && res.pose_stamped.length > 0) {
          const pose = res.pose_stamped[0].pose;
          // Convertir quaternion a RPY
          const q = pose.orientation;
          const t = pose.position;
          function quaternionToRPY(q) {
            const ysqr = q.y * q.y;
            let t0 = +2.0 * (q.w * q.x + q.y * q.z);
            let t1 = +1.0 - 2.0 * (q.x * q.x + ysqr);
            let roll = Math.atan2(t0, t1);
            let t2 = +2.0 * (q.w * q.y - q.z * q.x);
            t2 = t2 > 1.0 ? 1.0 : t2;
            t2 = t2 < -1.0 ? -1.0 : t2;
            let pitch = Math.asin(t2);
            let t3 = +2.0 * (q.w * q.z + q.x * q.y);
            let t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
            let yaw = Math.atan2(t3, t4);
            return { roll, pitch, yaw };
          }
          const rpy = quaternionToRPY(q);
          const poseObj = {
            x: t.x * 1000,
            y: t.y * 1000,
            z: t.z * 1000,
            roll: rpy.roll * 180 / Math.PI,
            pitch: rpy.pitch * 180 / Math.PI,
            yaw: rpy.yaw * 180 / Math.PI
          };
          lastPoseOnTabChange.current = poseObj;
          setIkPose(poseObj);
        }
      });
    }
    // eslint-disable-next-line
  }, [activeTab]);

  // Console log para verificar el estado de los joints del ghost robot
  useEffect(() => {
    if (ghostRef.current) {
      console.log('Ghost Robot Joints:', ghostRef.current.getJointValues());
    }
  }, [ghostRef.current]); 

  // Determina qué preview mostrar
  const effectivePreviewJoints = activeTab === 'forward' ? fkJoints : previewJoints;

  const handleFabClick = () => {
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
        ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'].map(jointName => [jointName, currentJoints[jointName]])
      ),
      gripperBase: ikPose
    };

    const updatedPoses = [...poses, poseData];
    setPoses(updatedPoses);
    localStorage.setItem('savedPoses', JSON.stringify(updatedPoses));
    console.log('Pose saved:', poseData);

    if (poseRef.current) {
      poseRef.current.updatePoses();
      console.log('Poses updated in Poses component');
    }

    setIsDialogOpen(false);
    setPoseName('');
  };

  // Rediseñar la página para mostrar únicamente UrdfViewer a pantalla completa y un div flotante con JointSliders
  const memoizedOnPreviewJointsChange = React.useCallback((joints) => setPreviewJoints(joints), []);

  return (
    <RosProvider>
      <div className="app-container" style={{ display: 'flex', height: '100vh' }}>
        {/* Menú lateral fijo */}
        <div className="sidebar">
          <div style={{ textAlign: 'center', padding: '0.5rem' }}>
            <img src="/images/thor_logo.png" alt="Thor Logo" style={{ maxWidth: '20%', height: 'auto' }} />
          </div>
          {/* Accordions */}
          <Accordion
            expanded={activeTab === 'forward'}
            onChange={() => setActiveTab(activeTab === 'forward' ? '' : 'forward')}
            className="accordion forward-kinematics"
          >
            <AccordionSummary
              className='accordion-summary'
              expandIcon={<ExpandMoreIcon className="expand-icon" />}
            >
              <span className="accordion-title">Forward Kinematics</span>
            </AccordionSummary>
            <AccordionDetails>
              <JointSliders onPreviewJointsChange={setFkJoints} initialJoints={lastJointsOnTabChange.current} />
            </AccordionDetails>
          </Accordion>

          <Accordion
            expanded={activeTab === 'inverse'}
            onChange={() => {
              const newTab = activeTab === 'inverse' ? '' : 'inverse';
              setActiveTab(newTab);
              if (newTab === 'inverse' && lastPoseOnTabChange.current) {
                setIkPose(lastPoseOnTabChange.current);
              }
            }}
            className="accordion inverse-kinematics"
          >
            <AccordionSummary
              className='accordion-summary'
              expandIcon={<ExpandMoreIcon className="expand-icon" />}
            >
              <span className="accordion-title">Inverse Kinematics</span>
            </AccordionSummary>
            <AccordionDetails>
              <IKSliders 
                onPreviewJointsChange={setPreviewJoints} 
                initialPose={ikPose}
                ghostJoints={ghostJoints} // Pasar los valores del ghost a IKSliders
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
                showGhostRobot={showGhostRobot}
                setShowGhostRobot={setShowGhostRobot}
                showFPS={showFPS} // Pasar el estado showFPS
                setShowFPS={setShowFPS} // Pasar el setter de showFPS
                showGhostRobotCoordinates={showGhostRobotCoordinates} // Pasar el estado showGhostRobotCoordinates
                setShowGhostRobotCoordinates={setShowGhostRobotCoordinates} // Pasar el setter de showGhostRobotCoordinates
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
              <Poses ref={poseRef} ghostRef={ghostRef} onPreviewJointsChange={memoizedOnPreviewJointsChange} poses={poses} setPoses={setPoses} />
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

        {/* UrdfViewer ocupa el resto de la pantalla */}
        <div style={{ flex: 1, position: 'relative' }}>
          <UrdfViewer
            previewJoints={effectivePreviewJoints}
            showRealRobot={showRealRobot}
            showGhostRobot={showGhostRobot}
            ikStatus={ikStatus}
            ghostRef={ghostRef} // Asegurar que ghostRef se pase correctamente
            onGhostJointsChange={setGhostJoints} // Callback para recibir valores del ghost
            className="urdf-viewer"
            showFPS={showFPS} // Pasar el estado showFPS
            showGhostRobotCoordinates={showGhostRobotCoordinates} // Pasar el estado showGhostRobotCoordinates
          />
        </div>
      {/* Floating Action Button */}
        <Fab variant="extended"
          color="primary"
          aria-label="add"
          style={{ position: 'absolute', bottom: '35px', right: '35px', zIndex: 10, fontWeight: 'bold', fontSize: '0.95rem' }}
          onClick={handleFabClick}
        >
          <AddIcon />
          Pose
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