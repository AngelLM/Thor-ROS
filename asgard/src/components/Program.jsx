import React, { useState, useEffect, useRef } from 'react';
import Button from '@mui/material/Button';
import Select from '@mui/material/Select';
import MenuItem from '@mui/material/MenuItem';
import Autocomplete from '@mui/material/Autocomplete';
import TextField from '@mui/material/TextField';
import FormControl from '@mui/material/FormControl';
import InputLabel from '@mui/material/InputLabel';
import Radio from '@mui/material/Radio';
import RadioGroup from '@mui/material/RadioGroup';
import FormControlLabel from '@mui/material/FormControlLabel';
import { useROS } from '../RosContext';
import ROSLIB from 'roslib';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogContentText from '@mui/material/DialogContentText';
import DialogTitle from '@mui/material/DialogTitle';
import ArrowForwardIosIcon from '@mui/icons-material/ArrowForwardIos';

let isRunAllInProgress = false; // Variable global para controlar el estado de ejecución de Run All

function Program({ isMoving, poses }) {
  const [movements, setMovements] = useState([]);
  const [poseNames, setPoseNames] = useState(poses.map(pose => pose.name));
  const [currentStep, setCurrentStep] = useState(null);
  const [isStepDisabled, setIsStepDisabled] = useState(false);
  const [selectedMovement, setSelectedMovement] = useState(null);
  const [dialogOpen, setDialogOpen] = useState(false);
  const [pendingSelection, setPendingSelection] = useState(null);
  const previousRobotPoseRef = useRef(null); // Usar useRef para almacenar la pose anterior del robot
  const currentRobotPoseRef = useRef(null); // Usar useRef para almacenar la pose actual del robot
  const [robotMoving, setRobotMoving] = useState(false); // Estado para indicar si el robot está en movimiento
  const { ros, connected } = useROS();

  useEffect(() => {
    const savedPoses = JSON.parse(localStorage.getItem('savedPoses')) || [];
    setPoseNames(savedPoses.map(pose => pose.name));
  }, []);

  useEffect(() => {
    const savedProgram = JSON.parse(localStorage.getItem('program')) || [];
    setMovements(savedProgram);
  }, []);

  useEffect(() => {
    if (!robotMoving && !isRunAllInProgress) {
      setIsStepDisabled(false); // Re-enable step buttons when robot stops moving
    }
  }, [robotMoving]);

  useEffect(() => {
    if (movements.length > 0 && selectedMovement === null) {
      setSelectedMovement(0); // Select the first Radio Button by default
    }
  }, [movements]);

  useEffect(() => {
    if (!connected || !ros) {
      console.warn('ROS no está conectado.');
      return;
    }

    console.log('Suscribiéndose al tópico /joint_states...');

    const jointStateListener = new ROSLIB.Topic({
      ros,
      name: '/joint_states', // Tópico que publica los estados de las articulaciones
      messageType: 'sensor_msgs/JointState',
    });

    // Modificar updateRobotPose para comparar con la pose anterior
    const updateRobotPose = (message) => {
      const jointPositions = {};
      message.name.forEach((name, index) => {
        jointPositions[name] = parseFloat(message.position[index].toFixed(4)); // Redondear a 4 decimales
      });

      if (previousRobotPoseRef.current) {
        const isSamePose = Object.keys(jointPositions).every(joint => {
          const currentAngle = jointPositions[joint];
          const previousAngle = previousRobotPoseRef.current[joint];

          if (currentAngle === undefined || previousAngle === undefined) {
            return false;
          }

          return Math.abs(currentAngle - previousAngle) < 0.0001; // Comparación con tolerancia de 4 decimales
        });

        setRobotMoving(!isSamePose); // Si la pose es la misma, el robot no se está moviendo
      }
      previousRobotPoseRef.current = jointPositions; // Actualizar la pose anterior en useRef
      currentRobotPoseRef.current = jointPositions; // Actualizar la pose actual en useRef
    };

    jointStateListener.subscribe(updateRobotPose);

    return () => {
      console.log('Desuscribiéndose del tópico /joint_states...');
      jointStateListener.unsubscribe();
    };
  }, [connected, ros]);

  const isPoseCurrent = (poseName) => {
    if (!currentRobotPoseRef.current) {
      console.warn('No se ha recibido la posición actual del robot.');
      return false;
    }

    const savedPoses = JSON.parse(localStorage.getItem('savedPoses')) || [];
    const targetPose = savedPoses.find(p => p.name === poseName);

    if (!targetPose) {
      console.warn(`La pose objetivo ${poseName} no se encuentra en las poses guardadas.`);
      return false;
    }

    const tolerance = 0.0001; // Tolerancia para la comparación (4 decimales)

    const isMatch = Object.keys(targetPose.joints).every(joint => {
      const currentAngle = currentRobotPoseRef.current[joint];
      const targetAngle = targetPose.joints[joint];

      if (currentAngle === undefined || targetAngle === undefined) {
        console.warn('The angle of joint', joint, 'is not defined in the current or target pose.');
        return false;
      }

      const match = Math.abs(currentAngle - targetAngle) < tolerance;
      return match;
    });

    return isMatch;
  };

  const saveProgramToLocalStorage = (updatedMovements) => {
    localStorage.setItem('program', JSON.stringify(updatedMovements));
  };

  const handleAddMovement = () => {
    const updatedMovements = [...movements, { pose: '', type: 'Joint' }];
    setMovements(updatedMovements);
    saveProgramToLocalStorage(updatedMovements);
  };

  const handleAddMovementBelow = (index) => {
    const updatedMovements = [
      ...movements.slice(0, index + 1),
      { pose: '', type: 'Joint' },
      ...movements.slice(index + 1)
    ];
    setMovements(updatedMovements);
    saveProgramToLocalStorage(updatedMovements);
  };

  const handleDeleteMovement = (index) => {
    const updatedMovements = movements.filter((_, i) => i !== index);
    setMovements(updatedMovements);
    saveProgramToLocalStorage(updatedMovements);
  };

  const handleChangeMovement = (index, field, value) => {
    const updatedMovements = movements.map((movement, i) =>
      i === index ? { ...movement, [field]: value } : movement
    );
    setMovements(updatedMovements);
    saveProgramToLocalStorage(updatedMovements);
  };

  const handleMoveUp = (index) => {
    if (index === 0) return;
    const updatedMovements = [...movements];
    [updatedMovements[index - 1], updatedMovements[index]] = [updatedMovements[index], updatedMovements[index - 1]];
    setMovements(updatedMovements);
    saveProgramToLocalStorage(updatedMovements);
  };

  const handleMoveDown = (index) => {
    if (index === movements.length - 1) return;
    const updatedMovements = [...movements];
    [updatedMovements[index + 1], updatedMovements[index]] = [updatedMovements[index], updatedMovements[index + 1]];
    setMovements(updatedMovements);
    saveProgramToLocalStorage(updatedMovements); // Fix incorrect syntax
  };

  const handleRunAll = async () => {
    isRunAllInProgress = true; // Marcar que Run All está en progreso
    setIsStepDisabled(true); // Disable the Start button at the beginning

    if (selectedMovement !== 0) {
      const userResponse = window.confirm(
        'The pointer is not at the first movement. Do you want to execute the program from the beginning?'
      );

      if (userResponse) {
        setSelectedMovement(0);
        setCurrentStep(0);
      }
    }

    for (let i = selectedMovement; i < movements.length; i++) {
      executeMovement(i); // Usar la función executeMovement para ejecutar cada movimiento
      const newPointer = i + 1 >= movements.length ? 0 : i + 1;
      setSelectedMovement(newPointer);
      setCurrentStep(i); // Highlight the executed movement
      // Esperar a que el movimiento se complete antes de continuar con un bucle de espera
      // Wait until the robot reaches the target pose
      while(!isPoseCurrent(movements[i].pose)) {
        console.log(`Waiting for robot to reach pose: ${movements[i].pose}`);
        // Esperar un breve período para evitar un bucle infinito
        await new Promise(resolve => setTimeout(resolve, 100));  
      }
      
    }

    isRunAllInProgress = false; // Marcar que Run All ha terminado
    setIsStepDisabled(false); // Re-enable the Start button after all movements are executed
    console.log('All movements executed.');
  };

  const handleStop = () => {
    console.log('Stop');
  };

  const handleStepFW = () => {
    const nextStep = selectedMovement === null ? 0 : selectedMovement;

    if (currentStep === movements.length - 1) {
      if (window.confirm('You have reached the last movement. Do you want to go to the first movement?')) {
        executeMovement(0);
        setCurrentStep(0); // Highlight the executed movement
        setSelectedMovement(1); // Pointer always points to the next movement
      }
      return;
    }

    executeMovement(nextStep);

    const newPointer = nextStep + 1 >= movements.length ? 0 : nextStep + 1;
    setSelectedMovement(newPointer);
    setCurrentStep(nextStep); // Highlight the executed movement
  };

  const handleStepBW = () => {
    const lastExecutedStep = currentStep;
    const prevStep = lastExecutedStep !== null ? lastExecutedStep - 1 : selectedMovement;

    if (prevStep < 0) {
      if (window.confirm('You have reached the first movement. Do you want to go to the last movement?')) {
        executeMovement(movements.length - 1);
        setCurrentStep(movements.length - 1); // Highlight the executed movement
        setSelectedMovement(0); // Pointer always points to the next movement
      }
      return;
    }

    executeMovement(prevStep);
    setCurrentStep(prevStep); // Highlight the executed movement
    setSelectedMovement(prevStep + 1 >= movements.length ? 0 : prevStep + 1); // Pointer always points to the next movement
  };

  const executeMovement = async (step) => {
    const movement = movements[step];
    const poseName = movement.pose;

    if (!connected || !ros) {
      console.warn('ROS is not connected.');
      return;
    }

    const savedPoses = JSON.parse(localStorage.getItem('savedPoses')) || [];
    const pose = savedPoses.find(p => p.name === poseName);

    if (!pose) {
      console.warn(`Pose ${poseName} not found.`);
      return;
    }

    const topic = new ROSLIB.Topic({
      ros,
      name: '/joint_group_position_controller/command',
      messageType: 'std_msgs/Float64MultiArray',
    });

    const message = new ROSLIB.Message({
      data: Object.values(pose.joints).slice(0, 6),
    });

    setIsStepDisabled(true); // Disable step buttons

    topic.publish(message);
    console.log(`Executing movement: ${step}`, movement);

    // Activate the RadioButton of the next movement
    const nextStep = step + 1 >= movements.length ? 0 : step + 1;
    setSelectedMovement(nextStep);

    if (isPoseCurrent(poseName) && !isRunAllInProgress) {
      console.log('El robot ya está en la pose objetivo después de ejecutar el movimiento. Habilitando botones.');
      setIsStepDisabled(false);
    }
  };

  const handleResetPointer = () => {
    setCurrentStep(null);
    console.log('Pointer reset to initial state.');
  };

  const handleSelectMovement = (index) => {
    setPendingSelection(index);
    setDialogOpen(true);
  };

  const handleDialogClose = (confirm) => {
    setDialogOpen(false);
    if (confirm) {
      setSelectedMovement(pendingSelection);
    }
    setPendingSelection(null);
  };

  useEffect(() => {
    setPoseNames(poses.map(pose => pose.name));
  }, [poses]);

  useEffect(() => {
    const updatedMovements = movements.map(movement => {
      if (!poseNames.includes(movement.pose)) {
        return { ...movement, pose: '' }; // Reset pose if it no longer exists
      }
      return movement;
    });
    setMovements(updatedMovements);
    saveProgramToLocalStorage(updatedMovements);
  }, [poseNames]);

  return (
    <div style={{ padding: '0.25rem' }}>
      <Dialog open={dialogOpen} onClose={() => handleDialogClose(false)}>
        <DialogTitle>Caution!</DialogTitle>
        <DialogContent>
          <DialogContentText>
            The robot may move unexpectedly when starting execution from an intermediate position in the program.
            Make sure the path is clear and there are no obstacles in the work area.
            Verify that the target positions and surroundings are correct before proceeding.
            Do you want to continue?
          </DialogContentText>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => handleDialogClose(false)} color="primary">
            Cancel
          </Button>
          <Button onClick={() => handleDialogClose(true)} color="primary" autoFocus>
            Confirm
          </Button>
        </DialogActions>
      </Dialog>

      {movements.length === 0 ? (
        <div style={{ textAlign: 'center', marginTop: '2rem' }}>
          <p style={{ fontSize: '1.2rem', color: '#555' }}>There are no movements programmed yet</p>
          <Button variant="contained" color="primary" onClick={handleAddMovement}>
            Add Movement
          </Button>
        </div>
      ) : (
        <div style={{ marginTop: '0.25rem' }}>
          {movements.map((movement, index) => (
            <div
              key={index}
              style={{
                display: 'flex',
                alignItems: 'center',
                marginBottom: '1rem',
                marginLeft: '0.3rem',
                marginRight: '0.3rem',
                backgroundColor: currentStep === index ? 'lightyellow' : 'transparent',
              }}
            >
              <RadioGroup
                value={selectedMovement}
                onChange={() => handleSelectMovement(index)}
                style={{ borderRadius: '50%' }}
              >
                <FormControlLabel
                  value={index}
                  control={
                    <Radio
                      style={{ backgroundColor: selectedMovement === index ? 'yellow' : 'transparent', borderRadius: '50%' }}
                      icon={<ArrowForwardIosIcon style={{ color: selectedMovement === index ? 'black' : 'lightgray' }} />}
                      checkedIcon={<ArrowForwardIosIcon style={{ color: 'black' }} />}
                    />
                  }
                  label=""
                />
              </RadioGroup>

              <div style={{ display: 'flex', flexDirection: 'column', marginRight: '0.3rem' }}>
                <Button
                  variant="contained"
                  style={{ backgroundColor: 'green', color: 'white', padding: '0.3rem', minWidth: '36px', marginBottom: '0.2rem' }}
                  onClick={() => handleAddMovementBelow(index)}
                >
                  <span className="material-icons" style={{ fontSize: '20px' }}>add</span>
                </Button>
                <Button
                  variant="contained"
                  color="error"
                  onClick={() => handleDeleteMovement(index)}
                  style={{ padding: '0.3rem', minWidth: '36px' }}
                >
                  <span className="material-icons" style={{ fontSize: '20px' }}>delete</span>
                </Button>
              </div>

              <div style={{ display: 'flex', flexDirection: 'column', marginRight: '0.3rem' }}>
                <Button
                  variant="contained"
                  style={{ backgroundColor: 'blue', color: 'white', padding: '0.3rem', minWidth: '36px', marginBottom: '0.2rem' }}
                  onClick={() => handleMoveUp(index)}
                  disabled={index === 0}
                >
                  <span className="material-icons" style={{ fontSize: '20px' }}>arrow_upward</span>
                </Button>

                <Button
                  variant="contained"
                  style={{ backgroundColor: 'blue', color: 'white', padding: '0.3rem', minWidth: '36px' }}
                  onClick={() => handleMoveDown(index)}
                  disabled={index === movements.length - 1}
                >
                  <span className="material-icons" style={{ fontSize: '20px' }}>arrow_downward</span>
                </Button>
              </div>

              <Autocomplete
                options={poseNames}
                value={movement.pose}
                onChange={(event, newValue) => handleChangeMovement(index, 'pose', newValue)}
                renderInput={(params) => (
                  <TextField {...params} label="Pose" variant="outlined" style={{ marginLeft: '0.5rem', width: '180px', paddingTop: '0', paddingBottom: '0' }} />
                )}
              />

              <FormControl style={{ marginLeft: '0.5rem', width: '60px', minWidth: '60px' }} variant="outlined">
                <InputLabel>Type</InputLabel>
                <Select
                  value={movement.type}
                  onChange={(e) => handleChangeMovement(index, 'type', e.target.value)}
                  label="Type"
                >
                  <MenuItem value="J">J</MenuItem>
                  <MenuItem value="L">L</MenuItem>
                </Select>
              </FormControl>
            </div>
          ))}

          <div style={{ display: 'flex', justifyContent: 'center', marginTop: '1rem' }}>
            <Button
              variant="contained"
              style={{ 
                backgroundColor: isStepDisabled ? 'lightgray' : 'yellow', 
                color: isStepDisabled ? 'darkgray' : 'black', 
                marginRight: '0.5rem', 
                fontSize: '1rem', 
                fontWeight: 'bold',
                cursor: isStepDisabled ? 'not-allowed' : 'pointer'
              }}
              onClick={handleStepFW}
              disabled={isStepDisabled} // Disable button when step is in progress
            >
              Step FW
            </Button>
            <Button
              variant="contained"
              style={{ 
                backgroundColor: isStepDisabled ? 'lightgray' : 'orange', 
                color: isStepDisabled ? 'darkgray' : 'white', 
                marginRight: '0.5rem', 
                fontSize: '1rem', 
                fontWeight: 'bold',
                cursor: isStepDisabled ? 'not-allowed' : 'pointer'
              }}
              onClick={handleStepBW}
              disabled={isStepDisabled} // Disable button when step is in progress
            >
              Step BW
            </Button>
            <Button
              variant="contained"
              style={{
                backgroundColor: isStepDisabled ? 'lightgray' : 'gray',
                color: isStepDisabled ? 'darkgray' : 'white',
                fontSize: '1rem',
                fontWeight: 'bold',
                cursor: isStepDisabled ? 'not-allowed' : 'pointer'
              }}
              onClick={handleResetPointer}
              disabled={isStepDisabled} // Disable button when step is in progress
            >
              Reset
            </Button>
          </div>

          <div style={{ display: 'flex', justifyContent: 'center', marginTop: '1rem' }}>
            <Button
              variant="contained"
              style={{
                backgroundColor: isStepDisabled ? 'lightgray' : 'green',
                color: isStepDisabled ? 'darkgray' : 'white',
                marginRight: '0.5rem',
                fontSize: '1rem',
                fontWeight: 'bold',
                cursor: isStepDisabled ? 'not-allowed' : 'pointer'
              }}
              onClick={handleRunAll}
              disabled={isStepDisabled} // Disable button when step is in progress
            >
              Run All
            </Button>
            <Button
              variant="contained"
              style={{ backgroundColor: 'red', color: 'white', fontSize: '1rem', fontWeight: 'bold' }}
              onClick={handleStop}
            >
              Stop
            </Button>
          </div>
        </div>
      )}
    </div>
  );
}

export default Program;
