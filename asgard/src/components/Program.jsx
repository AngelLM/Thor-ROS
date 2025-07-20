import React, { useState, useEffect } from 'react';
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

function Program({ isMoving }) {
  const [movements, setMovements] = useState([]);
  const [poseNames, setPoseNames] = useState([]);
  const [currentStep, setCurrentStep] = useState(null);
  const [isStepDisabled, setIsStepDisabled] = useState(false);
  const [selectedMovement, setSelectedMovement] = useState(null);
  const [dialogOpen, setDialogOpen] = useState(false);
  const [pendingSelection, setPendingSelection] = useState(null);
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
    console.log('isMoving', isMoving);
    if (!isMoving) {
      setIsStepDisabled(false); // Re-enable step buttons when robot stops moving
    }
  }, [isMoving]);

  useEffect(() => {
    if (movements.length > 0 && selectedMovement === null) {
      setSelectedMovement(0); // Select the first Radio Button by default
    }
  }, [movements]);

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
    saveProgramToLocalStorage(updatedMovements);
  };

  const handleRunAll = () => {
    console.log('Run All');
  };

  const handleRunStep = () => {
    const nextStep = currentStep === null ? 0 : currentStep + 1;
    if (nextStep >= movements.length) return; // No more steps to run

    const movement = movements[nextStep];
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

    topic.publish(message);
    console.log(`Running step: ${nextStep}`, movement);

    setCurrentStep(nextStep);
  };

  const handleStop = () => {
    console.log('Stop');
  };

  const handleStepFW = () => {
    const nextStep = currentStep === null ? 0 : currentStep + 1;

    if (nextStep >= movements.length) {
      if (window.confirm('You have reached the last movement. Do you want to start from the beginning?')) {
        setCurrentStep(0);
        executeMovement(0);
      }
      return;
    }

    setCurrentStep(nextStep);
    executeMovement(nextStep);

    // Force re-evaluation of isMoving
    setTimeout(() => {
      if (!isMoving) {
        setIsStepDisabled(false); // Re-enable step buttons when robot stops moving
      }
    }, 1000);
  };

  const handleStepBW = () => {
    const prevStep = currentStep === null ? movements.length - 1 : currentStep - 1;

    if (prevStep < 0) {
      if (window.confirm('You have reached the first movement. Do you want to go to the last movement?')) {
        setCurrentStep(movements.length - 1);
        executeMovement(movements.length - 1);
      }
      return;
    }

    setCurrentStep(prevStep);
    executeMovement(prevStep);

    // Force re-evaluation of isMoving
    setTimeout(() => {
      if (!isMoving) {
        setIsStepDisabled(false); // Re-enable step buttons when robot stops moving
      }
    }, 1000);
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
              style={{ backgroundColor: 'gray', color: 'white', fontSize: '1rem', fontWeight: 'bold' }}
              onClick={handleResetPointer}
            >
              Reset
            </Button>
          </div>

          <div style={{ display: 'flex', justifyContent: 'center', marginTop: '1rem' }}>
            <Button
              variant="contained"
              style={{ backgroundColor: 'green', color: 'white', marginRight: '0.5rem', fontSize: '1rem', fontWeight: 'bold' }}
              onClick={handleRunAll}
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
