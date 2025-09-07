import { useState, useEffect, useRef } from 'react';
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
import useRosApi from '../ros/useRosApi';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogContentText from '@mui/material/DialogContentText';
import DialogTitle from '@mui/material/DialogTitle';
import ArrowForwardIosIcon from '@mui/icons-material/ArrowForwardIos';

let isRunAllInProgress = false; // Global flag to track "Run All" execution state

function Program({ poses }) {
  const [movements, setMovements] = useState([]);
  const [poseNames, setPoseNames] = useState(poses.map(pose => pose.name));
  const [currentStep, setCurrentStep] = useState(null);
  const [controlsDisabled, setControlsDisabled] = useState(false);
  const [pointerIndex, setPointerIndex] = useState(null);
  const [selectionDialogOpen, setSelectionDialogOpen] = useState(false);
  const [pendingSelectionIndex, setPendingSelectionIndex] = useState(null);
  const previousRobotPoseRef = useRef(null); // Previous robot pose (useRef)
  const currentRobotPoseRef = useRef(null); // Current robot pose (useRef)
  const [robotMoving, setRobotMoving] = useState(false); // Whether the robot is moving
  const rosApi = useRosApi();
  const [runAllDialogOpen, setRunAllDialogOpen] = useState(false);

  // Load saved poses from localStorage on mount
  useEffect(() => {
    const savedPoses = JSON.parse(localStorage.getItem('savedPoses')) || [];
    setPoseNames(savedPoses.map(pose => pose.name));
  }, []);

  // Load saved program from localStorage on mount
  useEffect(() => {
    const savedProgram = JSON.parse(localStorage.getItem('program')) || [];
    setMovements(savedProgram);
  }, []);

  // Listen for program import events (dispatched by Settings)
  useEffect(() => {
    const handler = (e) => {
      try {
        const imported = e.detail || JSON.parse(localStorage.getItem('program') || '[]');
        if (!Array.isArray(imported)) return;
        const normalized = imported.map(m => ({ type: m.type || 'J', pose: m.pose || '', speed: m.speed ?? 100 }));
        setMovements(normalized);
        saveProgramToLocalStorage(normalized);
      } catch (err) {
        console.error('Error handling imported program', err);
      }
    };
    window.addEventListener('programImported', handler);
    return () => window.removeEventListener('programImported', handler);
  }, []);

  // Monitor robotMoving state to disable/enable controls
  useEffect(() => {
    if (!robotMoving && !isRunAllInProgress) {
      setControlsDisabled(false);
    }
  }, [robotMoving]);

  // Auto-select first movement if none selected
  useEffect(() => {
    if (movements.length > 0 && pointerIndex === null) {
      setPointerIndex(0);
    }
  }, [movements]);

  // Subscribe to /joint_states to monitor robot movement
  useEffect(() => {
    if (!rosApi.connected) {
      console.warn('ROS is not connected.');
      return;
    }

    console.log('Subscribing to /joint_states...');
    const unsubscribe = rosApi.subscribeJointStates((message) => {
      const jointPositions = {};
      message.name.forEach((name, index) => {
        jointPositions[name] = parseFloat(message.position[index].toFixed(4));
      });
      if (previousRobotPoseRef.current) {
        const isSamePose = Object.keys(jointPositions).every(joint => {
          const currentAngle = jointPositions[joint];
          const previousAngle = previousRobotPoseRef.current[joint];
          if (currentAngle === undefined || previousAngle === undefined) return false;
          return Math.abs(currentAngle - previousAngle) < 0.0001;
        });
        setRobotMoving(!isSamePose);
      }
      previousRobotPoseRef.current = jointPositions;
      currentRobotPoseRef.current = jointPositions;
    });
    return () => {
      console.log('Unsubscribing from /joint_states...');
      try { unsubscribe && unsubscribe(); } catch (_) {}
    };
  }, [rosApi.connected]);

  const isPoseCurrent = (poseName) => {
    if (!currentRobotPoseRef.current) {
      console.warn('Current robot position has not been received yet.');
      return false;
    }

    const savedPoses = JSON.parse(localStorage.getItem('savedPoses')) || [];
    const targetPose = savedPoses.find(p => p.name === poseName);

    if (!targetPose) {
      console.warn(`The target pose ${poseName} is not found in the saved poses.`);
      return false;
    }

    const tolerance = 0.0001; // Tolerance for comparison (4 decimals)

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
  const updatedMovements = [...movements, { pose: '', type: 'J', speed: 100 }]; // Default type set to 'J' and speed 100
    setMovements(updatedMovements);
    saveProgramToLocalStorage(updatedMovements);
  };

  const handleAddMovementBelow = (index) => {
    const updatedMovements = [
  ...movements.slice(0, index + 1),
  { pose: '', type: 'J', speed: 100 }, // Default type set to 'J' and speed 100
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

  const handleRunAll = async () => {
    // Check if all movements have an associated pose
    const movementsWithoutPose = movements.filter(movement => !movement.pose);
    if (movementsWithoutPose.length > 0) {
      alert('Some movements do not have an associated pose. Please ensure all movements have a pose before running.');
      return;
    }

  if (pointerIndex !== 0) {
      setRunAllDialogOpen(true);
      return; // Wait for user input from the dialog
    }

  await executeRunAll();
  };

  const executeRunAll = async (startIndex = 0) => {
    isRunAllInProgress = true; // Mark that Run All is in progress
    setControlsDisabled(true); // Disable controls at the beginning

    for (let i = startIndex; i < movements.length; i++) {
      if (!isRunAllInProgress) {
        console.log('Execution interrupted by Stop command.');
        break; // Exit the main loop if Stop is pressed
      }

      setCurrentStep(i); // Highlight the current movement
      const newPointer = i + 1 >= movements.length ? 0 : i + 1;
      setPointerIndex(newPointer);

      console.log(`Executing movement ${i + 1}/${movements.length}: ${movements[i].pose} (${movements[i].type})`);
      
      // Execute the movement and wait for completion
      const result = await executeMovement(i);
      
      if (!result.success) {
        console.error(`Movement ${i + 1} failed:`, result.error);
        alert(`Movement ${i + 1} failed: ${result.error}\nStopping program execution.`);
        break; // Stop execution on error
      }

      console.log(`Movement ${i + 1} completed successfully:`, result.message);

      // Add a small delay between movements to ensure robot stabilization
      // This is especially important for cartesian movements that need accurate current position
      if (i < movements.length - 1) { // Don't delay after the last movement
        const currentMovementType = movements[i]?.type;
        const nextMovementType = movements[i + 1]?.type;
        
        if (nextMovementType === 'L') {
          console.log('Next movement is cartesian, adding stabilization delay...');
          if (currentMovementType === 'J') {
            // Extra delay when going from Joint to Cartesian movement
            await new Promise(resolve => setTimeout(resolve, 1500)); // 2000ms after joint movement before cartesian
          } else {
            await new Promise(resolve => setTimeout(resolve, 200)); // 1000ms after cartesian before cartesian
          }
        } else {
          await new Promise(resolve => setTimeout(resolve, 200)); // 200ms for joint movements
        }
      }

      if (!isRunAllInProgress) {
        console.log('Execution interrupted by Stop command.');
        break; // Exit the main loop if Stop is pressed
      }
    }

    isRunAllInProgress = false; // Mark that Run All has finished
    setControlsDisabled(false); // Re-enable controls after all movements are executed
    console.log('All movements executed.');
  };

  const handleStop = () => {
    // Stop the Run All process
    isRunAllInProgress = false;
    setControlsDisabled(false); // Re-enable buttons

  if (!rosApi.connected) {
      console.warn('ROS is not connected. Cannot send stop command.');
      return;
    }
  rosApi.publishStopEvent();

    console.log('Stop command sent to /trajectory_execution_event.');
  };

  const handleStepForward = async () => {
  const nextStep = pointerIndex === null ? 0 : pointerIndex;
  console.log('[Program] Step FW requested, pointerIndex=', pointerIndex, 'nextStep=', nextStep);

    if (!movements[nextStep]?.pose) {
      alert('The selected movement does not have a defined pose. Please select a pose before proceeding.');
      return;
    }

    if (currentStep === movements.length - 1) {
      if (window.confirm('You have reached the last movement. Do you want to go to the first movement?')) {
        const result = await executeMovement(0);
        if (result.success) {
          setCurrentStep(0); // Highlight the executed movement
          setPointerIndex(1); // Pointer always points to the next movement
        } else {
          alert(`Movement failed: ${result.error}`);
        }
      }
      return;
    }

    const result = await executeMovement(nextStep);
    
    if (result.success) {
      const newPointer = nextStep + 1 >= movements.length ? 0 : nextStep + 1;
      setPointerIndex(newPointer);
      setCurrentStep(nextStep); // Highlight the executed movement
    } else {
      alert(`Movement failed: ${result.error}`);
    }
  };

  const handleStepBackward = async () => {
    const lastExecutedStep = currentStep;
    const prevStep = lastExecutedStep !== null ? lastExecutedStep - 1 : pointerIndex;
  console.log('[Program] Step BW requested, pointerIndex=', pointerIndex, 'prevStep=', prevStep);

    if (prevStep < 0) {
      if (!movements[movements.length - 1]?.pose) {
        alert('The last movement does not have a defined pose. Please select a pose before proceeding.');
        return;
      }

      if (window.confirm('You have reached the first movement. Do you want to go to the last movement?')) {
        const result = await executeMovement(movements.length - 1);
        if (result.success) {
          setCurrentStep(movements.length - 1); // Highlight the executed movement
          setPointerIndex(0); // Pointer always points to the next movement
        } else {
          alert(`Movement failed: ${result.error}`);
        }
      }
      return;
    }

    if (!movements[prevStep]?.pose) {
      alert('The selected movement does not have a defined pose. Please select a pose before proceeding.');
      return;
    }

    const result = await executeMovement(prevStep);
    
    if (result.success) {
      setCurrentStep(prevStep); // Highlight the executed movement
      setPointerIndex(prevStep + 1 >= movements.length ? 0 : prevStep + 1); // Pointer always points to the next movement
    } else {
      alert(`Movement failed: ${result.error}`);
    }
  };

  const executeMovement = async (step) => {
    const movement = movements[step];
    const poseName = movement.pose;
    const movementType = movement.type || 'J'; // Default to Joint movement if type is not specified

    if (!rosApi.connected) {
      console.warn('ROS is not connected.');
      return { success: false, error: 'ROS not connected' };
    }

    const savedPoses = JSON.parse(localStorage.getItem('savedPoses')) || [];
    const pose = savedPoses.find(p => p.name === poseName);

    if (!pose) {
      console.warn(`Pose ${poseName} not found.`);
      return { success: false, error: `Pose ${poseName} not found` };
    }

  setControlsDisabled(true); // Disable controls while executing

  try {
  if (movementType === 'L') {
        // Handle cartesian movement (Type: L)
        const gripperValue = pose.joints['gripperbase_to_armgearright'] || 0;
        
        // Use the saved TCP coordinates from pose.gripperBase
        if (!pose.gripperBase) {
          console.error('No TCP coordinates available for cartesian movement');
          // Fall back to joint movement
          const jointOrder = Object.keys(pose.joints);
          rosApi.publishJointGroupCommand(jointOrder, pose.joints);
          console.log(`Executing movement: ${step}`, movement, '(fallback to joint)');
          
          // Wait for joint movement to complete
          return await rosApi.waitForMovementCompletion(30000, isRunAllInProgress ? () => !isRunAllInProgress : () => false);
        } else {
          // Execute cartesian movement
          console.log(`Executing cartesian movement: ${step}`, movement);
          const result = await rosApi.publishCartesianGoal(
            { x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 }, // Start pose (will be calculated from current position)
            {
              x: pose.gripperBase.x || pose.gripperBase.position?.x || 0,
              y: pose.gripperBase.y || pose.gripperBase.position?.y || 0,
              z: pose.gripperBase.z || pose.gripperBase.position?.z || 0,
              qx: pose.gripperBase.qx || pose.gripperBase.orientation?.x || 0,
              qy: pose.gripperBase.qy || pose.gripperBase.orientation?.y || 0,
              qz: pose.gripperBase.qz || pose.gripperBase.orientation?.z || 0,
              qw: pose.gripperBase.qw || pose.gripperBase.orientation?.w || 1
            },
            {
              gripperValue: gripperValue,
              maxStep: 0.005, // Reduced from 0.01 for smoother execution
              jumpThreshold: 0.0,
              avoidCollisions: false
            }
          );

          if (!result.ok) {
            console.error('Cartesian movement failed:', result.message || result.error);
            // Fall back to joint movement
            console.log('Falling back to joint movement...');
            const jointOrder = Object.keys(pose.joints);
            rosApi.publishJointGroupCommand(jointOrder, pose.joints);
            
              // Wait for fallback joint movement to complete
              console.log('[Program] waiting for fallback joint movement, isRunAllInProgress=', isRunAllInProgress);
              return await rosApi.waitForMovementCompletion(30000, isRunAllInProgress ? () => !isRunAllInProgress : () => false);
          } else {
            console.log('Cartesian movement completed successfully');
            if (result.waypoints && result.waypoints.length > 0) {
              console.log(`Generated ${result.waypoints.length} waypoints for the trajectory`);
            }
            return { success: true, message: 'Cartesian movement completed' };
          }
        }
      } else {
        // Handle joint movement (Type: J or default)
        const jointOrder = Object.keys(pose.joints);
        rosApi.publishJointGroupCommand(jointOrder, pose.joints);
        console.log(`Executing joint movement: ${step}`, movement);
        
            // Wait for joint movement to complete
            console.log('[Program] waiting for joint movement, isRunAllInProgress=', isRunAllInProgress);
            return await rosApi.waitForMovementCompletion(30000, isRunAllInProgress ? () => !isRunAllInProgress : () => false);
      }
    } catch (error) {
      console.error('Error during movement execution:', error);
      // Fall back to joint movement on error
      const jointOrder = Object.keys(pose.joints);
      rosApi.publishJointGroupCommand(jointOrder, pose.joints);
      
      // Wait for fallback joint movement to complete
      console.log('[Program] waiting for fallback joint movement after exception, isRunAllInProgress=', isRunAllInProgress);
      return await rosApi.waitForMovementCompletion(30000, isRunAllInProgress ? () => !isRunAllInProgress : () => false);
    }
    finally {
      // Ensure controls are re-enabled after this single-step execution finishes
      // Run All manages controlsDisabled itself; only re-enable if Run All is not in progress.
      if (!isRunAllInProgress) {
        setControlsDisabled(false);
      }
    }
  };

  const openSelectionDialog = (index) => {
    setPendingSelectionIndex(index);
    setSelectionDialogOpen(true);
  };

  const handleSelectionDialogClose = (confirm) => {
    setSelectionDialogOpen(false);
    if (confirm) {
      setPointerIndex(pendingSelectionIndex);
    }
    setPendingSelectionIndex(null);
  };

  useEffect(() => {
    setPoseNames(poses.map(pose => pose.name));
  }, [poses]);

  useEffect(() => {
    if (movements.length === 0) return; // Prevent overwriting movements loaded from localStorage

    const updatedMovements = movements.map(movement => {
      if (!poseNames.includes(movement.pose)) {
        return { ...movement, pose: '', speed: movement.speed ?? 100 }; // Reset pose if it no longer exists
      }
      return { ...movement, speed: movement.speed ?? 100 };
    });
    setMovements(updatedMovements);
    saveProgramToLocalStorage(updatedMovements);
  }, [poseNames]);

  useEffect(() => {
    if (currentStep !== null) {
      const currentElement = document.querySelector(`.scrollable-container > div:nth-child(${currentStep + 1})`);
      if (currentElement) {
        currentElement.scrollIntoView({ behavior: 'smooth', block: 'center' });
      }
    }
  }, [currentStep]);

  useEffect(() => {
    if (!isRunAllInProgress) {
      const scrollableContainer = document.querySelector('.scrollable-container');
      if (scrollableContainer) {
        scrollableContainer.scrollTo({ top: 0, behavior: 'smooth' });
      }
    }
  }, [isRunAllInProgress]);

  return (
    <div>
    <Dialog open={selectionDialogOpen} onClose={() => handleSelectionDialogClose(false)}>
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
          <Button onClick={() => handleSelectionDialogClose(false)} color="primary">
            Cancel
          </Button>
          <Button onClick={() => handleSelectionDialogClose(true)} color="primary" autoFocus>
            Confirm
          </Button>
        </DialogActions>
      </Dialog>

      <Dialog open={runAllDialogOpen} onClose={() => setRunAllDialogOpen(false)}>
        <DialogTitle>Run All</DialogTitle>
        <DialogContent>
          <DialogContentText>
            The pointer is not at the first movement. Do you want to execute the program from the beginning or from the pointer?
          </DialogContentText>
        </DialogContent>
        <DialogActions>
          <Button
            onClick={() => {
              console.log('Running all movements from the beginning.');
              setPointerIndex(0);
              setCurrentStep(0);
              setRunAllDialogOpen(false);

              executeRunAll(0);
            }}
            color="primary"
          >
            Execute from the beginning
          </Button>
          <Button
            onClick={() => {
              // setRunAllFromBeginning(false);
              setRunAllDialogOpen(false);
              executeRunAll(pointerIndex);
            }}
            color="secondary"
          >
            Execute from the pointer
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
          <div className="scrollable-container">
            {movements.map((movement, index) => (
              <div
                key={index}
                style={{
                  display: 'flex',
                  alignItems: 'center',
                  marginBottom: '1rem',
                  marginLeft: '0.5rem',
                  marginRight: '0.3rem',
                  backgroundColor: currentStep === index ? 'lightyellow' : 'transparent',
                }}
              >
                <RadioGroup
                  value={pointerIndex}
                  onChange={() => openSelectionDialog(index)}
                  style={{ borderRadius: '50%' }}
                >
                  <FormControlLabel
                    value={index}
                    control={
                      <Radio
                        style={{ backgroundColor: pointerIndex === index ? 'yellow' : 'transparent', borderRadius: '50%' }}
                        icon={<ArrowForwardIosIcon style={{ color: pointerIndex === index ? 'black' : 'lightgray' }} />}
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
                {/*
                <TextField
                  className="speed-input"
                  label="Speed (%)"
                  variant="outlined"
                  type="number"
                  value={movement.speed ?? 100}
                  onChange={(e) => {
                    let val = parseInt(e.target.value, 10);
                    if (isNaN(val)) val = 100;
                    if (val < 1) val = 1;
                    if (val > 100) val = 100;
                    handleChangeMovement(index, 'speed', val);
                  }}
                  style={{ marginLeft: '0.5rem', width: '85px'}}
                  InputProps={{ inputProps: { min: 1, max: 100, step: 1 } }}
                />
                */}
              </div>
            ))}
          </div>

          <div style={{ display: 'flex', justifyContent: 'center', marginTop: '1rem' }}>
            <Button
              variant="contained"
              style={{ 
                backgroundColor: controlsDisabled ? 'lightgray' : 'yellow', 
                color: controlsDisabled ? 'darkgray' : 'black', 
                marginRight: '0.5rem', 
                fontSize: '1rem', 
                fontWeight: 'bold',
                cursor: controlsDisabled ? 'not-allowed' : 'pointer'
              }}
              onClick={handleStepForward}
              disabled={controlsDisabled} // Disable button when step is in progress
            >
              Step FW
            </Button>
            <Button
              variant="contained"
              style={{ 
                backgroundColor: controlsDisabled ? 'lightgray' : 'orange', 
                color: controlsDisabled ? 'darkgray' : 'white', 
                marginRight: '0.5rem', 
                fontSize: '1rem', 
                fontWeight: 'bold',
                cursor: controlsDisabled ? 'not-allowed' : 'pointer'
              }}
              onClick={handleStepBackward}
              disabled={controlsDisabled} // Disable button when step is in progress
            >
              Step BW
            </Button>
          </div>

          <div style={{ display: 'flex', justifyContent: 'center', marginTop: '1rem' }}>
            <Button
              variant="contained"
              style={{
                backgroundColor: controlsDisabled ? 'lightgray' : 'green',
                color: controlsDisabled ? 'darkgray' : 'white',
                marginRight: '0.5rem',
                fontSize: '1rem',
                fontWeight: 'bold',
                cursor: controlsDisabled ? 'not-allowed' : 'pointer'
              }}
              onClick={handleRunAll}
              disabled={controlsDisabled} // Disable button when step is in progress
            >
              Run All
            </Button>
            <Button
              variant="contained"
              style={{
                backgroundColor: controlsDisabled ? 'red' : 'lightgray', // Red when enabled, gray when disabled
                color: controlsDisabled ? 'white' : 'darkgray', // White text when enabled, dark gray when disabled
                fontSize: '1rem',
                fontWeight: 'bold',
                cursor: controlsDisabled ? 'pointer' : 'not-allowed', // Pointer when enabled, not-allowed when disabled
              }}
              onClick={handleStop}
              disabled={!controlsDisabled} // Enable Stop only when Step and Run All are disabled
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
