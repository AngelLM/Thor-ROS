import React, { useState, useEffect, useRef } from 'react';
import { useROS } from '../RosContext';
import Button from '@mui/material/Button';
import ButtonGroup from '@mui/material/ButtonGroup';
import * as THREE from 'three';
import useRosApi from '../ros/useRosApi';

export default function IKControls({
  ikPose: ikTargetPose,
  onPreviewJointsChange: onPreviewJointsUpdate,
  onIKStatusChange: onIKReachabilityChange,
  initialPose,
  ghostJoints,
  urdfApi,
  active = true,
}) {
  const { connected } = useROS();
  const rosApi = useRosApi();

  // Main state: TCP position and orientation
  const [tcpPosition, setTcpPosition] = useState(new THREE.Vector3(200, 0, 300)); // in mm
  const [tcpQuaternion, setTcpQuaternion] = useState(new THREE.Quaternion(0, 0, 0, 1)); // identity
  const suppressNextSolveRef = useRef(false);

  // State for cartesian movement
  const [cartesianError, setCartesianError] = useState('');
  const [isComputingCartesian, setIsComputingCartesian] = useState(false);

  // When the IK tab becomes active, sync with the ghost's current TCP
  useEffect(() => {
    if (!active) return;
    if (!urdfApi) return;
    const state = urdfApi.getGhostState && urdfApi.getGhostState();
    if (state && state.tcp) {
      suppressNextSolveRef.current = true; // prevent triggering IK solve from this sync
      setTcpPosition(new THREE.Vector3(state.tcp.x, state.tcp.y, state.tcp.z));
      setTcpQuaternion(new THREE.Quaternion(state.tcp.qx, state.tcp.qy, state.tcp.qz, state.tcp.qw));
    }
  }, [active, urdfApi]);
  
  const [gripperPercent, setGripperPercent] = useState(0);

  // Update gripper percent state
  const updateGripperPercent = (value) => {
    console.warn('Gripper control changed, new percent:', value);
    setGripperPercent(value);
  };
  
  const continuousMoveIntervalRef = useRef(null);
  const ghostJointsRef = useRef(null);

  // keep a ref to the latest ghostJoints so we can update gripper only when user changes it
  useEffect(() => { ghostJointsRef.current = ghostJoints; }, [ghostJoints]);

  // Initialize only when `ikPose` changes
  useEffect(() => {
    if (ikTargetPose) {
      setTcpPosition(new THREE.Vector3(
        Math.round(ikTargetPose.x),
        Math.round(ikTargetPose.y),
        Math.round(ikTargetPose.z)
      ));
      if (
        ikTargetPose.qx !== undefined &&
        ikTargetPose.qy !== undefined &&
        ikTargetPose.qz !== undefined &&
        ikTargetPose.qw !== undefined
      ) {
        setTcpQuaternion(new THREE.Quaternion(
          ikTargetPose.qx,
          ikTargetPose.qy,
          ikTargetPose.qz,
          ikTargetPose.qw
        ));
      }
    }
  }, [ikTargetPose]);

  // Initialize when `initialPose` changes
  useEffect(() => {
    if (initialPose) {
      setTcpPosition(new THREE.Vector3(
        initialPose.x || 0,
        initialPose.y || 0,
        initialPose.z || 0
      ));
      if (
        initialPose.qx !== undefined &&
        initialPose.qy !== undefined &&
        initialPose.qz !== undefined &&
        initialPose.qw !== undefined
      ) {
        setTcpQuaternion(new THREE.Quaternion(
          initialPose.qx,
          initialPose.qy,
          initialPose.qz,
          initialPose.qw
        ));
      }
    }
  }, [initialPose]);

  // IK requests (when position or quaternion change) => delegate to urdfApi
  useEffect(() => {
    if (!active) return; // only when the tab is active
    if (!urdfApi || !connected) return;
    if (suppressNextSolveRef.current) { // avoid re-triggering IK if this is an external sync
      suppressNextSolveRef.current = false;
      return;
    }
    const pose = {
      x: tcpPosition.x,
      y: tcpPosition.y,
      z: tcpPosition.z,
      qx: tcpQuaternion.x,
      qy: tcpQuaternion.y,
      qz: tcpQuaternion.z,
      qw: tcpQuaternion.w,
    };
    urdfApi.solveAndMoveToPose(pose).then((res) => {
      if (res && res.ok) {
        if (onPreviewJointsUpdate) onPreviewJointsUpdate(res.joints);
        if (onIKReachabilityChange) onIKReachabilityChange('reachable');
        // Align the visual target/sphere with the newly solved TCP
        if (urdfApi.syncTargetToTCP) urdfApi.syncTargetToTCP();
      } else {
        if (onPreviewJointsUpdate) onPreviewJointsUpdate({});
        if (onIKReachabilityChange) onIKReachabilityChange('unreachable');
      }
    });
  }, [tcpPosition, tcpQuaternion, urdfApi, connected, active]);

  // Sync the UI with the ghost's TCP when the ghost changes
  useEffect(() => {
    if (!active) return;
    if (!urdfApi || !ghostJoints) return;
    const state = urdfApi.getGhostState && urdfApi.getGhostState();
    if (state && state.tcp) {
      const { x, y, z, qx, qy, qz, qw } = state.tcp;
      // Avoid loop: flag to prevent launching IK in the next effect
      suppressNextSolveRef.current = true;
      setTcpPosition(new THREE.Vector3(x, y, z));
      setTcpQuaternion(new THREE.Quaternion(qx, qy, qz, qw));
    }
  }, [ghostJoints, active, urdfApi]);

  // Publish the ghost to the robot controller
  const publishGhostToController = () => {
    if (!urdfApi) return;
    urdfApi.publishGhostToController();
  };

  // Execute cartesian move to the current TCP position
  const executeCartesianMove = async () => {
    if (!urdfApi || !rosApi.connected) return;
    
    setIsComputingCartesian(true);
    setCartesianError('');

    try {
      // Get current TCP pose from ghost
      const currentState = urdfApi.getGhostState && urdfApi.getGhostState();
      if (!currentState || !currentState.tcp) {
        setCartesianError('Could not get current TCP position');
        setIsComputingCartesian(false);
        return;
      }

      const startPose = {
        x: currentState.tcp.x,
        y: currentState.tcp.y,
        z: currentState.tcp.z,
        qx: currentState.tcp.qx,
        qy: currentState.tcp.qy,
        qz: currentState.tcp.qz,
        qw: currentState.tcp.qw,
      };

      const endPose = {
        x: tcpPosition.x,
        y: tcpPosition.y,
        z: tcpPosition.z,
        qx: tcpQuaternion.x,
        qy: tcpQuaternion.y,
        qz: tcpQuaternion.z,
        qw: tcpQuaternion.w,
      };

      // Compute gripper value from local gripperPercent to avoid race with ghostJoints updates
      let gripperRadians;
      if (gripperPercent === 0) {
        gripperRadians = 0;
      } else if (gripperPercent === 100) {
        gripperRadians = -89.9 * Math.PI / 180;
      } else {
        gripperRadians = -89.9 * (gripperPercent / 100) * Math.PI / 180;
      }

      // Use the locally computed value to ensure we send what the user selected
      const currentGripperValue = gripperRadians;

      // Warn the gripper value (percent and radians) that will be sent with the cartesian goal
      console.warn('MOVE L - gripper percent:', gripperPercent, '-> radians:', currentGripperValue);

      // Send cartesian goal to ROS
      const cartesianResult = await rosApi.publishCartesianGoal(startPose, endPose, {
        gripperValue: currentGripperValue
      });
      
      if (!cartesianResult.ok) {
        if (cartesianResult.error === 'timeout') {
          setCartesianError('Cartesian movement timeout');
        } else {
          setCartesianError(cartesianResult.message || 'No cartesian solution found');
        }
        setIsComputingCartesian(false);
        return;
      }

      // Success
      console.log('Cartesian movement completed successfully');
      console.log('Waypoints:', cartesianResult.waypoints);

    } catch (error) {
      console.error('Cartesian move error:', error);
      setCartesianError('Error during cartesian movement');
    } finally {
      setIsComputingCartesian(false);
    }
  };

  // Apply delta in world/global coordinates
  const applyWorldDelta = (axis, increment) => {
    if (axis === 'x' || axis === 'y' || axis === 'z') {
      // Linear global movement (no transform)
      setTcpPosition(prevPos => {
        const newPos = prevPos.clone();
        newPos[axis] += increment; // mm in global coordinates
        return newPos;
      });
    } else {
      // Rotation: fixed global axes (no transform)
      setTcpQuaternion(prevQuat => {
        let rotAxis;
        if (axis === 'roll') rotAxis = new THREE.Vector3(1, 0, 0);
        if (axis === 'pitch') rotAxis = new THREE.Vector3(0, 1, 0);
        if (axis === 'yaw') rotAxis = new THREE.Vector3(0, 0, 1);

        // Create incremental rotation quaternion in global axes
        const deltaQ = new THREE.Quaternion().setFromAxisAngle(rotAxis, increment * Math.PI / 180);

        // Apply global rotation
        return prevQuat.clone().premultiply(deltaQ);
      });
    }
  };

  // Apply delta in TCP local coordinates
  const applyTcpDelta = (axis, increment) => {
    if (axis === 'x' || axis === 'y' || axis === 'z') {
      // Linear movement: transform delta by current quaternion (local coordinates)
      setTcpPosition(prevPos => {
        const localDelta = new THREE.Vector3();
        localDelta[axis] = increment; // mm
        localDelta.applyQuaternion(tcpQuaternion); // transform to global coordinates
        return prevPos.clone().add(localDelta);
      });
    } else {
      // Rotation: use pure quaternions
      setTcpQuaternion(prevQuat => {
        const q = prevQuat.clone();
        let rotAxis;
        if (axis === 'roll') rotAxis = new THREE.Vector3(1, 0, 0);
        if (axis === 'pitch') rotAxis = new THREE.Vector3(0, 1, 0);
        if (axis === 'yaw') rotAxis = new THREE.Vector3(0, 0, 1);

        // Transform axis to current local coordinates
        rotAxis.applyQuaternion(q);

        // Create incremental rotation quaternion
        const deltaQ = new THREE.Quaternion().setFromAxisAngle(rotAxis, increment * Math.PI / 180);

        // Apply rotation
        return q.premultiply(deltaQ);
      });
    }
  };

  // Start continuous movement in the given axis and direction
  const startContinuousMove = (axis, increment, frameType = 'tcp') => {
    const moveFunction = frameType === 'world' ? applyWorldDelta : applyTcpDelta;
    const intervalId = setInterval(() => moveFunction(axis, increment), 100); // Runs every 100ms
    continuousMoveIntervalRef.current = intervalId;
  };

  // Stop continuous movement
  const stopContinuousMove = () => {
    if (continuousMoveIntervalRef.current) {
      clearInterval(continuousMoveIntervalRef.current);
      continuousMoveIntervalRef.current = null;
    }
  };

  // Sync the gripper position with the ghost's state
  useEffect(() => {
    const currentGhost = ghostJointsRef.current;
    if (!currentGhost || typeof onPreviewJointsUpdate !== 'function') return;
    const gripperValue = gripperPercent === 0
      ? 0
      : gripperPercent === 100
      ? (-89.9 * Math.PI / 180)
      : (-89.9 * (gripperPercent / 100) * Math.PI / 180);
    if (currentGhost['gripperbase_to_armgearright'] !== gripperValue) {
      const updatedJoints = { ...currentGhost, gripperbase_to_armgearright: gripperValue };
      onPreviewJointsUpdate(updatedJoints);
    }
  }, [gripperPercent, onPreviewJointsUpdate]);

  return (
    <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', marginTop: '0', marginBottom: '1rem' }}>
       <h3 style={{ marginTop: '1.5rem', textAlign: 'center', fontSize: '1.5rem' }}>World Frame</h3>
      <div style={{ display: 'flex', justifyContent: 'center', gap: '1rem', marginBottom: '1rem' }}>

        {/* X group */}
        <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', marginRight: '8px' }}>
          <label style={{ marginBottom: '4px' }}><strong>X</strong></label>
          <ButtonGroup>
            <Button
              variant="contained"
                      onMouseDown={() => startContinuousMove('x', -1, 'world')}
                    onMouseUp={stopContinuousMove}
                    onMouseLeave={stopContinuousMove}
              style={{ backgroundColor: '#ffcccc', color: 'black', border: '1px solid #ff9999' }}
            >
              -
            </Button>
            <Button
              variant="contained"
                onMouseDown={() => startContinuousMove('x', 1, 'world')}
              onMouseUp={stopContinuousMove}
              onMouseLeave={stopContinuousMove}
              style={{ backgroundColor: '#ffcccc', color: 'black', border: '1px solid #ff9999' }}
            >
              +
            </Button>
          </ButtonGroup>
        </div>

        {/* Y group */}
        <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', marginRight: '8px' }}>
          <label style={{ marginBottom: '4px' }}><strong>Y</strong></label>
          <ButtonGroup>
            <Button
              variant="contained"
              onMouseDown={() => startContinuousMove('y', -1, 'world')}
              onMouseUp={stopContinuousMove}
              onMouseLeave={stopContinuousMove}
              style={{ backgroundColor: '#ffffcc', color: 'black', border: '1px solid #cccc99' }}
            >
              -
            </Button>
            <Button
              variant="contained"
              onMouseDown={() => startContinuousMove('y', 1, 'world')}
              onMouseUp={stopContinuousMove}
              onMouseLeave={stopContinuousMove}
              style={{ backgroundColor: '#ffffcc', color: 'black', border: '1px solid #cccc99' }}
            >
              +
            </Button>
          </ButtonGroup>
        </div>

        {/* Z group */}
        <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
          <label style={{ marginBottom: '4px' }}><strong>Z</strong></label>
          <ButtonGroup>
            <Button
              variant="contained"
              onMouseDown={() => startContinuousMove('z', -1, 'world')}
              onMouseUp={stopContinuousMove}
              onMouseLeave={stopContinuousMove}
              style={{ backgroundColor: '#ccccff', color: 'black', border: '1px solid #9999cc' }}
            >
              -
            </Button>
            <Button
              variant="contained"
              onMouseDown={() => startContinuousMove('z', 1, 'world')}
              onMouseUp={stopContinuousMove}
              onMouseLeave={stopContinuousMove}
              style={{ backgroundColor: '#ccccff', color: 'black', border: '1px solid #9999cc' }}
            >
              +
            </Button>
          </ButtonGroup>
        </div>
      </div>

      {/* Buttons for TCP coordinate adjustments */}
      <div style={{ display: 'flex', justifyContent: 'center', gap: '1rem', marginBottom: '1rem' }}> {/* Row for Roll, Pitch, Yaw Buttons */}
        <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', marginRight: '8px' }}> {/* Roll group */}
          <label style={{ marginBottom: '4px' }}><strong>Roll</strong></label>
          <ButtonGroup>
            <Button
              variant="contained"
              onMouseDown={() => startContinuousMove('roll', -1, 'world')}
              onMouseUp={stopContinuousMove}
              onMouseLeave={stopContinuousMove}
              style={{ backgroundColor: '#ffcccc', color: 'black', border: '1px solid #ff9999' }}
            >
              -
            </Button>
            <Button
              variant="contained"
                onMouseDown={() => startContinuousMove('roll', 1, 'world')}
              onMouseUp={stopContinuousMove}
              onMouseLeave={stopContinuousMove}
              style={{ backgroundColor: '#ffcccc', color: 'black', border: '1px solid #ff9999' }}
            >
              +
            </Button>
          </ButtonGroup>
        </div>
        <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', marginRight: '8px' }}> {/* Pitch group */}
          <label style={{ marginBottom: '4px' }}><strong>Pitch</strong></label>
          <ButtonGroup>
            <Button
              variant="contained"
              onMouseDown={() => startContinuousMove('pitch', -1, 'world')}
              onMouseUp={stopContinuousMove}
              onMouseLeave={stopContinuousMove}
              style={{ backgroundColor: '#ffffcc', color: 'black', border: '1px solid #cccc99' }}
            >
              -
            </Button>
            <Button
              variant="contained"
              onMouseDown={() => startContinuousMove('pitch', 1, 'world')}
              onMouseUp={stopContinuousMove}
              onMouseLeave={stopContinuousMove}
              style={{ backgroundColor: '#ffffcc', color: 'black', border: '1px solid #cccc99' }}
            >
              +
            </Button>
          </ButtonGroup>
        </div>
        <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}> {/* Yaw group */}
          <label style={{ marginBottom: '4px' }}><strong>Yaw</strong></label>
          <ButtonGroup>
            <Button
              variant="contained"
              onMouseDown={() => startContinuousMove('yaw', -1, 'world')}
              onMouseUp={stopContinuousMove}
              onMouseLeave={stopContinuousMove}
              style={{ backgroundColor: '#ccccff', color: 'black', border: '1px solid #9999cc' }}
            >
              -
            </Button>
            <Button
              variant="contained"
              onMouseDown={() => startContinuousMove('yaw', 1, 'world')}
              onMouseUp={stopContinuousMove}
              onMouseLeave={stopContinuousMove}
              style={{ backgroundColor: '#ccccff', color: 'black', border: '1px solid #9999cc' }}
            >
              +
            </Button>
          </ButtonGroup>
        </div>
      </div>

      {/* Buttons for TCP coordinate adjustments */}        
      <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', marginTop: '1rem' }}> {/* TCP Buttons */}
        <h3 style={{ marginTop: '1.5rem', textAlign: 'center', fontSize: '1.5rem' }}>TCP Frame</h3>
        <div style={{ display: 'flex', justifyContent: 'center', gap: '1rem', marginBottom: '1rem' }}> {/* Row for X, Y, Z */}
          <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', marginRight: '8px' }}> {/* X group */}
            <label style={{ marginBottom: '4px' }}><strong>X</strong></label>
            <ButtonGroup>
              <Button
                variant="contained"
                onMouseDown={() => startContinuousMove('x', -1, 'tcp')}
                onMouseUp={stopContinuousMove}
                onMouseLeave={stopContinuousMove} // Ensure it stops if the mouse leaves the button
                style={{ backgroundColor: '#ffcccc', color: 'black', border: '1px solid #ff9999' }}
              >
                -
              </Button>
              <Button
                variant="contained"
                onMouseDown={() => startContinuousMove('x', 1, 'tcp')}
                onMouseUp={stopContinuousMove}
                onMouseLeave={stopContinuousMove}
                style={{ backgroundColor: '#ffcccc', color: 'black', border: '1px solid #ff9999' }}
              >
                +
              </Button>
            </ButtonGroup>
          </div>
          <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', marginRight: '8px' }}> {/* Y group */}
            <label style={{ marginBottom: '4px' }}><strong>Y</strong></label>
            <ButtonGroup>
              <Button
                variant="contained"
                onMouseDown={() => startContinuousMove('y', -1, 'tcp')}
                onMouseUp={stopContinuousMove}
                onMouseLeave={stopContinuousMove}
                style={{ backgroundColor: '#ffffcc', color: 'black', border: '1px solid #cccc99' }}
              >
                -
              </Button>
              <Button
                variant="contained"
                onMouseDown={() => startContinuousMove('y', 1, 'tcp')}
                onMouseUp={stopContinuousMove}
                onMouseLeave={stopContinuousMove}
                style={{ backgroundColor: '#ffffcc', color: 'black', border: '1px solid #cccc99' }}
              >
                +
              </Button>
            </ButtonGroup>
          </div>
          <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}> {/* Z group */}
            <label style={{ marginBottom: '4px' }}><strong>Z</strong></label>
            <ButtonGroup>
              <Button
                variant="contained"
                onMouseDown={() => startContinuousMove('z', -1, 'tcp')}
                onMouseUp={stopContinuousMove}
                onMouseLeave={stopContinuousMove}
                style={{ backgroundColor: '#ccccff', color: 'black', border: '1px solid #9999cc' }}
              >
                -
              </Button>
              <Button
                variant="contained"
                onMouseDown={() => startContinuousMove('z', 1, 'tcp')}
                onMouseUp={stopContinuousMove}
                onMouseLeave={stopContinuousMove}
                style={{ backgroundColor: '#ccccff', color: 'black', border: '1px solid #9999cc' }}
              >
                +
              </Button>
            </ButtonGroup>
          </div>
        </div>
        <div style={{ display: 'flex', justifyContent: 'center', gap: '1rem', marginBottom: '1rem' }}> {/* Row for Roll, Pitch, Yaw */}
          <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', marginRight: '8px' }}> {/* Roll group */}
            <label style={{ marginBottom: '4px' }}><strong>Roll</strong></label>
            <ButtonGroup>
              <Button
                variant="contained"
                onMouseDown={() => startContinuousMove('roll', -1, 'tcp')}
                onMouseUp={stopContinuousMove}
                onMouseLeave={stopContinuousMove}
                style={{ backgroundColor: '#ffcccc', color: 'black', border: '1px solid #ff9999' }}
              >
                -
              </Button>
              <Button
                variant="contained"
                onMouseDown={() => startContinuousMove('roll', 1, 'tcp')}
                onMouseUp={stopContinuousMove}
                onMouseLeave={stopContinuousMove}
                style={{ backgroundColor: '#ffcccc', color: 'black', border: '1px solid #ff9999' }}
              >
                +
              </Button>
            </ButtonGroup>
          </div>
          <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', marginRight: '8px' }}> {/* Pitch group */}
            <label style={{ marginBottom: '4px' }}><strong>Pitch</strong></label>
            <ButtonGroup>
              <Button
                variant="contained"
                onMouseDown={() => startContinuousMove('pitch', -1, 'tcp')}
                onMouseUp={stopContinuousMove}
                onMouseLeave={stopContinuousMove}
                style={{ backgroundColor: '#ffffcc', color: 'black', border: '1px solid #cccc99' }}
              >
                -
              </Button>
              <Button
                variant="contained"
                onMouseDown={() => startContinuousMove('pitch', 1, 'tcp')}
                onMouseUp={stopContinuousMove}
                onMouseLeave={stopContinuousMove}
                style={{ backgroundColor: '#ffffcc', color: 'black', border: '1px solid #cccc99' }}
              >
                +
              </Button>
            </ButtonGroup>
          </div>
          <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}> {/* Yaw group */}
            <label style={{ marginBottom: '4px' }}><strong>Yaw</strong></label>
            <ButtonGroup>
              <Button
                variant="contained"
                onMouseDown={() => startContinuousMove('yaw', -1, 'tcp')}
                onMouseUp={stopContinuousMove}
                onMouseLeave={stopContinuousMove}
                style={{ backgroundColor: '#ccccff', color: 'black', border: '1px solid #9999cc' }}
              >
                -
              </Button>
              <Button
                variant="contained"
                onMouseDown={() => startContinuousMove('yaw', 1, 'tcp')}
                onMouseUp={stopContinuousMove}
                onMouseLeave={stopContinuousMove}
                style={{ backgroundColor: '#ccccff', color: 'black', border: '1px solid #9999cc' }}
              >
                +
              </Button>
            </ButtonGroup>
          </div>
        </div>
      </div>

      {/* Gripper control buttons */}
      <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', marginTop: '1rem' }}> {/* Gripper Buttons */}
        <h3 style={{ marginTop: '1.5rem', textAlign: 'center', fontSize: '1.5rem' }}>Gripper Control</h3>
        <ButtonGroup>
          {[0, 20, 40, 60, 80, 100].map((percentage) => (
            <Button
              key={percentage}
              variant={gripperPercent === percentage ? "contained" : "outlined"} // Highlight selected button
              onClick={() => updateGripperPercent(percentage)} // Update selected state
              style={{ backgroundColor: gripperPercent === percentage ? '#ccccff' : 'white', color: 'black', border: '1px solid #9999cc' }}
            >
              {percentage}%
            </Button>
          ))}
        </ButtonGroup>
      </div>

      <div style={{ display: 'flex', justifyContent: 'center', gap: '0.5rem', marginTop: '1rem' }}>
        <Button
          onClick={publishGhostToController}
          disabled={!ghostJoints || Object.keys(ghostJoints).length === 0}
          variant="contained"
          color={(!ghostJoints || Object.keys(ghostJoints).length === 0) ? "secondary" : "primary"}
          style={{ fontWeight: 'bold' }}
        >
          MOVE J
        </Button>

        <Button
          onClick={executeCartesianMove}
          disabled={!ghostJoints || Object.keys(ghostJoints).length === 0 || !connected || isComputingCartesian}
          variant="contained"
          color={(!ghostJoints || Object.keys(ghostJoints).length === 0 || !connected) ? "secondary" : "success"}
          style={{ fontWeight: 'bold' }}
        >
          {isComputingCartesian ? 'COMPUTING...' : 'MOVE L'}
        </Button>
      </div>

      {cartesianError && (
        <div style={{ color: 'red', marginTop: '0.5rem', fontSize: '0.9rem', textAlign: 'center' }}>
          {cartesianError}
        </div>
      )}

    </div>
  );
}