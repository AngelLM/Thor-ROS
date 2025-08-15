import React, { useState, useEffect, useRef } from 'react';
import { useROS } from '../RosContext';
import Button from '@mui/material/Button';
import ButtonGroup from '@mui/material/ButtonGroup';
import * as THREE from 'three';

export default function InverseKinematicsControls({ ikPose, onPreviewJointsChange, onIKStatusChange, initialPose, ghostJoints, urdfApi, active = true }) {
  const { connected } = useROS();
  
  // Main state: position and base quaternion (source of truth)
  const [position, setPosition] = useState(new THREE.Vector3(200, 0, 300)); // in mm
  const [baseQuaternion, setBaseQuaternion] = useState(new THREE.Quaternion(0, 0, 0, 1)); // identity
  const suppressSolveRef = useRef(false);

  // When the IK tab becomes active, sync with the ghost's current TCP
  useEffect(() => {
    if (!active) return;
    if (!urdfApi) return;
    const state = urdfApi.getGhostState && urdfApi.getGhostState();
    if (state && state.tcp) {
      suppressSolveRef.current = true; // prevent triggering IK solve from this sync
      setPosition(new THREE.Vector3(state.tcp.x, state.tcp.y, state.tcp.z));
      setBaseQuaternion(new THREE.Quaternion(state.tcp.qx, state.tcp.qy, state.tcp.qz, state.tcp.qw));
    }
  }, [active, urdfApi]);
  
  const [selectedGripper, setSelectedGripper] = useState(0);
  const [activeInterval, setActiveInterval] = useState(null);
  // keep a ref to the latest ghostJoints so we can update gripper only when user changes it
  const latestGhostJointsRef = useRef(null);
  useEffect(() => { latestGhostJointsRef.current = ghostJoints; }, [ghostJoints]);

  // Initialize only when `ikPose` changes
  useEffect(() => {
    if (ikPose) {
      setPosition(new THREE.Vector3(
        Math.round(ikPose.x),
        Math.round(ikPose.y),
        Math.round(ikPose.z)
      ));
      if (
        ikPose.qx !== undefined &&
        ikPose.qy !== undefined &&
        ikPose.qz !== undefined &&
        ikPose.qw !== undefined
      ) {
        setBaseQuaternion(new THREE.Quaternion(
          ikPose.qx,
          ikPose.qy,
          ikPose.qz,
          ikPose.qw
        ));
      }
    }
  }, [ikPose]);

  // Initialize when `initialPose` changes
  useEffect(() => {
    if (initialPose) {
      setPosition(new THREE.Vector3(
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
        setBaseQuaternion(new THREE.Quaternion(
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
    if (suppressSolveRef.current) { // avoid re-triggering IK if this is an external sync
      suppressSolveRef.current = false;
      return;
    }
    const pose = {
      x: position.x,
      y: position.y,
      z: position.z,
      qx: baseQuaternion.x,
      qy: baseQuaternion.y,
      qz: baseQuaternion.z,
      qw: baseQuaternion.w,
    };
    urdfApi.solveAndMoveToPose(pose).then((res) => {
      if (res && res.ok) {
        if (onPreviewJointsChange) onPreviewJointsChange(res.joints);
        if (onIKStatusChange) onIKStatusChange('reachable');
        // Align the visual target/sphere with the newly solved TCP
        if (urdfApi.syncTargetToTCP) urdfApi.syncTargetToTCP();
      } else {
        if (onPreviewJointsChange) onPreviewJointsChange({});
        if (onIKStatusChange) onIKStatusChange('unreachable');
      }
    });
  }, [position, baseQuaternion, urdfApi, connected, active]);

  // Sync the UI with the ghost's TCP when the ghost changes
  useEffect(() => {
    if (!active) return;
    if (!urdfApi || !ghostJoints) return;
    const state = urdfApi.getGhostState && urdfApi.getGhostState();
    if (state && state.tcp) {
      const { x, y, z, qx, qy, qz, qw } = state.tcp;
      // Avoid loop: flag to prevent launching IK in the next effect
      suppressSolveRef.current = true;
      setPosition(new THREE.Vector3(x, y, z));
      setBaseQuaternion(new THREE.Quaternion(qx, qy, qz, qw));
    }
  }, [ghostJoints, active, urdfApi]);

  const sendJointCommand = () => {
    if (!urdfApi) return;
    urdfApi.publishGhostToController();
  };

  const handleWorldMove = (axis, increment) => {
    if (axis === 'x' || axis === 'y' || axis === 'z') {
      // Linear global movement (no transform)
      setPosition(prevPos => {
        const newPos = prevPos.clone();
        newPos[axis] += increment; // mm in global coordinates
        return newPos;
      });
    } else {
      // Rotation: fixed global axes (no transform)
      setBaseQuaternion(prevQuat => {
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

  const handleTCPMove = (axis, increment) => {
    if (axis === 'x' || axis === 'y' || axis === 'z') {
      // Linear movement: transform delta by current quaternion (local coordinates)
      setPosition(prevPos => {
        const localDelta = new THREE.Vector3();
        localDelta[axis] = increment; // mm
        localDelta.applyQuaternion(baseQuaternion); // transformar a coordenadas globales
        return prevPos.clone().add(localDelta);
      });
    } else {
      // Rotation: use logic from GizmoPage (pure quaternion)
      setBaseQuaternion(prevQuat => {
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

  const handleMouseDown = (axis, increment, frameType = 'tcp') => {
    const moveFunction = frameType === 'world' ? handleWorldMove : handleTCPMove;
    const intervalId = setInterval(() => moveFunction(axis, increment), 100); // Runs every 100ms
    setActiveInterval(intervalId);
  };

  const handleMouseUp = () => {
    if (activeInterval) {
      clearInterval(activeInterval);
      setActiveInterval(null);
    }
  };

  // Sync the gripper position with the ghost's state
  useEffect(() => {
    const currentGhost = latestGhostJointsRef.current;
    if (!currentGhost || typeof onPreviewJointsChange !== 'function') return;
    const gripperValue = selectedGripper === 0
      ? 0
      : selectedGripper === 100
      ? (-89.9 * Math.PI / 180)
      : (-89.9 * (selectedGripper / 100) * Math.PI / 180);
    if (currentGhost['gripperbase_to_armgearright'] !== gripperValue) {
      const updatedJoints = { ...currentGhost, gripperbase_to_armgearright: gripperValue };
      onPreviewJointsChange(updatedJoints);
    }
  }, [selectedGripper, onPreviewJointsChange]);

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
              onMouseDown={() => handleMouseDown('x', -1, 'world')}
              onMouseUp={handleMouseUp}
              onMouseLeave={handleMouseUp}
              style={{ backgroundColor: '#ffcccc', color: 'black', border: '1px solid #ff9999' }}
            >
              -
            </Button>
            <Button
              variant="contained"
              onMouseDown={() => handleMouseDown('x', 1, 'world')}
              onMouseUp={handleMouseUp}
              onMouseLeave={handleMouseUp}
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
              onMouseDown={() => handleMouseDown('y', -1, 'world')}
              onMouseUp={handleMouseUp}
              onMouseLeave={handleMouseUp}
              style={{ backgroundColor: '#ffffcc', color: 'black', border: '1px solid #cccc99' }}
            >
              -
            </Button>
            <Button
              variant="contained"
              onMouseDown={() => handleMouseDown('y', 1, 'world')}
              onMouseUp={handleMouseUp}
              onMouseLeave={handleMouseUp}
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
              onMouseDown={() => handleMouseDown('z', -1, 'world')}
              onMouseUp={handleMouseUp}
              onMouseLeave={handleMouseUp}
              style={{ backgroundColor: '#ccccff', color: 'black', border: '1px solid #9999cc' }}
            >
              -
            </Button>
            <Button
              variant="contained"
              onMouseDown={() => handleMouseDown('z', 1, 'world')}
              onMouseUp={handleMouseUp}
              onMouseLeave={handleMouseUp}
              style={{ backgroundColor: '#ccccff', color: 'black', border: '1px solid #9999cc' }}
            >
              +
            </Button>
          </ButtonGroup>
        </div>
      </div>
      <div style={{ display: 'flex', justifyContent: 'center', gap: '1rem', marginBottom: '1rem' }}> {/* Row for Roll, Pitch, Yaw Buttons */}
        <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', marginRight: '8px' }}> {/* Roll group */}
          <label style={{ marginBottom: '4px' }}><strong>Roll</strong></label>
          <ButtonGroup>
            <Button
              variant="contained"
              onMouseDown={() => handleMouseDown('roll', -1, 'world')}
              onMouseUp={handleMouseUp}
              onMouseLeave={handleMouseUp}
              style={{ backgroundColor: '#ffcccc', color: 'black', border: '1px solid #ff9999' }}
            >
              -
            </Button>
            <Button
              variant="contained"
              onMouseDown={() => handleMouseDown('roll', 1, 'world')}
              onMouseUp={handleMouseUp}
              onMouseLeave={handleMouseUp}
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
              onMouseDown={() => handleMouseDown('pitch', -1, 'world')}
              onMouseUp={handleMouseUp}
              onMouseLeave={handleMouseUp}
              style={{ backgroundColor: '#ffffcc', color: 'black', border: '1px solid #cccc99' }}
            >
              -
            </Button>
            <Button
              variant="contained"
              onMouseDown={() => handleMouseDown('pitch', 1, 'world')}
              onMouseUp={handleMouseUp}
              onMouseLeave={handleMouseUp}
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
              onMouseDown={() => handleMouseDown('yaw', -1, 'world')}
              onMouseUp={handleMouseUp}
              onMouseLeave={handleMouseUp}
              style={{ backgroundColor: '#ccccff', color: 'black', border: '1px solid #9999cc' }}
            >
              -
            </Button>
            <Button
              variant="contained"
              onMouseDown={() => handleMouseDown('yaw', 1, 'world')}
              onMouseUp={handleMouseUp}
              onMouseLeave={handleMouseUp}
              style={{ backgroundColor: '#ccccff', color: 'black', border: '1px solid #9999cc' }}
            >
              +
            </Button>
          </ButtonGroup>
        </div>
      </div>
      
      {/* Added buttons for TCP coordinate adjustments */}
      <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', marginTop: '1rem' }}> {/* TCP Buttons */}
        <h3 style={{ marginTop: '1.5rem', textAlign: 'center', fontSize: '1.5rem' }}>TCP Frame</h3>
        <div style={{ display: 'flex', justifyContent: 'center', gap: '1rem', marginBottom: '1rem' }}> {/* Row for X, Y, Z */}
          <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', marginRight: '8px' }}> {/* X group */}
            <label style={{ marginBottom: '4px' }}><strong>X</strong></label>
            <ButtonGroup>
              <Button
                variant="contained"
                onMouseDown={() => handleMouseDown('x', -1, 'tcp')}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp} // Ensure it stops if the mouse leaves the button
                style={{ backgroundColor: '#ffcccc', color: 'black', border: '1px solid #ff9999' }}
              >
                -
              </Button>
              <Button
                variant="contained"
                onMouseDown={() => handleMouseDown('x', 1, 'tcp')}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
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
                onMouseDown={() => handleMouseDown('y', -1, 'tcp')}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
                style={{ backgroundColor: '#ffffcc', color: 'black', border: '1px solid #cccc99' }}
              >
                -
              </Button>
              <Button
                variant="contained"
                onMouseDown={() => handleMouseDown('y', 1, 'tcp')}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
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
                onMouseDown={() => handleMouseDown('z', -1, 'tcp')}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
                style={{ backgroundColor: '#ccccff', color: 'black', border: '1px solid #9999cc' }}
              >
                -
              </Button>
              <Button
                variant="contained"
                onMouseDown={() => handleMouseDown('z', 1, 'tcp')}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
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
                onMouseDown={() => handleMouseDown('roll', -1, 'tcp')}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
                style={{ backgroundColor: '#ffcccc', color: 'black', border: '1px solid #ff9999' }}
              >
                -
              </Button>
              <Button
                variant="contained"
                onMouseDown={() => handleMouseDown('roll', 1, 'tcp')}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
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
                onMouseDown={() => handleMouseDown('pitch', -1, 'tcp')}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
                style={{ backgroundColor: '#ffffcc', color: 'black', border: '1px solid #cccc99' }}
              >
                -
              </Button>
              <Button
                variant="contained"
                onMouseDown={() => handleMouseDown('pitch', 1, 'tcp')}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
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
                onMouseDown={() => handleMouseDown('yaw', -1, 'tcp')}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
                style={{ backgroundColor: '#ccccff', color: 'black', border: '1px solid #9999cc' }}
              >
                -
              </Button>
              <Button
                variant="contained"
                onMouseDown={() => handleMouseDown('yaw', 1, 'tcp')}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
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
              variant={selectedGripper === percentage ? "contained" : "outlined"} // Highlight selected button
              onClick={() => setSelectedGripper(percentage)} // Update selected state
              style={{ backgroundColor: selectedGripper === percentage ? '#ccccff' : 'white', color: 'black', border: '1px solid #9999cc' }}
            >
              {percentage}%
            </Button>
          ))}
        </ButtonGroup>
      </div>

      <Button
        onClick={sendJointCommand}
        disabled={!ghostJoints || Object.keys(ghostJoints).length === 0}
        variant="contained"
        color={(!ghostJoints || Object.keys(ghostJoints).length === 0) ? "secondary" : "primary"}
        style={{ marginTop: '1rem', fontWeight: 'bold' }}
      >
        Move
      </Button>

    </div>
  );
}