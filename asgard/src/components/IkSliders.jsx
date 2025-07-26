import React, { useState, useEffect } from 'react';
import { useROS } from '../RosContext';
import ROSLIB from 'roslib';
import RadioGroup from '@mui/material/RadioGroup';
import FormControlLabel from '@mui/material/FormControlLabel';
import Radio from '@mui/material/Radio';
import FormLabel from '@mui/material/FormLabel';
import { styled } from '@mui/system';
import Button from '@mui/material/Button';
import ButtonGroup from '@mui/material/ButtonGroup';
import * as THREE from 'three';

// Utilidad para convertir RPY a quaternion
function rpyToQuaternion(roll, pitch, yaw) {
  const cy = Math.cos(yaw * 0.5);
  const sy = Math.sin(yaw * 0.5);
  const cp = Math.cos(pitch * 0.5);
  const sp = Math.sin(pitch * 0.5);
  const cr = Math.cos(roll * 0.5);
  const sr = Math.sin(roll * 0.5);

  return {
    x: sr * cp * cy - cr * sp * sy,
    y: cr * sp * cy + sr * cp * sy,
    z: cr * cp * sy - sr * sp * cy,
    w: cr * cp * cy + sr * sp * sy
  };
}

const blue = {
  100: '#daecff',
  200: '#b6daff',
  300: '#66b2ff',
  400: '#3399ff',
  500: '#007fff',
  600: '#0072e5',
  700: '#0059B2',
  800: '#004c99',
};

const grey = {
  50: '#F3F6F9',
  100: '#E5EAF2',
  200: '#DAE2ED',
  300: '#C7D0DD',
  400: '#B0B8C4',
  500: '#9DA8B7',
  600: '#6B7A90',
  700: '#434D5B',
  800: '#303740',
  900: '#1C2025',
};

const StyledInput = styled('input')(
  ({ theme, axis }) => `
  font-size: 0.875rem;
  font-family: inherit;
  font-weight: 400;
  line-height: 1.375;
  color: ${theme.palette.mode === 'dark' ? grey[300] : grey[900]};
  background: ${axis === 'x' ? '#ffcccc' : axis === 'y' ? '#ffffcc' : axis === 'z' ? '#ccccff' : theme.palette.mode === 'dark' ? grey[900] : '#fff'};
  border: 1px solid ${theme.palette.mode === 'dark' ? grey[700] : grey[200]};
  box-shadow: 0 2px 4px ${
    theme.palette.mode === 'dark' ? 'rgba(0,0,0, 0.5)' : 'rgba(0,0,0, 0.05)'
  };
  border-radius: 8px;
  margin: 0 8px;
  padding: 10px 12px;
  outline: 0;
  min-width: 0;
  width: 4rem;
  text-align: center;

  &:hover {
    border-color: ${blue[400]};
  }

  &:focus {
    border-color: ${blue[400]};
    box-shadow: 0 0 0 3px ${theme.palette.mode === 'dark' ? blue[700] : blue[200]};
  }

  &:focus-visible {
    outline: 0;
  }
`,
);

const NumberInput = React.forwardRef(function CustomNumberInput(props, ref) {
  return (
    <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center' }}>
      <label style={{ marginBottom: '4px' }}>{props.label}</label>
      <StyledInput
        type="number"
        value={props.value.toFixed(1)} // Ensure displayed value is limited to one decimal place
        onChange={(e) => {
          const newValue = parseFloat(e.target.value);
          if (!isNaN(newValue)) {
            props.onChange(Math.min(Math.max(newValue, props.min), props.max));
          }
        }}
        min={props.min}
        max={props.max}
        step={props.step} // Use step directly
        ref={ref}
        axis={props.axis} // Pass axis prop for background color
        style={{ textAlign: 'center', width: '4rem', margin: '0 8px' }}
      />
    </div>
  );
});

export default function IKSliders({ ikPose, onPreviewJointsChange, onIKStatusChange, initialPose, ghostJoints }) {
  const { ros, connected } = useROS();
  const [values, setValues] = useState({
    x: 200, y: 0, z: 300, roll: 0, pitch: 0, yaw: 0
  });
  const [ikConfig, setIkConfig] = useState({
    elbow: 'any',
    shoulder: 'any',
    wrist: 'any' // Default values set to 'any'
  });
  const [statusMsg, setStatusMsg] = useState(null);

  // Estado para saber si la posición es alcanzable
  const isUnreachable = statusMsg && (statusMsg.status === 'unreachable' || statusMsg.status === 'error');

  // Inicializa los sliders SOLO cuando cambia ikPose (sin lógica extra)
  useEffect(() => {
    if (ikPose) {
      setValues({
        x: Math.round(ikPose.x),
        y: Math.round(ikPose.y),
        z: Math.round(ikPose.z),
        roll: (ikPose.roll * 180 / Math.PI),
        pitch: (ikPose.pitch * 180 / Math.PI),
        yaw: (ikPose.yaw * 180 / Math.PI)
      });
    }
  }, [ikPose]);

  // Asegura que los inputs se actualicen cuando cambia initialPose
  useEffect(() => {
    if (initialPose) {
      setValues({
        x: initialPose.x || 0,
        y: initialPose.y || 0,
        z: initialPose.z || 0,
        roll: initialPose.roll || 0,
        pitch: initialPose.pitch || 0,
        yaw: initialPose.yaw || 0,
      });
    }
  }, [initialPose]);

  // Suscripción al status del IK
  useEffect(() => {
    if (!ros || !connected) return;
    const statusTopic = new ROSLIB.Topic({
      ros,
      name: '/ik_goal_status',
      messageType: 'std_msgs/String'
    });
    const cb = (msg) => {
      try {
        const data = JSON.parse(msg.data);
        setStatusMsg(data);
        if (onIKStatusChange && data.status) {
          onIKStatusChange(data.status);
        }
      } catch {
        setStatusMsg({ status: "error", detail: "Error interpretando el mensaje de estado" });
        if (onIKStatusChange) onIKStatusChange('error');
      }
    };
    statusTopic.subscribe(cb);
    return () => statusTopic.unsubscribe(cb);
  }, [ros, connected, onIKStatusChange]);

  useEffect(() => {
    if (onPreviewJointsChange) {
      onPreviewJointsChange({
        x: values.x,
        y: values.y,
        z: values.z,
        roll: values.roll * Math.PI / 180,
        pitch: values.pitch * Math.PI / 180,
        yaw: values.yaw * Math.PI / 180
      });
    }
  }, [values, onPreviewJointsChange]);

  // Peticiones IK solo cuando cambian los values (NumberInputs)
  useEffect(() => {
    if (!ros || !connected) return;
    console.log('Sending IK request with values:', values);
    
    const service = new ROSLIB.Service({
      ros,
      name: '/compute_ik',
      serviceType: 'moveit_msgs/srv/GetPositionIK'
    });
    
    const pose = {
      header: { frame_id: 'base_link' },
      pose: {
        position: {
          x: values.x / 1000,
          y: values.y / 1000,
          z: values.z / 1000
        },
        orientation: rpyToQuaternion(
          values.roll * Math.PI / 180,
          values.pitch * Math.PI / 180,
          values.yaw * Math.PI / 180
        )
      }
    };

    // Construir robot_state con ghostJoints si están disponibles
    let robotState = {};
    if (ghostJoints && Object.keys(ghostJoints).length > 0) {
      // Filtrar solo las joints principales y del gripper que necesitamos
      const jointNames = [
        'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6',
        'gripperbase_to_armgearright'
      ];
      
      const filteredNames = [];
      const filteredPositions = [];
      
      jointNames.forEach(jointName => {
        if (ghostJoints[jointName] !== undefined) {
          filteredNames.push(jointName);
          filteredPositions.push(ghostJoints[jointName]);
        }
      });
      
      if (filteredNames.length > 0) {
        robotState = {
          joint_state: {
            name: filteredNames,
            position: filteredPositions
          }
        };
      }
    }
    
    const req = {
      ik_request: {
        group_name: 'arm_group',
        pose_stamped: pose,
        ik_link_name: 'gripper_base',
        timeout: { sec: 0, nanosec: 0 },
        constraints: {},
        robot_state: robotState,
        avoid_collisions: false
      }
    };
    service.callService(req, (res) => {
      console.log('Received IK response:', res);
      if (res && res.solution && res.solution.joint_state && res.error_code && res.error_code.val === 1) {
        const joints = {};
        res.solution.joint_state.name.forEach((name, i) => {
          joints[name] = res.solution.joint_state.position[i];
        });
        if (onPreviewJointsChange) {
          onPreviewJointsChange(joints);
        }
        setStatusMsg({ status: 'reachable' });
      } else {
        console.warn('IK response indicates unreachable or error:', res);
        if (onPreviewJointsChange) {
          onPreviewJointsChange({});
        }
        setStatusMsg({ status: 'unreachable' });
      }
    });
  }, [values, ros, connected, onPreviewJointsChange]); // Solo cuando cambian values, NO ghostJoints

  const handleValueChange = (key, newValue) => {
    setValues((prevValues) => ({ ...prevValues, [key]: newValue }));
  };

  const handleConfigChange = (name, value) => {
    setIkConfig(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const sendIKGoal = () => {
    if (!connected || !ros) {
      console.warn('ROS no está conectado.');
      return;
    }
    const pose = {
      position: {
        x: values.x / 1000,
        y: values.y / 1000,
        z: values.z / 1000
      },
      orientation: rpyToQuaternion(
        values.roll * Math.PI / 180,
        values.pitch * Math.PI / 180,
        values.yaw * Math.PI / 180
      )
    };

    const topic = new ROSLIB.Topic({
      ros,
      name: '/ik_goal',
      messageType: 'thor_msgs/IKGoal'
    });

    topic.publish({
      pose: pose,
      preferred_joints: [], // vacío, ya no se usan
      elbow_config: ikConfig.elbow,
      shoulder_config: ikConfig.shoulder,
      wrist_config: ikConfig.wrist
    });
  };

const handleTCPMove = (axis, increment) => {
  console.log(`Moving TCP for axis: ${axis}, increment: ${increment}`);

  setValues((prevValues) => {
    // 1. Delta pose
    const deltaPose = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
    if (axis === 'x' || axis === 'y' || axis === 'z') {
      deltaPose[axis] = increment / 1000; // mm -> m
    } else {
      deltaPose[axis] = increment * Math.PI / 180; // deg -> rad
    }

    // 2. Pose actual del TCP respecto al Origen
    const currentPose = {
      x: prevValues.x * 0.001,
      y: prevValues.y * 0.001,
      z: prevValues.z * 0.001,
      roll: prevValues.roll * Math.PI / 180,
      pitch: prevValues.pitch * Math.PI / 180,
      yaw: prevValues.yaw * Math.PI / 180
    };

    // 3. Matriz actual
    const currentPosition = new THREE.Vector3(currentPose.x, currentPose.y, currentPose.z);
    const currentEuler = new THREE.Euler(currentPose.roll, currentPose.pitch, currentPose.yaw, 'XYZ');
    const currentMatrix = new THREE.Matrix4();
    currentMatrix.makeRotationFromEuler(currentEuler);
    currentMatrix.setPosition(currentPosition);

    // 4. Matriz de incremento en el sistema del TCP
    const deltaPosition = new THREE.Vector3(deltaPose.x, deltaPose.y, deltaPose.z);
    const deltaEuler = new THREE.Euler(deltaPose.roll, deltaPose.pitch, deltaPose.yaw, 'XYZ');
    const deltaMatrix = new THREE.Matrix4();
    deltaMatrix.makeRotationFromEuler(deltaEuler);
    deltaMatrix.setPosition(deltaPosition);

    // 5. Multiplicación: nueva pose = current * delta (incremento en sistema TCP)
    const newMatrix = currentMatrix.clone().multiply(deltaMatrix);

    // 6. Descomposición de la nueva matriz
    const newPosition = new THREE.Vector3();
    const newQuaternion = new THREE.Quaternion();
    const newScale = new THREE.Vector3();
    newMatrix.decompose(newPosition, newQuaternion, newScale);

    // 7. Extraer Euler desde Quaternion (orden XYZ)
    const newEuler = new THREE.Euler().setFromQuaternion(newQuaternion, 'XYZ');

    // Normaliza ángulos a [-180, 180] grados
    const normalizeAngleDeg = (rad) => {
      let deg = rad * 180 / Math.PI;
      while (deg > 180) deg -= 360;
      while (deg < -180) deg += 360;
      return deg;
    };

    // 8. Actualizar estado
    const updatedPose = {
      x: newPosition.x * 1000,
      y: newPosition.y * 1000,
      z: newPosition.z * 1000,
      roll: normalizeAngleDeg(newEuler.x),
      pitch: normalizeAngleDeg(newEuler.y),
      yaw: normalizeAngleDeg(newEuler.z)
    };

    console.log('Updated pose:', updatedPose);
    return updatedPose;
  });
};



  const handleTCPAdjustment = (axis, increment) => {
    console.log(`Adjusting TCP for axis: ${axis}, increment: ${increment}`);

    // Step 1: Initialize point with default values
    const point = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
    point[axis] = increment; // Modify the specific axis
    console.log(`Initialized point:`, point);

    // Step 2: Build TCP coordinate system using current gripper_base pose
    const gripperBasePose = {
      x: values.x * 0.001, // Convert mm to meters
      y: values.y * 0.001,
      z: values.z * 0.001,
      roll: values.roll * Math.PI / 180, // Convert degrees to radians
      pitch: values.pitch * Math.PI / 180,
      yaw: values.yaw * Math.PI / 180
    };
    console.log(`GripperBase pose:`, gripperBasePose);

    // Step 3: Transform point from TCP to origin coordinate system
    const transformedPoint = transformPointToOrigin(point, gripperBasePose);
    console.log(`Transformed point:`, transformedPoint);

    // Step 4: Update NumberInputs with transformed values
    setValues((prevValues) => {
      const updatedValues = {
        ...prevValues,
        x: transformedPoint.x * 1000, // Convert back to mm
        y: transformedPoint.y * 1000,
        z: transformedPoint.z * 1000,
        roll: transformedPoint.roll * 180 / Math.PI, // Convert radians to degrees
        pitch: transformedPoint.pitch * 180 / Math.PI,
        yaw: transformedPoint.yaw * 180 / Math.PI
      };
      console.log(`Updated values:`, updatedValues);
      return updatedValues;
    });
  };

  const transformPointToOrigin = (point, gripperBasePose) => {
    console.log(`Transforming point:`, point, `using GripperBase pose:`, gripperBasePose);

    const { x, y, z, roll, pitch, yaw } = gripperBasePose;

    // Create rotation matrix from roll, pitch, yaw
    const cosRoll = Math.cos(roll * Math.PI / 180);
    const sinRoll = Math.sin(roll * Math.PI / 180);
    const cosPitch = Math.cos(pitch * Math.PI / 180);
    const sinPitch = Math.sin(pitch * Math.PI / 180);
    const cosYaw = Math.cos(yaw * Math.PI / 180);
    const sinYaw = Math.sin(yaw * Math.PI / 180);

    const rotationMatrix = [
      [cosYaw * cosPitch, cosYaw * sinPitch * sinRoll - sinYaw * cosRoll, cosYaw * sinPitch * cosRoll + sinYaw * sinRoll],
      [sinYaw * cosPitch, sinYaw * sinPitch * sinRoll + cosYaw * cosRoll, sinYaw * sinPitch * cosRoll - cosYaw * sinRoll],
      [-sinPitch, cosPitch * sinRoll, cosPitch * cosRoll]
    ];
    console.log(`Rotation matrix:`, rotationMatrix);

    // Apply rotation and translation to the point
    const transformedPoint = {
      x: rotationMatrix[0][0] * point.x + rotationMatrix[0][1] * point.y + rotationMatrix[0][2] * point.z + x,
      y: rotationMatrix[1][0] * point.x + rotationMatrix[1][1] * point.y + rotationMatrix[1][2] * point.z + y,
      z: rotationMatrix[2][0] * point.x + rotationMatrix[2][1] * point.y + rotationMatrix[2][2] * point.z + z,
      roll: point.roll + roll, // Orientation adjustments
      pitch: point.pitch + pitch,
      yaw: point.yaw + yaw
    };
    console.log(`Transformed point after applying rotation and translation:`, transformedPoint);

    return transformedPoint;
  };

  const [activeInterval, setActiveInterval] = useState(null);

  const handleMouseDown = (axis, increment) => {
    const intervalId = setInterval(() => handleTCPMove(axis, increment), 100); // Ejecuta cada 100ms
    setActiveInterval(intervalId);
  };

  const handleMouseUp = () => {
    if (activeInterval) {
      clearInterval(activeInterval);
      setActiveInterval(null);
    }
  };

  return (
    <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', marginTop: '0', marginBottom: '1rem' }}> {/* Center all content */}
      <div style={{ display: 'flex', justifyContent: 'center', gap: '1rem', marginBottom: '1rem' }}> {/* Row for X, Y, Z */}
        <NumberInput
          value={values.x}
          onChange={(newValue) => handleValueChange('x', newValue)}
          unit="mm"
          min={-500}
          max={500}
          step={1}
          label={<strong>X</strong>}
          axis="x"        />
        <NumberInput
          value={values.y}
          onChange={(newValue) => handleValueChange('y', newValue)}
          unit="mm"
          min={-500}
          max={500}
          step={1}
          label={<strong>Y</strong>}
          axis="y"        />
        <NumberInput
          value={values.z}
          onChange={(newValue) => handleValueChange('z', newValue)}
          unit="mm"
          min={0}
          max={1000}
          step={1}
          label={<strong>Z</strong>}
          axis="z"        />
      </div>
      <div style={{ display: 'flex', justifyContent: 'center', gap: '1rem', marginBottom: '1rem' }}> {/* Row for Roll, Pitch, Yaw */}
        <NumberInput
          value={values.roll}
          onChange={(newValue) => handleValueChange('roll', newValue)}
          unit="°"
          min={-180}
          max={180}
          step={1}
          label={<strong>Roll</strong>}
          axis="x"
        />
        <NumberInput
          value={values.pitch}
          onChange={(newValue) => handleValueChange('pitch', newValue)}
          unit="°"
          min={-180}
          max={180}
          step={1}
          label={<strong>Pitch</strong>}
          axis="y"
        />
        <NumberInput
          value={values.yaw}
          onChange={(newValue) => handleValueChange('yaw', newValue)}
          unit="°"
          min={-180}
          max={180}
          step={1}
          label={<strong>Yaw</strong>}
          axis="z"
        />
      </div>
      <h4 style={{ marginTop: '1.5rem', textAlign: 'center' }}>Robot Configuration</h4>
      <div style={{ display: 'flex', flexDirection: 'row', justifyContent: 'center', width: '70%', margin: '0 auto', gap: '2rem' }}> {/* Container for radio groups */}
        <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}> {/* Shoulder group */}
          <FormLabel component="legend" style={{ fontWeight: 'bold' }}>Shoulder</FormLabel>
          <RadioGroup
            value={ikConfig.shoulder}
            onChange={(e) => handleConfigChange('shoulder', e.target.value)}
          >
            <FormControlLabel value="left" control={<Radio disabled />} label="Left" />
            <FormControlLabel value="right" control={<Radio disabled />} label="Right" />
            <FormControlLabel value="any" control={<Radio disabled />} label="Any" />
          </RadioGroup>
        </div>
        <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}> {/* Elbow group */}
          <FormLabel component="legend" style={{ fontWeight: 'bold' }}>Elbow</FormLabel>
          <RadioGroup
            value={ikConfig.elbow}
            onChange={(e) => handleConfigChange('elbow', e.target.value)}
          >
            <FormControlLabel value="above" control={<Radio disabled />} label="Above" />
            <FormControlLabel value="below" control={<Radio disabled />} label="Below" />
            <FormControlLabel value="any" control={<Radio disabled />} label="Any" />
          </RadioGroup>
        </div>
        <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}> {/* Wrist group */}
          <FormLabel component="legend" style={{ fontWeight: 'bold' }}>Wrist</FormLabel>
          <RadioGroup
            value={ikConfig.wrist}
            onChange={(e) => handleConfigChange('wrist', e.target.value)}
          >
            <FormControlLabel value="up" control={<Radio disabled />} label="Up" />
            <FormControlLabel value="down" control={<Radio disabled />} label="Down" />
            <FormControlLabel value="any" control={<Radio disabled />} label="Any" />
          </RadioGroup>
        </div>
      </div>
      <Button
        onClick={sendIKGoal}
        disabled={isUnreachable}
        variant="contained"
        color={isUnreachable ? "secondary" : "primary"}
        style={{ marginTop: '1rem', fontWeight: 'bold' }}
      >
        Move
      </Button>
      {/* Added buttons for TCP coordinate adjustments */}
      <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', marginTop: '1rem' }}> {/* TCP Buttons */}
        <div style={{ display: 'flex', justifyContent: 'center', gap: '1rem', marginBottom: '1rem' }}> {/* Row for X, Y, Z */}
          <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', marginRight: '8px' }}> {/* X group */}
            <label style={{ marginBottom: '4px' }}><strong>X</strong></label>
            <ButtonGroup>
              <Button
                variant="contained"
                onMouseDown={() => handleMouseDown('x', -1)}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp} // Asegura que se detenga si el mouse sale del botón
                style={{ backgroundColor: '#ffcccc', color: 'black', border: '1px solid #ff9999' }}
              >
                -
              </Button>
              <Button
                variant="contained"
                onMouseDown={() => handleMouseDown('x', 1)}
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
                onMouseDown={() => handleMouseDown('y', -1)}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
                style={{ backgroundColor: '#ffffcc', color: 'black', border: '1px solid #cccc99' }}
              >
                -
              </Button>
              <Button
                variant="contained"
                onMouseDown={() => handleMouseDown('y', 1)}
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
                onMouseDown={() => handleMouseDown('z', -1)}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
                style={{ backgroundColor: '#ccccff', color: 'black', border: '1px solid #9999cc' }}
              >
                -
              </Button>
              <Button
                variant="contained"
                onMouseDown={() => handleMouseDown('z', 1)}
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
                onMouseDown={() => handleMouseDown('roll', -1)}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
                style={{ backgroundColor: '#ffcccc', color: 'black', border: '1px solid #ff9999' }}
              >
                -
              </Button>
              <Button
                variant="contained"
                onMouseDown={() => handleMouseDown('roll', 1)}
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
                onMouseDown={() => handleMouseDown('yaw', -1)}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
                style={{ backgroundColor: '#ffffcc', color: 'black', border: '1px solid #cccc99' }}
              >
                -
              </Button>
              <Button
                variant="contained"
                onMouseDown={() => handleMouseDown('yaw', 1)}
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
                onMouseDown={() => handleMouseDown('pitch', -1)}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
                style={{ backgroundColor: '#ccccff', color: 'black', border: '1px solid #9999cc' }}
              >
                -
              </Button>
              <Button
                variant="contained"
                onMouseDown={() => handleMouseDown('pitch', 1)}
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

    </div>
  );
}