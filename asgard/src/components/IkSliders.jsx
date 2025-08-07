import React, { useState, useEffect } from 'react';
import { useROS } from '../RosContext';
import ROSLIB from 'roslib';
import { styled } from '@mui/system';
import Button from '@mui/material/Button';
import ButtonGroup from '@mui/material/ButtonGroup';
import * as THREE from 'three';

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
  
  // Estados principales: posición y quaternion base (fuentes de verdad)
  const [position, setPosition] = useState(new THREE.Vector3(200, 0, 300)); // en mm
  const [baseQuaternion, setBaseQuaternion] = useState(new THREE.Quaternion(0, 0, 0, 1)); // identidad
  
  // Estados para funcionalidad
  const [ikConfig, setIkConfig] = useState({
    elbow: 'any',
    shoulder: 'any',
    wrist: 'any'
  });
  const [statusMsg, setStatusMsg] = useState(null);
  const [selectedGripper, setSelectedGripper] = useState(0);
  const [activeInterval, setActiveInterval] = useState(null);

  // Valores derivados para UI (calculados en tiempo real)
  const displayEuler = React.useMemo(() => {
    const euler = new THREE.Euler().setFromQuaternion(baseQuaternion, 'XYZ');
    return {
      roll: euler.x * 180 / Math.PI,
      pitch: euler.y * 180 / Math.PI,
      yaw: euler.z * 180 / Math.PI
    };
  }, [baseQuaternion]);

  const displayValues = React.useMemo(() => ({
    x: position.x,
    y: position.y,
    z: position.z,
    roll: displayEuler.roll,
    pitch: displayEuler.pitch,
    yaw: displayEuler.yaw
  }), [position, displayEuler]);

  // Inicializa SOLO cuando cambia ikPose
useEffect(() => {
  if (ikPose) {
    setPosition(new THREE.Vector3(
      Math.round(ikPose.x),
      Math.round(ikPose.y),
      Math.round(ikPose.z)
    ));
    // Usar cuaterniones directamente
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

  // Inicializa cuando cambia initialPose
useEffect(() => {
  if (initialPose) {
    setPosition(new THREE.Vector3(
      initialPose.x || 0,
      initialPose.y || 0,
      initialPose.z || 0
    ));
    // Usar cuaterniones directamente
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

  // Notificar cambios de pose
  useEffect(() => {
    if (onPreviewJointsChange) {
      onPreviewJointsChange({
        x: position.x,
        y: position.y,
        z: position.z,
        roll: displayEuler.roll * Math.PI / 180,
        pitch: displayEuler.pitch * Math.PI / 180,
        yaw: displayEuler.yaw * Math.PI / 180
      });
    }
  }, [position, displayEuler, onPreviewJointsChange]);

  // Peticiones IK (cuando cambian posición o quaternion)
  useEffect(() => {
    if (!ros || !connected) return;
    console.log('Sending IK request with pose:', { position: position.toArray(), quaternion: baseQuaternion.toArray() });
    
    const service = new ROSLIB.Service({
      ros,
      name: '/compute_ik',
      serviceType: 'moveit_msgs/srv/GetPositionIK'
    });
    
    const pose = {
      header: { frame_id: 'base_link' },
      pose: {
        position: {
          x: position.x / 1000,
          y: position.y / 1000,
          z: position.z / 1000
        },
        orientation: {
          x: baseQuaternion.x,
          y: baseQuaternion.y,
          z: baseQuaternion.z,
          w: baseQuaternion.w
        }
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
        ik_link_name: 'gripper_mid_point',
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
  }, [position, baseQuaternion, ros, connected, onPreviewJointsChange, ghostJoints]);

  const handleValueChange = (key, newValue) => {
    if (key === 'x' || key === 'y' || key === 'z') {
      setPosition(prevPos => {
        const newPos = prevPos.clone();
        newPos[key] = newValue;
        return newPos;
      });
    } else {
      // Para cambios en Euler, actualizar el quaternion base
      const currentEuler = new THREE.Euler().setFromQuaternion(baseQuaternion, 'XYZ');
      if (key === 'roll') currentEuler.x = newValue * Math.PI / 180;
      if (key === 'pitch') currentEuler.y = newValue * Math.PI / 180;
      if (key === 'yaw') currentEuler.z = newValue * Math.PI / 180;
      setBaseQuaternion(new THREE.Quaternion().setFromEuler(currentEuler));
    }
  };

  const sendJointCommand = () => {
    if (!connected || !ros) {
      console.warn('ROS no está conectado.');
      return;
    }
    if (!ghostJoints) {
      console.warn('No hay ghostJoints disponibles.');
      return;
    }
    // Articulaciones principales y gripper
    const jointNames = [
      'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'gripperbase_to_armgearright'
    ];
    // Construir el array de posiciones en radianes
    const jointPositions = jointNames.map(name => ghostJoints[name] !== undefined ? ghostJoints[name] : 0);
    // Calcular el valor del gripper según selectedGripper
    const gripperIdx = jointNames.indexOf('gripperbase_to_armgearright');
    let gripperValue = 0;
    if (selectedGripper === 0) {
      gripperValue = 0;
    } else if (selectedGripper === 100) {
      gripperValue = -89.9 * Math.PI / 180;
    } else {
      gripperValue = -89.9 * (selectedGripper / 100) * Math.PI / 180;
    }
    jointPositions[gripperIdx] = gripperValue;
    // Publicar en el topic igual que JointSliders.jsx
    const topic = new ROSLIB.Topic({
      ros,
      name: '/joint_group_position_controller/command',
      messageType: 'std_msgs/Float64MultiArray',
    });
    const message = new ROSLIB.Message({
      data: jointPositions,
    });
    console.log('Enviando comando articular:', message.data);
    topic.publish(message);
  };

  const handleWorldMove = (axis, increment) => {
    console.log(`Moving World for axis: ${axis}, increment: ${increment}`);

    if (axis === 'x' || axis === 'y' || axis === 'z') {
      // Movimiento lineal global (sin transformar)
      setPosition(prevPos => {
        const newPos = prevPos.clone();
        newPos[axis] += increment; // mm en coordenadas globales
        return newPos;
      });
    } else {
      // Rotación: ejes globales fijos (sin transformar)
      setBaseQuaternion(prevQuat => {
        let rotAxis;
        if (axis === 'roll') rotAxis = new THREE.Vector3(1, 0, 0);
        if (axis === 'pitch') rotAxis = new THREE.Vector3(0, 1, 0);
        if (axis === 'yaw') rotAxis = new THREE.Vector3(0, 0, 1);
        
        // Crear quaternion de rotación incremental en ejes globales
        const deltaQ = new THREE.Quaternion().setFromAxisAngle(rotAxis, increment * Math.PI / 180);
        
        // Aplicar rotación global
        return prevQuat.clone().premultiply(deltaQ);
      });
    }
  };

  const handleTCPMove = (axis, increment) => {
    console.log(`Moving TCP for axis: ${axis}, increment: ${increment}`);

    if (axis === 'x' || axis === 'y' || axis === 'z') {
      // Movimiento lineal: transformar delta por quaternion actual (coordenadas locales)
      setPosition(prevPos => {
        const localDelta = new THREE.Vector3();
        localDelta[axis] = increment; // mm
        localDelta.applyQuaternion(baseQuaternion); // transformar a coordenadas globales
        return prevPos.clone().add(localDelta);
      });
    } else {
      // Rotación: usar lógica de GizmoPage (quaternion puro)
      setBaseQuaternion(prevQuat => {
        const q = prevQuat.clone();
        let rotAxis;
        if (axis === 'roll') rotAxis = new THREE.Vector3(1, 0, 0);
        if (axis === 'pitch') rotAxis = new THREE.Vector3(0, 1, 0);
        if (axis === 'yaw') rotAxis = new THREE.Vector3(0, 0, 1);
        
        // Transformar eje a coordenadas locales actuales
        rotAxis.applyQuaternion(q);
        
        // Crear quaternion de rotación incremental
        const deltaQ = new THREE.Quaternion().setFromAxisAngle(rotAxis, increment * Math.PI / 180);
        
        // Aplicar rotación
        return q.premultiply(deltaQ);
      });
    }
  };

  const handleMouseDown = (axis, increment, frameType = 'tcp') => {
    const moveFunction = frameType === 'world' ? handleWorldMove : handleTCPMove;
    const intervalId = setInterval(() => moveFunction(axis, increment), 100); // Ejecuta cada 100ms
    setActiveInterval(intervalId);
  };

  const handleMouseUp = () => {
    if (activeInterval) {
      clearInterval(activeInterval);
      setActiveInterval(null);
    }
  };

  useEffect(() => {
    if (ghostJoints && onPreviewJointsChange) {
      const gripperValue = selectedGripper === 0 
        ? 0 
        : selectedGripper === 100 
        ? (-89.9 * Math.PI / 180) 
        : (-89.9 * (selectedGripper / 100) * Math.PI / 180);
      if (ghostJoints['gripperbase_to_armgearright'] !== gripperValue) {
        const updatedJoints = { ...ghostJoints };
        updatedJoints['gripperbase_to_armgearright'] = gripperValue;
        onPreviewJointsChange(updatedJoints);
      }
    }
  }, [selectedGripper, ghostJoints, onPreviewJointsChange]);

  return (
    <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', marginTop: '0', marginBottom: '1rem' }}>
      {/* Posición y orientación actual del TCP ghost */}
      <div style={{ marginBottom: '1rem', padding: '0.5rem 1rem', background: '#f7f7ff', borderRadius: '8px', border: '1px solid #ccccff', maxWidth: '500px' }}>
        <div><strong>TCP Position (mm):</strong></div>
        <div style={{ fontFamily: 'monospace', fontSize: '0.95em' }}>
          {displayValues.x.toFixed(1)}, {displayValues.y.toFixed(1)}, {displayValues.z.toFixed(1)}
        </div>
        <div style={{ marginTop: '0.5em' }}><strong>TCP Orientation (Quaternion):</strong></div>
        <div style={{ fontFamily: 'monospace', fontSize: '0.95em' }}>
          {baseQuaternion.x.toFixed(5)}, {baseQuaternion.y.toFixed(5)}, {baseQuaternion.z.toFixed(5)}, {baseQuaternion.w.toFixed(5)}
        </div>
      </div>

       <h3 style={{ marginTop: '1.5rem', textAlign: 'center' }}>World Frame</h3>
      <div style={{ display: 'flex', justifyContent: 'center', gap: '1rem', marginBottom: '1rem' }}> {/* Row for X, Y, Z as buttons */}
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
        <h3 style={{ marginTop: '1.5rem', textAlign: 'center' }}>TCP Frame</h3>
        <div style={{ display: 'flex', justifyContent: 'center', gap: '1rem', marginBottom: '1rem' }}> {/* Row for X, Y, Z */}
          <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', marginRight: '8px' }}> {/* X group */}
            <label style={{ marginBottom: '4px' }}><strong>X</strong></label>
            <ButtonGroup>
              <Button
                variant="contained"
                onMouseDown={() => handleMouseDown('x', -1, 'tcp')}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp} // Asegura que se detenga si el mouse sale del botón
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
        <h3 style={{ marginTop: '1.5rem', textAlign: 'center' }}>Gripper Control</h3>
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