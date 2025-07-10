import React, { useState, useEffect } from 'react';
import { useROS } from '../RosContext';
import ROSLIB from 'roslib';
import RadioGroup from '@mui/material/RadioGroup';
import FormControlLabel from '@mui/material/FormControlLabel';
import Radio from '@mui/material/Radio';
import FormLabel from '@mui/material/FormLabel';
import { styled } from '@mui/system';
import RemoveIcon from '@mui/icons-material/Remove';
import AddIcon from '@mui/icons-material/Add';
import AccordionSummary from '@mui/material/AccordionSummary';

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

const elbowOptions = [
  { value: 'up', label: 'Codo arriba' },
  { value: 'down', label: 'Codo abajo' }
];
const shoulderOptions = [
  { value: 'up', label: 'Hombro arriba' },
  { value: 'down', label: 'Hombro abajo' }
];
const wristOptions = [
  { value: 'up', label: 'Muñeca arriba' },
  { value: 'down', label: 'Muñeca abajo' }
];

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
  ({ theme }) => `
  font-size: 0.875rem;
  font-family: inherit;
  font-weight: 400;
  line-height: 1.375;
  color: ${theme.palette.mode === 'dark' ? grey[300] : grey[900]};
  background: ${theme.palette.mode === 'dark' ? grey[900] : '#fff'};
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
  appearance: textfield;

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

  &::-webkit-inner-spin-button, &::-webkit-outer-spin-button {
    -webkit-appearance: none;
    margin: 0;
  }
`,
);

const StyledButton = styled('button')(
  ({ theme }) => `
  font-family: 'IBM Plex Sans', sans-serif;
  font-size: 0.875rem;
  box-sizing: border-box;
  line-height: 1.5;
  border: 1px solid;
  border-radius: 999px;
  border-color: ${theme.palette.mode === 'dark' ? grey[800] : grey[200]};
  background: ${theme.palette.mode === 'dark' ? grey[900] : grey[50]};
  color: ${theme.palette.mode === 'dark' ? grey[200] : grey[900]};
  width: 32px;
  height: 32px;
  display: flex;
  flex-flow: row nowrap;
  justify-content: center;
  align-items: center;
  transition-property: all;
  transition-timing-function: cubic-bezier(0.4, 0, 0.2, 1);
  transition-duration: 120ms;

  &:hover {
    cursor: pointer;
    background: ${theme.palette.mode === 'dark' ? blue[700] : blue[500]};
    border-color: ${theme.palette.mode === 'dark' ? blue[500] : blue[400]};
    color: ${grey[50]};
  }

  &:focus-visible {
    outline: 0;
  }

  &.increment {
    order: 1;
  }
`,
);

const NumberInput = React.forwardRef(function CustomNumberInput(props, ref) {
  return (
    <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center' }}>
      <label style={{ marginBottom: '4px' }}>{props.label}</label>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
        <StyledButton
          onClick={() => props.onChange(props.value - props.increment)}
          disabled={props.value <= props.min}
        >
          <RemoveIcon fontSize="small" />
        </StyledButton>
        <StyledInput
          type="number"
          value={props.value}
          onChange={(e) => {
            const newValue = parseFloat(e.target.value);
            if (!isNaN(newValue)) {
              props.onChange(Math.min(Math.max(newValue, props.min), props.max));
            }
          }}
          min={props.min}
          max={props.max}
          step={props.step}
          ref={ref}
          style={{ textAlign: 'center', width: '4rem', margin: '0 8px' }}
        />
        <StyledButton
          onClick={() => props.onChange(props.value + props.increment)}
          disabled={props.value >= props.max}
        >
          <AddIcon fontSize="small" />
        </StyledButton>
      </div>
    </div>
  );
});

export default function IKSliders({ ikPose, onPreviewJointsChange, onIKStatusChange }) {
  const { ros, connected } = useROS();
  const [values, setValues] = useState({
    x: 200, y: 0, z: 300, roll: 0, pitch: 0, yaw: 0
  });
  const [ikConfig, setIkConfig] = useState({
    elbow: 'up',
    shoulder: 'up',
    wrist: 'up'
  });
  const [statusMsg, setStatusMsg] = useState(null);
  const [moveStep, setMoveStep] = useState(1);

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

  // Notifica cambios de sliders al padre para preview
  useEffect(() => {
    if (onPreviewJointsChange) {
      // Aquí deberías calcular las articulaciones objetivo a partir de los sliders
      // Por simplicidad, pasamos los valores de los sliders en radianes
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

  // --- NUEVO: Calcular IK en tiempo real para el ghost robot y actualizar statusMsg ---
  useEffect(() => {
    if (!ros || !connected) return;
    // Construir la petición para /compute_ik
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
    const req = {
      ik_request: {
        group_name: 'arm_group',
        pose_stamped: pose,
        ik_link_name: 'gripper_base',
        timeout: { sec: 0, nanosec: 0 },
        constraints: {}
      }
    };
    service.callService(req, (res) => {
      if (res && res.solution && res.solution.joint_state && res.error_code && res.error_code.val === 1) {
        // Mapear a objeto {joint_1: val, ...}
        const joints = {};
        res.solution.joint_state.name.forEach((name, i) => {
          joints[name] = res.solution.joint_state.position[i];
        });
        if (onPreviewJointsChange) onPreviewJointsChange(joints);
        setStatusMsg({ status: 'reachable' });
      } else {
        if (onPreviewJointsChange) onPreviewJointsChange({});
        setStatusMsg({ status: 'unreachable' });
      }
    });
    // eslint-disable-next-line
  }, [values, ros, connected]);

  // --- Botones de incremento/decremento con step configurable y repetición ---
  const intervalRef = React.useRef(null);
  const timeoutRef = React.useRef(null);

  const handleStep = (name, dir) => {
    setValues(prev => ({
      ...prev,
      [name]: parseFloat((parseFloat(prev[name]) + dir * step[name]).toFixed(3))
    }));
  };

  const handleValueChange = (key, newValue) => {
    setValues((prevValues) => ({ ...prevValues, [key]: newValue }));
  };

  const handleChange = (name, value) => {
    setValues(prev => ({
      ...prev,
      [name]: parseFloat(value)
    }));
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

  return (
    <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', marginTop: '0', marginBottom: '1rem' }}> {/* Center all content */}
      <div style={{ marginBottom: '1rem' }}>
        <NumberInput
          value={values.x}
          onChange={(newValue) => handleValueChange('x', newValue)}
          unit="mm"
          min={-500}
          max={500}
          step={1}
          increment={moveStep}
          label={<strong>X</strong>}
        />
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <NumberInput
          value={values.y}
          onChange={(newValue) => handleValueChange('y', newValue)}
          unit="mm"
          min={-500}
          max={500}
          step={1}
          increment={moveStep}
          label={<strong>Y</strong>}
        />
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <NumberInput
          value={values.z}
          onChange={(newValue) => handleValueChange('z', newValue)}
          unit="mm"
          min={0}
          max={1000}
          step={1}
          increment={moveStep}
          label={<strong>Z</strong>}
        />
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <NumberInput
          value={values.roll}
          onChange={(newValue) => handleValueChange('roll', newValue)}
          unit="°"
          min={-180}
          max={180}
          step={0.1}
          increment={moveStep}
          label={<strong>Roll</strong>}
        />
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <NumberInput
          value={values.pitch}
          onChange={(newValue) => handleValueChange('pitch', newValue)}
          unit="°"
          min={-180}
          max={180}
          step={0.1}
          increment={moveStep}
          label={<strong>Pitch</strong>}
        />
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <NumberInput
          value={values.yaw}
          onChange={(newValue) => handleValueChange('yaw', newValue)}
          unit="°"
          min={-180}
          max={180}
          step={0.1}
          increment={moveStep}
          label={<strong>Yaw</strong>}
        />
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <FormLabel component="legend">Increment (mm/º)</FormLabel>
        <RadioGroup
          row
          value={moveStep}
          onChange={(e) => setMoveStep(parseInt(e.target.value))}
        >
          <FormControlLabel value={1} control={<Radio />} label="1" />
          <FormControlLabel value={10} control={<Radio />} label="10" />
          <FormControlLabel value={100} control={<Radio />} label="100" />
        </RadioGroup>
      </div>
      <h4 style={{ marginTop: '1.5rem', textAlign: 'center' }}>Configuración preferida</h4> {/* Centered title */}
      <div style={{ marginBottom: '1rem' }}>
        <label>Codo:&nbsp;</label>
        <select value={ikConfig.elbow} onChange={e => handleConfigChange('elbow', e.target.value)}>
          {elbowOptions.map(opt => <option key={opt.value} value={opt.value}>{opt.label}</option>)}
        </select>
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <label>Hombro:&nbsp;</label>
        <select value={ikConfig.shoulder} onChange={e => handleConfigChange('shoulder', e.target.value)}>
          {shoulderOptions.map(opt => <option key={opt.value} value={opt.value}>{opt.label}</option>)}
        </select>
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <label>Muñeca:&nbsp;</label>
        <select value={ikConfig.wrist} onChange={e => handleConfigChange('wrist', e.target.value)}>
          {wristOptions.map(opt => <option key={opt.value} value={opt.value}>{opt.label}</option>)}
        </select>
      </div>
      <button
        onClick={sendIKGoal}
        disabled={isUnreachable}
        style={{
          marginTop: '1rem',
          padding: '0.5rem 1rem',
          backgroundColor: isUnreachable ? '#aaa' : '#007bff',
          color: '#fff',
          border: 'none',
          borderRadius: '5px',
          cursor: isUnreachable ? 'not-allowed' : 'pointer'
        }}
      >
        {isUnreachable ? 'Position unreachable' : 'Enviar a posición'}
      </button>
    </div>
  );
}