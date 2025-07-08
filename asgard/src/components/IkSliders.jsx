import React, { useState, useEffect } from 'react';
import { useROS } from '../RosContext';
import ROSLIB from 'roslib';

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
  const [moveStep, setMoveStep] = useState(1);
  const step = { x: moveStep, y: moveStep, z: moveStep, roll: 1, pitch: 1, yaw: 1 };
  const intervalRef = React.useRef(null);
  const timeoutRef = React.useRef(null);

  const handleStep = (name, dir) => {
    setValues(prev => ({
      ...prev,
      [name]: parseFloat((parseFloat(prev[name]) + dir * step[name]).toFixed(3))
    }));
  };

  // Repetición al mantener pulsado
  const handleStepMouseDown = (name, dir) => {
    handleStep(name, dir);
    timeoutRef.current = setTimeout(() => {
      intervalRef.current = setInterval(() => handleStep(name, dir), 80);
    }, 350);
  };
  const handleStepMouseUp = () => {
    clearTimeout(timeoutRef.current);
    clearInterval(intervalRef.current);
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
    <div style={{ marginTop: '2rem' }}>
      <h3>🎯 Cinemática inversa (IK)</h3>
      <div style={{ marginBottom: '1rem' }}>
        <label>Incremento XYZ:&nbsp;</label>
        <label style={{marginRight:'1em'}}>
          <input type="radio" name="step" value={1} checked={moveStep===1} onChange={()=>setMoveStep(1)} /> 1 mm
        </label>
        <label style={{marginRight:'1em'}}>
          <input type="radio" name="step" value={10} checked={moveStep===10} onChange={()=>setMoveStep(10)} /> 10 mm
        </label>
        <label>
          <input type="radio" name="step" value={100} checked={moveStep===100} onChange={()=>setMoveStep(100)} /> 100 mm
        </label>
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <label>X:&nbsp;</label>
        <button onMouseDown={() => handleStepMouseDown('x', -1)} onMouseUp={handleStepMouseUp} onMouseLeave={handleStepMouseUp}>-</button>
        <input type="number" min="-400" max="400" step="1" value={values.x} onChange={e => handleChange('x', e.target.value)} style={{ width: '60px' }} /> mm
        <button onMouseDown={() => handleStepMouseDown('x', 1)} onMouseUp={handleStepMouseUp} onMouseLeave={handleStepMouseUp}>+</button>
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <label>Y:&nbsp;</label>
        <button onMouseDown={() => handleStepMouseDown('y', -1)} onMouseUp={handleStepMouseUp} onMouseLeave={handleStepMouseUp}>-</button>
        <input type="number" min="-400" max="400" step="1" value={values.y} onChange={e => handleChange('y', e.target.value)} style={{ width: '60px' }} /> mm
        <button onMouseDown={() => handleStepMouseDown('y', 1)} onMouseUp={handleStepMouseUp} onMouseLeave={handleStepMouseUp}>+</button>
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <label>Z:&nbsp;</label>
        <button onMouseDown={() => handleStepMouseDown('z', -1)} onMouseUp={handleStepMouseUp} onMouseLeave={handleStepMouseUp}>-</button>
        <input type="number" min="0" max="600" step="1" value={values.z} onChange={e => handleChange('z', e.target.value)} style={{ width: '60px' }} /> mm
        <button onMouseDown={() => handleStepMouseDown('z', 1)} onMouseUp={handleStepMouseUp} onMouseLeave={handleStepMouseUp}>+</button>
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <label>Roll:&nbsp;</label>
        <button onMouseDown={() => handleStepMouseDown('roll', -1)} onMouseUp={handleStepMouseUp} onMouseLeave={handleStepMouseUp}>-</button>
        <input type="number" min={-180} max={180} step={0.1} value={values.roll} onChange={e => handleChange('roll', e.target.value)} style={{ width: '60px' }} /> °
        <button onMouseDown={() => handleStepMouseDown('roll', 1)} onMouseUp={handleStepMouseUp} onMouseLeave={handleStepMouseUp}>+</button>
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <label>Pitch:&nbsp;</label>
        <button onMouseDown={() => handleStepMouseDown('pitch', -1)} onMouseUp={handleStepMouseUp} onMouseLeave={handleStepMouseUp}>-</button>
        <input type="number" min={-180} max={180} step={0.1} value={values.pitch} onChange={e => handleChange('pitch', e.target.value)} style={{ width: '60px' }} /> °
        <button onMouseDown={() => handleStepMouseDown('pitch', 1)} onMouseUp={handleStepMouseUp} onMouseLeave={handleStepMouseUp}>+</button>
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <label>Yaw:&nbsp;</label>
        <button onMouseDown={() => handleStepMouseDown('yaw', -1)} onMouseUp={handleStepMouseUp} onMouseLeave={handleStepMouseUp}>-</button>
        <input type="number" min={-180} max={180} step={0.1} value={values.yaw} onChange={e => handleChange('yaw', e.target.value)} style={{ width: '60px' }} /> °
        <button onMouseDown={() => handleStepMouseDown('yaw', 1)} onMouseUp={handleStepMouseUp} onMouseLeave={handleStepMouseUp}>+</button>
      </div>
      <h4 style={{ marginTop: '1.5rem' }}>Configuración preferida</h4>
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