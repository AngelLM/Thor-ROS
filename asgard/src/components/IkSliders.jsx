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

export default function IKSliders({ ikPose, onPreviewJointsChange }) {
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
      } catch {
        setStatusMsg({ status: "error", detail: "Error interpretando el mensaje de estado" });
      }
    };
    statusTopic.subscribe(cb);
    return () => statusTopic.unsubscribe(cb);
  }, [ros, connected]);

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

  // --- NUEVO: Calcular IK en tiempo real para el ghost robot ---
  useEffect(() => {
    if (!ros || !connected || !onPreviewJointsChange) return;
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
        constraints: {} // sin constraints extra para preview rápido
      }
    };
    service.callService(req, (res) => {
      if (res && res.solution && res.solution.joint_state && res.error_code && res.error_code.val === 1) {
        // Mapear a objeto {joint_1: val, ...}
        const joints = {};
        res.solution.joint_state.name.forEach((name, i) => {
          joints[name] = res.solution.joint_state.position[i];
        });
        onPreviewJointsChange(joints);
      } else {
        // Si no hay solución, ghost en posición neutra
        onPreviewJointsChange({});
      }
    });
    // eslint-disable-next-line
  }, [values, ros, connected]);

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
        <label>X:&nbsp;</label>
        <input type="range" min="-400" max="400" step="1" value={values.x} onChange={e => handleChange('x', e.target.value)} />
        <input type="number" min="-400" max="400" step="1" value={values.x} onChange={e => handleChange('x', e.target.value)} style={{ width: '60px' }} /> mm
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <label>Y:&nbsp;</label>
        <input type="range" min="-400" max="400" step="1" value={values.y} onChange={e => handleChange('y', e.target.value)} />
        <input type="number" min="-400" max="400" step="1" value={values.y} onChange={e => handleChange('y', e.target.value)} style={{ width: '60px' }} /> mm
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <label>Z:&nbsp;</label>
        <input type="range" min="0" max="600" step="1" value={values.z} onChange={e => handleChange('z', e.target.value)} />
        <input type="number" min="0" max="600" step="1" value={values.z} onChange={e => handleChange('z', e.target.value)} style={{ width: '60px' }} /> mm
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <label>Roll:&nbsp;</label>
        <input type="range" min={-180} max={180} step={0.1} value={values.roll} onChange={e => handleChange('roll', e.target.value)} />
        <input type="number" min={-180} max={180} step={0.1} value={values.roll} onChange={e => handleChange('roll', e.target.value)} style={{ width: '60px' }} /> °
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <label>Pitch:&nbsp;</label>
        <input type="range" min={-180} max={180} step={0.1} value={values.pitch} onChange={e => handleChange('pitch', e.target.value)} />
        <input type="number" min={-180} max={180} step={0.1} value={values.pitch} onChange={e => handleChange('pitch', e.target.value)} style={{ width: '60px' }} /> °
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <label>Yaw:&nbsp;</label>
        <input type="range" min={-180} max={180} step={0.1} value={values.yaw} onChange={e => handleChange('yaw', e.target.value)} />
        <input type="number" min={-180} max={180} step={0.1} value={values.yaw} onChange={e => handleChange('yaw', e.target.value)} style={{ width: '60px' }} /> °
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
        style={{
          marginTop: '1rem',
          padding: '0.5rem 1rem',
          backgroundColor: '#007bff',
          color: '#fff',
          border: 'none',
          borderRadius: '5px',
          cursor: 'pointer'
        }}
      >
        Mover a posición
      </button>
      {/* Mensaje de estado mejorado */}
      {statusMsg && (
        <div
          style={{
            marginTop: '1rem',
            padding: '0.75rem',
            borderRadius: '6px',
            fontWeight: 'bold',
            background:
              statusMsg.status === 'unreachable' ? '#ffeaea' :
              statusMsg.status === 'reachable_exact' ? '#eaffea' :
              statusMsg.status === 'reachable_approx' ? '#fffbe6' :
              '#eaf4ff',
            color:
              statusMsg.status === 'unreachable' ? '#c00' :
              statusMsg.status === 'reachable_exact' ? '#080' :
              statusMsg.status === 'reachable_approx' ? '#b59a00' :
              '#0057b8',
            border:
              statusMsg.status === 'unreachable' ? '1.5px solid #c00' :
              statusMsg.status === 'reachable_exact' ? '1.5px solid #080' :
              statusMsg.status === 'reachable_approx' ? '1.5px solid #b59a00' :
              '1.5px solid #0057b8',
            display: 'flex',
            alignItems: 'center',
            gap: '0.5em'
          }}
        >
          {statusMsg.status === 'unreachable' && '❌'}
          {statusMsg.status === 'reachable_exact' && '✅'}
          {statusMsg.status === 'reachable_approx' && '🟡'}
          {statusMsg.status === 'reachable' && '🟢'}
          <span style={{fontSize: '1em'}}>{statusMsg.status.replace('_', ' ').toUpperCase()}</span>
          <span style={{fontWeight: 'normal', marginLeft: '0.5em'}}>{statusMsg.detail}</span>
        </div>
      )}
    </div>
  );
}