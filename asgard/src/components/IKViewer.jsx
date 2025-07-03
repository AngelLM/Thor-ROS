import React, { useEffect, useState } from 'react';
import { useROS } from '../RosContext';
import ROSLIB from 'roslib';

export default function IKViewer({ onCopyPose }) {
  const { ros, connected } = useROS();
  const [ee, setEe] = useState(null);
  const [unit, setUnit] = useState('rad'); // 'rad' o 'deg'

  useEffect(() => {
    if (!ros || !connected) return;

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: '/robot_status',
      messageType: 'std_msgs/msg/String'
    });

    const callback = (msg) => {
      try {
        const data = JSON.parse(msg.data);
        setEe(data.end_effector);
      } catch (e) {
        setEe(null);
      }
    };

    topic.subscribe(callback);
    return () => topic.unsubscribe();
  }, [ros, connected]);

  const formatAngle = (val) => {
    if (val === null || val === undefined) return 'n/a';
    if (unit === 'rad') return `${val.toFixed(3)} rad`;
    return `${(val * 180 / Math.PI).toFixed(2)}°`;
  };

  const handleCopy = () => {
    if (ee && onCopyPose) {
      onCopyPose({ ...ee });
    }
  };

  return (
    <div>
      <h2>📍 Posición del extremo</h2>
      <div style={{ marginBottom: '1rem' }}>
        <label htmlFor="ee-unit-select">Ángulos en:&nbsp;</label>
        <select
          id="ee-unit-select"
          value={unit}
          onChange={e => setUnit(e.target.value)}
        >
          <option value="rad">Radianes</option>
          <option value="deg">Grados</option>
        </select>
      </div>
      {!ee ? (
        <p>Esperando datos de /robot_status...</p>
      ) : (
        <div>
          <div><strong>X:</strong> {ee.x} mm</div>
          <div><strong>Y:</strong> {ee.y} mm</div>
          <div><strong>Z:</strong> {ee.z} mm</div>
          <div><strong>Roll:</strong> {formatAngle(ee.roll)}</div>
          <div><strong>Pitch:</strong> {formatAngle(ee.pitch)}</div>
          <div><strong>Yaw:</strong> {formatAngle(ee.yaw)}</div>
          <button onClick={handleCopy} style={{ marginTop: '1rem', padding: '0.5rem 1rem' }}>
            Copiar a sliders
          </button>
        </div>
      )}
    </div>
  );
}