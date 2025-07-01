import React, { useEffect, useState } from 'react';
import { useROS } from '../RosContext';
import ROSLIB from 'roslib';

const jointLabels = {
  joint_1: 'Art 1',
  joint_2: 'Art 2',
  joint_3: 'Art 3',
  joint_4: 'Art 4',
  joint_5: 'Art 5',
  joint_6: 'Art 6',
  gripperbase_to_armgearright: 'Gripper'
};

export default function JointStateViewer() {
  const { ros, connected } = useROS();
  const [joints, setJoints] = useState([]);
  const [unit, setUnit] = useState('rad'); // 'rad' o 'deg'

  useEffect(() => {
    if (!ros || !connected) return;

    const listener = new ROSLIB.Topic({
      ros: ros,
      name: '/joint_states',
      messageType: 'sensor_msgs/msg/JointState'
    });

    const callback = (message) => {
      const result = message.name.map((name, idx) => ({
        name,
        position: message.position[idx] ?? null
      }));
      setJoints(result);
    };

    listener.subscribe(callback);

    return () => {
      listener.unsubscribe();
    };
  }, [ros, connected]);

  const formatPosition = (pos) => {
    if (pos === null) return 'n/a';
    if (unit === 'rad') return `${pos.toFixed(3)} rad`;
    return `${(pos * 180 / Math.PI).toFixed(2)}°`;
  };

  return (
    <div>
      <div style={{ marginBottom: '1rem' }}>
        <label htmlFor="unit-select">Units:&nbsp;</label>
        <select
          id="unit-select"
          value={unit}
          onChange={e => setUnit(e.target.value)}
        >
          <option value="rad">Radians</option>
          <option value="deg">Degrees</option>
        </select>
      </div>
      {joints.length === 0 ? (
        <p>Waiting data from /joint_states...</p>
      ) : (
        <div>
          {joints.map((joint) => (
            <div key={joint.name} style={{ marginBottom: '0.5rem' }}>
              <strong>{jointLabels[joint.name] || joint.name}:</strong> {formatPosition(joint.position)}
            </div>
          ))}
        </div>
      )}
    </div>
  );
}