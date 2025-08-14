import React, { useEffect, useState } from 'react';

const jointOrder = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'];

function radToDeg(r) {
  return r * 180 / Math.PI;
}

export default function RobotState({ urdfApi, currentJoints, ghostJoints }) {
  const [realTCP, setRealTCP] = useState(null);
  const [ghostTCP, setGhostTCP] = useState(null);

  useEffect(() => {
    if (!urdfApi) return;
    // Prefer explicit API if available
    if (typeof urdfApi.getGhostState === 'function') {
      try {
        const s = urdfApi.getGhostState && urdfApi.getGhostState();
        if (s && s.tcp) setGhostTCP(s.tcp);
      } catch {}
    }
    if (typeof urdfApi.getTCPFromJoints === 'function' && currentJoints) {
      try {
        const t = urdfApi.getTCPFromJoints(currentJoints);
        if (t) setRealTCP(t);
      } catch {}
    }
  }, [urdfApi, currentJoints]);

  useEffect(() => {
    if (!urdfApi) return;
    if (typeof urdfApi.getTCPFromJoints === 'function' && ghostJoints) {
      try {
        const t = urdfApi.getTCPFromJoints(ghostJoints);
        if (t) setGhostTCP(t);
      } catch {}
    } else if (typeof urdfApi.getGhostState === 'function') {
      try {
        const s = urdfApi.getGhostState && urdfApi.getGhostState();
        if (s && s.tcp) setGhostTCP(s.tcp);
      } catch {}
    }
  }, [urdfApi, ghostJoints]);

  const formatArmJoints = (joints) => {
    if (!joints) return 'N/A';
    const vals = jointOrder.map((n) => {
      const v = joints[n];
      if (v === undefined || v === null) return 'N/A';
      return `${radToDeg(v).toFixed(1)}°`;
    });
    return vals.join(', ');
  };

  const formatGripper = (joints) => {
    if (!joints) return 'N/A';
    const g = joints['gripperbase_to_armgearright'];
    if (g === undefined || g === null) return 'N/A';
    return `${Math.round(Math.abs(radToDeg(g)))}°`;
  };

  const formatPosition = (tcp) => {
    if (!tcp) return 'N/A';
    const { x, y, z } = tcp;
    // tcp might be in meters or mm depending on source; attempt to detect: if values < 10 assume meters and convert to mm
    const factor = (Math.abs(x) < 10 && Math.abs(y) < 10 && Math.abs(z) < 10) ? 1000 : 1;
    return `${(x*factor).toFixed(1)}mm, ${(y*factor).toFixed(1)}mm, ${(z*factor).toFixed(1)}mm`;
  };

  const formatOrientation = (tcp) => {
    if (!tcp) return 'N/A';
    const { qx, qy, qz, qw } = tcp;
    return `${qx.toFixed(1)}, ${qy.toFixed(1)}, ${qz.toFixed(1)}, ${qw.toFixed(1)}`;
  };

  return (
    <div style={{ width: '100%', background: '#f3f6fa', borderTop: '1px solid #ccc', padding: '0.25rem 0.75rem', fontSize: '0.85rem', fontFamily: 'monospace', display: 'flex', flexDirection: 'column', justifyContent: 'center', gap: '0.15rem' }}>
      <div>
    <strong style={{ fontSize: '1rem' }}>Real Robot{'\u00A0'}</strong> - <strong><em>Joints</em></strong>: {formatArmJoints(currentJoints)} - <strong><em>Gripper</em></strong>: {formatGripper(currentJoints)} - <strong><em>Position</em></strong>: {formatPosition(realTCP)} - <strong><em>Orientation</em></strong>: {formatOrientation(realTCP)}
      </div>
      <div>
    <strong style={{ fontSize: '1rem' }}>Ghost Robot</strong>  - <strong><em>Joints</em></strong>: {formatArmJoints(ghostJoints)} - <strong><em>Gripper</em></strong>: {formatGripper(ghostJoints)} - <strong><em>Position</em></strong>: {formatPosition(ghostTCP)} - <strong><em>Orientation</em></strong>: {formatOrientation(ghostTCP)}
      </div>
    </div>
  );
}
