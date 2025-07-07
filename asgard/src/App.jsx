import React, { useState, useEffect, useRef } from 'react';
import './App.css';
import { RosProvider } from './RosContext';
import JointStateViewer from './components/JointStateViewer';
import UrdfViewer from './components/UrdfViewer';
import JointSliders from './components/JointSliders';
import IKViewer from './components/IKViewer';
import IKSliders from './components/IkSliders';
import ROSLIB from 'roslib';

function App() {
  const [activeTab, setActiveTab] = useState('forward');
  const [ikPose, setIkPose] = useState(null); // Estado compartido para la pose IK
  const [previewJoints, setPreviewJoints] = useState(null); // Estado para las articulaciones objetivo (IK)
  const [fkJoints, setFkJoints] = useState(null); // Estado para las articulaciones objetivo (FK)
  const [currentJoints, setCurrentJoints] = useState(null); // Estado articular actual del robot
  const [showRealRobot, setShowRealRobot] = useState(true);
  const [showGhostRobot, setShowGhostRobot] = useState(true);
  const [ikStatus, setIkStatus] = useState('reachable');
  const rosRef = useRef(null);
  const lastJointsOnTabChange = useRef(null);
  const lastPoseOnTabChange = useRef(null);

  // Suscribirse a /joint_states para obtener la posición actual
  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
    rosRef.current = ros;
    const jointStateListener = new ROSLIB.Topic({
      ros,
      name: '/joint_states',
      messageType: 'sensor_msgs/msg/JointState'
    });
    jointStateListener.subscribe((msg) => {
      // Mapear a objeto {joint_1: val, ...}
      const joints = {};
      msg.name.forEach((name, i) => {
        joints[name] = msg.position[i];
      });
      setCurrentJoints(joints);
    });
    return () => {
      jointStateListener.unsubscribe();
      ros.close();
    };
  }, []);

  // Solo inicializar sliders al cambiar de tab
  useEffect(() => {
    if (activeTab === 'forward' && currentJoints) {
      lastJointsOnTabChange.current = { ...currentJoints };
      setFkJoints({ ...currentJoints });
    }
    if (activeTab === 'inverse' && currentJoints) {
      // Llamar a servicio FK para obtener la pose cartesiana actual
      const ros = rosRef.current;
      if (!ros) return;
      const service = new ROSLIB.Service({
        ros,
        name: '/compute_fk',
        serviceType: 'moveit_msgs/srv/GetPositionFK'
      });
      const req = {
        header: { frame_id: 'base_link' },
        fk_link_names: ['gripper_base'],
        robot_state: {
          joint_state: {
            name: Object.keys(currentJoints),
            position: Object.values(currentJoints)
          }
        }
      };
      service.callService(req, (res) => {
        if (res && res.pose_stamped && res.pose_stamped.length > 0) {
          const pose = res.pose_stamped[0].pose;
          // Convertir quaternion a RPY
          const q = pose.orientation;
          const t = pose.position;
          function quaternionToRPY(q) {
            const ysqr = q.y * q.y;
            let t0 = +2.0 * (q.w * q.x + q.y * q.z);
            let t1 = +1.0 - 2.0 * (q.x * q.x + ysqr);
            let roll = Math.atan2(t0, t1);
            let t2 = +2.0 * (q.w * q.y - q.z * q.x);
            t2 = t2 > 1.0 ? 1.0 : t2;
            t2 = t2 < -1.0 ? -1.0 : t2;
            let pitch = Math.asin(t2);
            let t3 = +2.0 * (q.w * q.z + q.x * q.y);
            let t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
            let yaw = Math.atan2(t3, t4);
            return { roll, pitch, yaw };
          }
          const rpy = quaternionToRPY(q);
          const poseObj = {
            x: t.x * 1000,
            y: t.y * 1000,
            z: t.z * 1000,
            roll: rpy.roll,
            pitch: rpy.pitch,
            yaw: rpy.yaw
          };
          lastPoseOnTabChange.current = poseObj;
          setIkPose(poseObj);
        }
      });
    }
    // eslint-disable-next-line
  }, [activeTab]);

  // Determina qué preview mostrar
  const effectivePreviewJoints = activeTab === 'forward' ? fkJoints : previewJoints;

  return (
    <RosProvider>
      <div className="app-layout">
        <div className="sidebar">
          <div className="tabs">
            <button
              className={`tab-btn${activeTab === 'forward' ? ' active' : ''}`}
              onClick={() => setActiveTab('forward')}
            >
              Forward Kinematics
            </button>
            <button
              className={`tab-btn${activeTab === 'inverse' ? ' active' : ''}`}
              onClick={() => setActiveTab('inverse')}
            >
              Inverse Kinematics
            </button>
          </div>
          <div className="tab-content">
            {activeTab === 'forward' && (
              <>
                <h2>Forward Kinematics</h2>
                <JointStateViewer />
                <JointSliders onPreviewJointsChange={setFkJoints} initialJoints={lastJointsOnTabChange.current} />
              </>
            )}
            {activeTab === 'inverse' && (
              <>
                <h2>Inverse Kinematics</h2>
                <IKViewer onCopyPose={pose => setIkPose({ ...pose, _ts: Date.now() })} />
                <IKSliders
                  ikPose={ikPose}
                  onPreviewJointsChange={setPreviewJoints}
                  onIKStatusChange={setIkStatus}
                />
              </>
            )}
          </div>
        </div>
        <div className="main-content">
          <h2>🤖 Visor 3D del robot</h2>
          <div style={{ marginBottom: '0.5em' }}>
            <label>
              <input
                type="checkbox"
                checked={showRealRobot}
                onChange={e => setShowRealRobot(e.target.checked)}
              />
              Mostrar robot real
            </label>
            <label style={{ marginLeft: '1em' }}>
              <input
                type="checkbox"
                checked={showGhostRobot}
                onChange={e => setShowGhostRobot(e.target.checked)}
              />
              Mostrar robot ghost (objetivo)
            </label>
          </div>
          <UrdfViewer
            previewJoints={effectivePreviewJoints}
            showRealRobot={showRealRobot}
            showGhostRobot={showGhostRobot}
            ikStatus={ikStatus}
          />
        </div>
      </div>
    </RosProvider>
  );
}

export default App;