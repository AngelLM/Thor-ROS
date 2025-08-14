import React, { useEffect, useRef, useState, forwardRef, useImperativeHandle } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import { ViewportGizmo } from "three-viewport-gizmo";
import { TransformControls } from "three/examples/jsm/controls/TransformControls";
import URDFLoader from 'urdf-loader';
import ROSLIB from 'roslib';
import { useROS } from '../RosContext';

const UrdfViewer = forwardRef(({ previewJoints, showRealRobot = true, showGhostRobot = true, onGhostJointsChange, showFPS = true, showGhostRobotCoordinates = true }, ref) => {
  const mountRef = useRef(null);
  const rendererRef = useRef(null);
  const cameraRef = useRef(null);
  const robotRef = useRef(null);
  const ghostRef = useRef(null);
  const sceneRef = useRef(null);
  const urdfXmlRef = useRef(null);
  const sphereRef = useRef(null);
  const translateCtrlRef = useRef(null);
  const rotateCtrlRef = useRef(null);
  const gizmoRef = useRef(null);
  const orbitRef = useRef(null);
  const gridRef = useRef(null);
  const hemiRef = useRef(null);
  const ikServiceRef = useRef(null);
  const fkServiceRef = useRef(null);
  const jointCmdTopicRef = useRef(null);
  const lastSentJointsRef = useRef(null);
  const isDraggingTargetRef = useRef(false);
  const [fps, setFps] = useState(0);

  const { ros, connected } = useROS();

  const jointOrder = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','gripperbase_to_armgearright'];

  const equalsJoints = (a, b) => {
    if (!a || !b) return false;
    for (const name of jointOrder) {
      const va = a[name];
      const vb = b[name];
      if (va === undefined && vb === undefined) continue;
      if (va === undefined || vb === undefined) return false;
      if (Math.abs(va - vb) > 1e-4) return false; // relaxed tolerance to reduce jitter
    }
    return true;
  };

  const emitGhostIfChanged = () => {
    if (!onGhostJointsChange) return;
    const current = gatherGhostJoints();
    if (!equalsJoints(current, lastSentJointsRef.current)) {
      lastSentJointsRef.current = current;
      onGhostJointsChange(current);
      // Sync target sphere to current TCP if not being dragged
      if (!isDraggingTargetRef.current && sphereRef.current) {
        const ee = getEE();
        if (ee) {
          ee.updateWorldMatrix(true, false);
          const p = new THREE.Vector3();
          const q = new THREE.Quaternion();
          ee.getWorldPosition(p);
          ee.getWorldQuaternion(q);
          sphereRef.current.position.copy(p);
          sphereRef.current.quaternion.copy(q);
          sphereRef.current.updateMatrixWorld(true);
        }
      }
    }
  };

  const getEE = () => {
    if (!ghostRef.current) return null;
    return ghostRef.current.getObjectByName('gripper_mid_point') || ghostRef.current.getObjectByName('base_gripper');
  };

  const gatherGhostJoints = () => {
    const vals = {};
    if (ghostRef.current && ghostRef.current.joints) {
      Object.keys(ghostRef.current.joints).forEach(jn => {
        const j = ghostRef.current.joints[jn];
        if (j && j.angle !== undefined) vals[jn] = j.angle;
      });
    }
    return vals;
  };

  const getTcpPoseMeters = () => {
    const ee = getEE();
    if (!ee) return null;
    ee.updateWorldMatrix(true, false);
    const p = new THREE.Vector3();
    const q = new THREE.Quaternion();
    ee.getWorldPosition(p);
    ee.getWorldQuaternion(q);
    return { x: p.x, y: p.y, z: p.z, qx: q.x, qy: q.y, qz: q.z, qw: q.w };
  };

  const solveAndMoveToPoseInternal = (poseMm) => new Promise((resolve) => {
    if (!ros || !connected || !ikServiceRef.current) return resolve({ ok: false, error: 'ros_not_ready' });
    const { x, y, z, qx, qy, qz, qw } = poseMm || {};
    const pose = {
      header: { frame_id: 'base_link' },
      pose: {
        position: { x: (x || 0) / 1000, y: (y || 0) / 1000, z: (z || 0) / 1000 },
        orientation: { x: qx || 0, y: qy || 0, z: qz || 0, w: qw || 1 }
      }
    };
    const current = gatherGhostJoints();
    const filteredNames = [];
    const filteredPositions = [];
    jointOrder.forEach(n => { if (current[n] !== undefined) { filteredNames.push(n); filteredPositions.push(current[n]); } });
    const robotState = filteredNames.length ? { joint_state: { name: filteredNames, position: filteredPositions } } : {};
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
    ikServiceRef.current.callService(req, (res) => {
      if (res && res.solution && res.solution.joint_state && res.error_code && res.error_code.val === 1) {
      const solved = {};
      res.solution.joint_state.name.forEach((n, i) => { solved[n] = res.solution.joint_state.position[i]; });
      // Apply
      Object.keys(solved).forEach(jn => {
        if (ghostRef.current && ghostRef.current.joints && ghostRef.current.joints[jn] !== undefined) {
        ghostRef.current.setJointValue(jn, solved[jn]);
        }
      });
      emitGhostIfChanged();
      resolve({ ok: true, joints: solved });
      } else {
      resolve({ ok: false, error: 'ik_unreachable', raw: res });
      }
    });
  });

  // Imperative API
  useImperativeHandle(ref, () => ({
    getGhostState: () => {
      const joints = gatherGhostJoints();
      const tcpM = getTcpPoseMeters();
      let tcp = null;
      if (tcpM) tcp = { x: tcpM.x * 1000, y: tcpM.y * 1000, z: tcpM.z * 1000, qx: tcpM.qx, qy: tcpM.qy, qz: tcpM.qz, qw: tcpM.qw };
      return { joints, tcp };
    },
    // Compute TCP pose (in mm) for a given joints map by using the ghost model as a virtual FK solver
    getTCPFromJoints: (joints) => {
      if (!ghostRef.current) return null;
      // Save current ghost joints
      const prev = {};
      Object.keys(ghostRef.current.joints || {}).forEach(jn => { prev[jn] = ghostRef.current.joints[jn] && ghostRef.current.joints[jn].angle; });
      // Apply requested joints where available
      Object.keys(joints || {}).forEach(jn => {
        if (ghostRef.current.joints && ghostRef.current.joints[jn] !== undefined) {
          ghostRef.current.setJointValue(jn, joints[jn]);
        }
      });
      // Force update and read EE
      const eePose = getTcpPoseMeters();
      // Restore previous joints
      Object.keys(prev).forEach(jn => {
        if (prev[jn] !== undefined && ghostRef.current.joints && ghostRef.current.joints[jn] !== undefined) {
          ghostRef.current.setJointValue(jn, prev[jn]);
        }
      });
      // Prevent emitting ghost change due to this temporary set by resetting lastSentJointsRef
      try { lastSentJointsRef.current = gatherGhostJoints(); } catch (e) {}
      if (!eePose) return null;
      return { x: eePose.x * 1000, y: eePose.y * 1000, z: eePose.z * 1000, qx: eePose.qx, qy: eePose.qy, qz: eePose.qz, qw: eePose.qw };
    },
    setGhostJoints: (joints) => {
      if (!ghostRef.current || !joints) return;
      Object.keys(joints).forEach(jn => {
        if (ghostRef.current.joints && ghostRef.current.joints[jn] !== undefined) {
          ghostRef.current.setJointValue(jn, joints[jn]);
        }
      });
      emitGhostIfChanged();
    },
    solveAndMoveToPose: (poseMm, opts = {}) => solveAndMoveToPoseInternal(poseMm, opts),
    publishGhostToController: () => {
      if (!ros || !connected || !jointCmdTopicRef.current) return;
      const current = gatherGhostJoints();
      // Always send all joints, including gripper, and invert gripper sign
      const data = jointOrder.map(n => {
        if (n === 'gripperbase_to_armgearright') {
          // Controller expects negative radians for gripper
          return current[n] !== undefined ? -Math.abs(current[n]) : 0;
        }
        return current[n] !== undefined ? current[n] : 0;
      });
      const msg = new ROSLIB.Message({ data });
      jointCmdTopicRef.current.publish(msg);
    },
    snapTargetToTCP: () => {
      const ee = getEE();
      if (!ee || !sphereRef.current) return;
      ee.updateWorldMatrix(true, false);
      const p = new THREE.Vector3();
      const q = new THREE.Quaternion();
      ee.getWorldPosition(p);
      ee.getWorldQuaternion(q);
      sphereRef.current.position.copy(p);
      sphereRef.current.quaternion.copy(q);
      sphereRef.current.updateMatrixWorld(true);
    },
    syncTargetToTCP: () => {
      const ee = getEE();
      if (!ee || !sphereRef.current) return;
      ee.updateWorldMatrix(true, false);
      const p = new THREE.Vector3();
      const q = new THREE.Quaternion();
      ee.getWorldPosition(p);
      ee.getWorldQuaternion(q);
      sphereRef.current.position.copy(p);
      sphereRef.current.quaternion.copy(q);
      sphereRef.current.updateMatrixWorld(true);
    }
  }), [connected]);

  // Init scene
  useEffect(() => {
    if (!mountRef.current) return;

    const width = mountRef.current.offsetWidth;
    const height = mountRef.current.offsetHeight;

    const camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
    camera.up.set(0, 0, 1);
    camera.position.set(-1, 1, 1);
    cameraRef.current = camera;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf0f0f0);
    sceneRef.current = scene;
    // Grid on Z=0 (XY plane) since this app uses Z as up
    const grid = new THREE.GridHelper(4, 40, 0x888888, 0xcccccc);
    // GridHelper is XZ by default; rotate to XY plane
    grid.rotation.x = Math.PI / 2;
    grid.position.set(0, 0, 0);
    if (grid.material) {
      grid.material.opacity = 0.5;
      grid.material.transparent = true;
    }
    scene.add(grid);
    gridRef.current = grid;
    
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    rendererRef.current = renderer;
    while (mountRef.current.firstChild) mountRef.current.removeChild(mountRef.current.firstChild);
    mountRef.current.appendChild(renderer.domElement);

    const light = new THREE.DirectionalLight(0xffffff, 0.6);
    light.position.set(-5, 10, 7.5);
    scene.add(light);
    const ambientLight = new THREE.AmbientLight(0x404040);
    scene.add(ambientLight);
    // Hemisphere light for soft sky/ground illumination
    const hemi = new THREE.HemisphereLight(0xffffbb, 0x080820, 0.2);
    hemi.position.set(0, 0, 10);
    scene.add(hemi);
    hemiRef.current = hemi;

    const orbit = new OrbitControls(camera, renderer.domElement);
    orbitRef.current = orbit;
    const gizmo = new ViewportGizmo(camera, renderer, { type: 'sphere' });
    gizmo.attachControls(orbit);
    gizmo.target.set(0, 0, 0.3);
    camera.lookAt(gizmo.target);
    gizmo.update();
    gizmoRef.current = gizmo;

    // Animación con limitador de FPS efectivo
    let lastTime = 0;
    const targetFPS = 30;
    const frameInterval = 1000 / targetFPS;
    let frameCount = 0;

    const logFPS = setInterval(() => { setFps(frameCount); frameCount = 0; }, 1000);

    const animate = (currentTime) => {
      if (currentTime - lastTime >= frameInterval) {
        frameCount++;
        requestAnimationFrame(animate);
        if (ghostRef.current) {
          emitGhostIfChanged();
          // Keep sphere following TCP when not dragging
          if (!isDraggingTargetRef.current && sphereRef.current) {
            const ee = getEE();
            if (ee) {
              ee.updateWorldMatrix(true, false);
              const p = new THREE.Vector3();
              const q = new THREE.Quaternion();
              ee.getWorldPosition(p);
              ee.getWorldQuaternion(q);
              sphereRef.current.position.copy(p);
              sphereRef.current.quaternion.copy(q);
              sphereRef.current.updateMatrixWorld(true);
            }
          }
          ghostRef.current.traverse(obj => {
            if (obj.isMesh) {
              if (Array.isArray(obj.material)) {
                obj.material = obj.material.map(() => new THREE.MeshPhongMaterial({ color: 0x2196f3, opacity: 0.4, transparent: true, depthWrite: false }));
              } else {
                obj.material = new THREE.MeshPhongMaterial({ color: 0x2196f3, opacity: 0.4, transparent: true, depthWrite: false });
              }
              obj.material.needsUpdate = true;
              obj.material.vertexColors = false;
            }
          });
        }
        renderer.toneMapping = THREE.CineonToneMapping;
        renderer.render(scene, camera);
        renderer.toneMapping = THREE.NoToneMapping;
        gizmo.render();
        lastTime = currentTime;
      } else {
        requestAnimationFrame(animate);
      }
    };
    requestAnimationFrame(animate);

    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = false;

    const handleResize = () => {
      if (mountRef.current && rendererRef.current) {
        const width = mountRef.current.offsetWidth;
        const height = mountRef.current.offsetHeight;
        rendererRef.current.setSize(width, height);
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
        gizmo.update();
      }
    };
    window.addEventListener('resize', handleResize);

    return () => {
      window.removeEventListener('resize', handleResize);
  if (rendererRef.current) rendererRef.current.dispose();
  if (gridRef.current && sceneRef.current) sceneRef.current.remove(gridRef.current);
  if (hemiRef.current && sceneRef.current) sceneRef.current.remove(hemiRef.current);
  clearInterval(logFPS);
    };
  }, []);

  // Load URDF and ROS services
  useEffect(() => {
    if (!ros || !connected || !sceneRef.current || !cameraRef.current || !rendererRef.current) return;

    ikServiceRef.current = new ROSLIB.Service({ ros, name: '/compute_ik', serviceType: 'moveit_msgs/srv/GetPositionIK' });
    fkServiceRef.current = new ROSLIB.Service({ ros, name: '/compute_fk', serviceType: 'moveit_msgs/srv/GetPositionFK' });
    jointCmdTopicRef.current = new ROSLIB.Topic({ ros, name: '/joint_group_position_controller/command', messageType: 'std_msgs/Float64MultiArray' });

    const srv = new ROSLIB.Service({ ros, name: '/robot_state_publisher/get_parameters', serviceType: 'rcl_interfaces/srv/GetParameters' });
    const req = new ROSLIB.ServiceRequest({ names: ['robot_description'] });

    srv.callService(req, (result) => {
      const urdfXml = result.values && result.values[0] ? result.values[0].string_value : null;
      if (!urdfXml) return;
      urdfXmlRef.current = urdfXml;
      const loader = new URDFLoader();
      loader.workingPath = '/thor_urdf';
      loader.fetchOptions = { mode: 'cors' };
      // Real robot
      const robot = loader.parse(urdfXml);
      robot.rotation.x = 0;
      robot.rotation.z = 0;
      robot.scale.set(1, 1, 1);
      sceneRef.current.add(robot);
      robotRef.current = robot;
      // Ghost robot
      const ghost = loader.parse(urdfXml);
      ghost.rotation.x = 0;
      ghost.rotation.z = 0;
      ghost.scale.set(1, 1, 1);
      sceneRef.current.add(ghost);
      ghostRef.current = ghost;

      // Sphere at TCP
      const eeLink = getEE();
      if (eeLink) {
        eeLink.updateWorldMatrix(true, false);
        const pos = new THREE.Vector3();
        eeLink.getWorldPosition(pos);
        const sphere = new THREE.Mesh(new THREE.SphereGeometry(0.02, 32, 32), new THREE.MeshPhongMaterial({ color: 0x4caf50, opacity: 0.5, transparent: true }));
        sphere.position.copy(pos);
        sceneRef.current.add(sphere);
        sphereRef.current = sphere;

        // TransformControls
        const translateControl = new TransformControls(cameraRef.current, rendererRef.current.domElement);
        translateControl.attach(sphere);
        translateControl.setMode('translate');
        translateControl.setSpace('local');
        translateControl.setTranslationSnap(null);
        translateControl.setRotationSnap(null);
        translateControl.setSize(1.0);
        sceneRef.current.add(translateControl.getHelper());
        translateCtrlRef.current = translateControl;

        const rotateControl = new TransformControls(cameraRef.current, rendererRef.current.domElement);
        rotateControl.attach(sphere);
        rotateControl.setMode('rotate');
        rotateControl.setSpace('local');
        rotateControl.setTranslationSnap(null);
        rotateControl.setRotationSnap(null);
        rotateControl.setSize(0.85);
        sceneRef.current.add(rotateControl.getHelper());
        rotateCtrlRef.current = rotateControl;

        const orbit = orbitRef.current;
        translateControl.addEventListener('dragging-changed', (e) => { if (orbit) orbit.enabled = !e.value; if (rotateControl) rotateControl.enabled = !e.value; isDraggingTargetRef.current = e.value; });
        rotateControl.addEventListener('dragging-changed', (e) => { if (orbit) orbit.enabled = !e.value; if (translateControl) translateControl.enabled = !e.value; isDraggingTargetRef.current = e.value; });

        const sendIKFromSphere = () => {
          if (!sphereRef.current) return;
          if (sendIKFromSphere.__locked) return;
          sendIKFromSphere.__locked = true;
          setTimeout(() => { sendIKFromSphere.__locked = false; }, 50); // throttle 20Hz
          sphereRef.current.updateMatrixWorld(true);
          const p = new THREE.Vector3();
          const q = new THREE.Quaternion();
          sphereRef.current.getWorldPosition(p);
          sphereRef.current.getWorldQuaternion(q);
          const poseMm = { x: p.x * 1000, y: p.y * 1000, z: p.z * 1000, qx: q.x, qy: q.y, qz: q.z, qw: q.w };
          solveAndMoveToPoseInternal(poseMm).then((res) => {
            if (res && res.ok) {
              // Empujar joints inmediatamente para sincronizar UI aguas arriba
              emitGhostIfChanged();
            }
          });
        };
        translateControl.addEventListener('objectChange', sendIKFromSphere);
        rotateControl.addEventListener('objectChange', sendIKFromSphere);

        const onKeyDown = (e) => {
          switch (e.key.toLowerCase()) {
            case 'l': {
              const newSpace = translateControl.space === 'local' ? 'world' : 'local';
              translateControl.setSpace(newSpace);
              rotateControl.setSpace(newSpace);
              break;
            }
            case 'g': {
              const ee = getEE();
              if (!ee) break;
              ee.updateWorldMatrix(true, false);
              const npos = new THREE.Vector3();
              const nquat = new THREE.Quaternion();
              ee.getWorldPosition(npos);
              ee.getWorldQuaternion(nquat);
              sphere.position.copy(npos);
              sphere.quaternion.copy(nquat);
              sphere.updateMatrixWorld(true);
              break;
            }
            default: break;
          }
        };
        window.addEventListener('keydown', onKeyDown);
      }

      // Real robot joint_states subscription
      const jointStateListener = new ROSLIB.Topic({ ros, name: '/joint_states', messageType: 'sensor_msgs/msg/JointState' });
      jointStateListener.subscribe((message) => {
        if (!robotRef.current) return;
        for (let i = 0; i < message.name.length; i++) {
          const jointName = message.name[i];
          const position = message.position[i];
          robotRef.current.setJointValue(jointName, position);
        }
      });
    });
  }, [ros, connected]);

  // Apply preview joints -> ghost
  useEffect(() => {
    if (!ghostRef.current || !previewJoints) return;
    // Only apply if different from current ghost to avoid flip-flop
    const current = gatherGhostJoints();
    if (equalsJoints(previewJoints, current)) return;
    if (previewJoints.joint_1 !== undefined) {
      jointOrder.forEach(j => {
        if (previewJoints[j] !== undefined) ghostRef.current.setJointValue(j, previewJoints[j]);
      });
      // Do NOT emit back to parent here to avoid feedback loops
      // emitGhostIfChanged();
    }
  }, [previewJoints]);

  // Visibility
  useEffect(() => {
    if (robotRef.current) robotRef.current.visible = !!showRealRobot;
    if (ghostRef.current) ghostRef.current.visible = !!showGhostRobot;
  }, [showRealRobot, showGhostRobot]);

  // Controlar visibilidad de la esfera y los TransformControls
  useEffect(() => {
    if (sphereRef.current) sphereRef.current.visible = !!showGhostRobotCoordinates;
    // Hide TransformControls helpers and disable controls
    if (translateCtrlRef.current) {
      translateCtrlRef.current.getHelper().visible = !!showGhostRobotCoordinates;
      translateCtrlRef.current.enabled = !!showGhostRobotCoordinates;
    }
    if (rotateCtrlRef.current) {
      rotateCtrlRef.current.getHelper().visible = !!showGhostRobotCoordinates;
      rotateCtrlRef.current.enabled = !!showGhostRobotCoordinates;
    }
  }, [showGhostRobotCoordinates]);

  return (
    <div style={{ width: '100%', height: '100%', position: 'relative', margin: 0, padding: 0, overflow: 'hidden' }}>
      <div ref={mountRef} style={{ width: '100%', height: '100%', margin: 0, padding: 0, position: 'relative', overflow: 'hidden' }} />
      {showFPS && (
        <div style={{ position: 'absolute', bottom: '10px', left: '10px', backgroundColor: 'rgba(0, 0, 0, 0.5)', color: 'white', padding: '5px', borderRadius: '5px', fontSize: '12px' }}>
          FPS: {fps}
        </div>
      )}
    </div>
  );
});

export default UrdfViewer;
