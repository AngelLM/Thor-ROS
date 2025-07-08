import React, { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import URDFLoader from 'urdf-loader';
import ROSLIB from 'roslib';

const UrdfViewer = ({ previewJoints, showRealRobot = true, showGhostRobot = true }) => {
  const mountRef = useRef(null);
  const rendererRef = useRef(null);
  const robotRef = useRef(null); // robot real
  const ghostRef = useRef(null); // robot ghost
  const sceneRef = useRef(null);
  const urdfXmlRef = useRef(null);
  const cubeRef = useRef(null); // cubo amarillo

  // Cargar y mostrar ambos robots (real y ghost)
  useEffect(() => {
    if (!mountRef.current) return;
    const width = mountRef.current.offsetWidth;
    const height = mountRef.current.offsetHeight;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf0f0f0);
    sceneRef.current = scene;
    const camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
    camera.position.set(2, 2, 2);
    camera.lookAt(0, 0, 0);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    rendererRef.current = renderer;
    while (mountRef.current.firstChild) {
      mountRef.current.removeChild(mountRef.current.firstChild);
    }
    mountRef.current.appendChild(renderer.domElement);
    const light = new THREE.DirectionalLight(0xffffff, 1);
    light.position.set(5, 10, 7.5);
    scene.add(light);
    const ambientLight = new THREE.AmbientLight(0x404040);
    scene.add(ambientLight);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.screenSpacePanning = false;
    controls.minDistance = 0.5;
    controls.maxDistance = 10;
    controls.maxPolarAngle = Math.PI;
    // Cubo amarillo para el end effector del ghost
    const cube = new THREE.Mesh(
      new THREE.BoxGeometry(0.06, 0.06, 0.06),
      new THREE.MeshPhongMaterial({ color: 0xffeb3b })
    );
    cube.castShadow = false;
    cube.receiveShadow = false;
    scene.add(cube);
    cubeRef.current = cube;
    // Animación
    const animate = () => {
      requestAnimationFrame(animate);
      controls.update();
      // Fuerza el material azul translúcido en todos los meshes del ghost en cada frame
      if (ghostRef.current) {
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
        // --- Cubo sigue al link gripper_base (o base_gripper) ---
        let eeLink = ghostRef.current.getObjectByName('gripper_base') || ghostRef.current.getObjectByName('base_gripper');
        if (eeLink && cubeRef.current) {
          eeLink.updateWorldMatrix(true, false);
          eeLink.getWorldPosition(cubeRef.current.position);
          eeLink.getWorldQuaternion(cubeRef.current.quaternion);
        }
      }
      renderer.render(scene, camera);
    };
    animate();
    const handleResize = () => {
      if (mountRef.current && rendererRef.current) {
        const width = mountRef.current.offsetWidth;
        const height = mountRef.current.offsetHeight;
        rendererRef.current.setSize(width, height);
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
      }
    };
    window.addEventListener('resize', handleResize);
    // --- ROS y URDF ---
    const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
    ros.on('connection', () => {
      const service = new ROSLIB.Service({
        ros: ros,
        name: '/robot_state_publisher/get_parameters',
        serviceType: 'rcl_interfaces/srv/GetParameters',
      });
      const request = new ROSLIB.ServiceRequest({
        names: ['robot_description'],
      });
      service.callService(request, (result) => {
        const urdfXml = result.values[0].string_value;
        urdfXmlRef.current = urdfXml;
        const loader = new URDFLoader();
        loader.workingPath = '/thor_urdf';
        loader.fetchOptions = { mode: 'cors' };
        // Robot real
        const robot = loader.parse(urdfXml);
        robot.rotation.x = -Math.PI / 2;
        robot.rotation.z = Math.PI / 2;
        robot.scale.set(1, 1, 1);
        scene.add(robot);
        robotRef.current = robot;
        // Ghost robot
        const ghost = loader.parse(urdfXml);
        ghost.rotation.x = -Math.PI / 2;
        ghost.rotation.z = Math.PI / 2;
        ghost.scale.set(1, 1, 1);
        scene.add(ghost);
        ghostRef.current = ghost;
        // Suscripción a joint_states para robot real
        const jointStateListener = new ROSLIB.Topic({
          ros: ros,
          name: '/joint_states',
          messageType: 'sensor_msgs/msg/JointState'
        });
        jointStateListener.subscribe((message) => {
          if (!robotRef.current) return;
          for (let i = 0; i < message.name.length; i++) {
            const jointName = message.name[i];
            const position = message.position[i];
            robotRef.current.setJointValue(jointName, position);
          }
        });
      });
    });
    ros.on('error', (error) => {
      console.error('[ROS] ❌ Error de conexión ', error);
    });
    ros.on('close', () => {
      console.warn('[ROS] 🔌 Conexión cerrada');
    });
    return () => {
      window.removeEventListener('resize', handleResize);
      if (rendererRef.current) {
        rendererRef.current.dispose();
      }
      ros.close();
    };
  }, []);

  // Actualiza el ghost robot cuando cambian las articulaciones objetivo
  useEffect(() => {
    if (!ghostRef.current || !previewJoints) return;
    const jointNames = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'];
    if (previewJoints.joint_1 !== undefined) {
      jointNames.forEach(j => {
        if (previewJoints[j] !== undefined) {
          ghostRef.current.setJointValue(j, previewJoints[j]);
        }
      });
    }
    // Eliminado el else que ponía todo a 0
  }, [previewJoints]);

  // Mostrar/ocultar robots según props
  useEffect(() => {
    if (robotRef.current) robotRef.current.visible = !!showRealRobot;
    if (ghostRef.current) ghostRef.current.visible = !!showGhostRobot;
  }, [showRealRobot, showGhostRobot]);

  return (
    <div>
      <div
        ref={mountRef}
        style={{ width: '100%', height: '100vh', borderRadius: '12px', boxShadow: '0 0 20px rgba(0,0,0,0.2)', overflow: 'hidden' }}
      />
    </div>
  );
};

export default UrdfViewer;
