import React, { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import URDFLoader from 'urdf-loader';
import ROSLIB from 'roslib';

const UrdfViewer = () => {
  const mountRef = useRef(null);
  const rendererRef = useRef(null);
  const robotRef = useRef(null);

  useEffect(() => {
    if (!mountRef.current) return;

    const width = mountRef.current.offsetWidth;
    const height = mountRef.current.offsetHeight;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf0f0f0);

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

    const animate = () => {
      requestAnimationFrame(animate);
      controls.update();
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

    const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

    ros.on('connection', () => {
      console.log('[ROS] 🔌 Conectado');

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

        const loader = new URDFLoader();
        loader.workingPath = '/thor_urdf';
        loader.fetchOptions = { mode: 'cors' };

        const robot = loader.parse(urdfXml);
        robot.rotation.x = -Math.PI / 2;
        robot.rotation.z = Math.PI / 2;
        robot.scale.set(1, 1, 1);
        scene.add(robot);
        robotRef.current = robot;

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

  return (
    <div
      ref={mountRef}
      style={{ width: '100%', height: '100vh', borderRadius: '12px', boxShadow: '0 0 20px rgba(0,0,0,0.2)', overflow: 'hidden' }}
    />
  );
};

export default UrdfViewer;
