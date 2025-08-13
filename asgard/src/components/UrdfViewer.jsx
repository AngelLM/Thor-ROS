import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import { ViewportGizmo } from "three-viewport-gizmo";
import { TransformControls } from "three/examples/jsm/controls/TransformControls";
import URDFLoader from 'urdf-loader';
import ROSLIB from 'roslib';

const UrdfViewer = ({ previewJoints, showRealRobot = true, showGhostRobot = true, onGhostJointsChange, showFPS = true, showGhostRobotCoordinates = true }) => {
  const mountRef = useRef(null);
  const rendererRef = useRef(null);
  const robotRef = useRef(null); // robot real
  const ghostRef = useRef(null); // robot ghost
  const sceneRef = useRef(null);
  const urdfXmlRef = useRef(null);
  const [fps, setFps] = useState(0); // Estado para los FPS

  // Cargar y mostrar ambos robots (real y ghost)
  useEffect(() => {
    if (!mountRef.current) return;

    const width = mountRef.current.offsetWidth;
    const height = mountRef.current.offsetHeight;

    const camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
    camera.up.set(0, 0, 1); // Set Z-UP system
    camera.position.set(-1, 1, 1);
    // camera.lookAt(0, 0, 0);

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf0f0f0);
    sceneRef.current = scene;
    
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

    // Gizmo
    const orbit = new OrbitControls(camera, renderer.domElement);
    const gizmo = new ViewportGizmo(camera, renderer, { type: 'sphere' });
    gizmo.attachControls(orbit);
    gizmo.target.set(0, 0, 0.3);
    camera.lookAt(gizmo.target);

    // Forced initial update of gizmo to ensure correct position
    gizmo.update(); // Force initial update of gizmo

    // Animación con limitador de FPS efectivo
    let lastTime = 0;
    const targetFPS = 30;
    const frameInterval = 1000 / targetFPS; // 33.33ms between frames
    let frameCount = 0;

    const logFPS = setInterval(() => {
      setFps(frameCount); // Actualizar el estado con el valor actual de FPS
      frameCount = 0; // Reiniciar el contador
    }, 1000); // Cada segundo

    const animate = (currentTime) => {
      // Controlar el tiempo transcurrido para limitar a los FPS objetivo
      if (currentTime - lastTime >= frameInterval) {
        frameCount++; // Incrementar el contador de frames
        requestAnimationFrame(animate);
        // Fuerza el material azul translúcido en todos los meshes del ghost en cada frame
        if (ghostRef.current) {
          // Obtener los valores actuales de las joints del ghost robot
          const jointValues = {};
          if (ghostRef.current.joints) {
            Object.keys(ghostRef.current.joints).forEach(jointName => {
              const joint = ghostRef.current.joints[jointName];
              if (joint && joint.angle !== undefined) {
                jointValues[jointName] = joint.angle;
              }
            });
          }
          // Enviar los valores al componente padre si hay callback
          if (onGhostJointsChange) {
            onGhostJointsChange(jointValues);
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
        // Render the scene
        renderer.toneMapping = THREE.CineonToneMapping;
        renderer.render(scene, camera);

        // Render the gizmo
        renderer.toneMapping = THREE.NoToneMapping;
        gizmo.render();
        
        lastTime = currentTime;
      } else {
        // Si no ha pasado suficiente tiempo, programa la próxima verificación
        requestAnimationFrame(animate);
      }
    };

    // Iniciar el loop de animación
    requestAnimationFrame(animate);

    // Optimized renderer settings
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = false; // Disable shadows for performance

    const handleResize = () => {
      if (mountRef.current && rendererRef.current) {
        const width = mountRef.current.offsetWidth;
        const height = mountRef.current.offsetHeight;
        rendererRef.current.setSize(width, height);
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
        // Update the gizmo on resize
        gizmo.update();
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
        robot.rotation.x = 0; // Reset X rotation
        robot.rotation.z = 0; // Reset Z rotation
        robot.scale.set(1, 1, 1);
        scene.add(robot);
        robotRef.current = robot;

        // Ghost robot
        const ghost = loader.parse(urdfXml);
        ghost.rotation.x = 0; // Reset X rotation
        ghost.rotation.z = 0; // Reset Z rotation
        ghost.scale.set(1, 1, 1);
        scene.add(ghost);
        ghostRef.current = ghost;

        // Crear esfera semitransparente en el TCP del ghost
        let eeLink = ghost.getObjectByName('gripper_mid_point') || ghost.getObjectByName('base_gripper');
        if (eeLink) {
          eeLink.updateWorldMatrix(true, false);
          const pos = new THREE.Vector3();
          eeLink.getWorldPosition(pos);
          const sphere = new THREE.Mesh(
            new THREE.SphereGeometry(0.02, 32, 32),
            new THREE.MeshPhongMaterial({ color: 0x4caf50, opacity: 0.5, transparent: true })
          );
          sphere.position.copy(pos);
          scene.add(sphere);

          // Dos TransformControls: translate y rotate
          const translateControl = new TransformControls(camera, renderer.domElement);
          translateControl.attach(sphere);
          translateControl.setMode('translate');
          translateControl.setSpace('local');
          translateControl.setTranslationSnap(null);
          translateControl.setRotationSnap(null);
          translateControl.setSize(1.0);
          scene.add(translateControl.getHelper());

          const rotateControl = new TransformControls(camera, renderer.domElement);
          rotateControl.attach(sphere);
          rotateControl.setMode('rotate');
          rotateControl.setSpace('local');
          rotateControl.setTranslationSnap(null);
          rotateControl.setRotationSnap(null);
          rotateControl.setSize(0.85); // un poco más pequeño para evitar solape visual
          scene.add(rotateControl.getHelper());

          // Función compartida para enviar IK con la pose de la esfera
          const sendIKFromSphere = () => {
            sphere.updateMatrixWorld(true);
            const p = new THREE.Vector3();
            const q = new THREE.Quaternion();
            sphere.getWorldPosition(p);
            sphere.getWorldQuaternion(q);
            // Construir robot_state con las articulaciones del ghost
            const names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','gripperbase_to_armgearright'];
            const filteredNames = [];
            const filteredPositions = [];
            if (ghostRef.current && ghostRef.current.joints) {
              names.forEach(jn => {
                if (ghostRef.current.joints[jn] !== undefined) {
                  filteredNames.push(jn);
                  filteredPositions.push(ghostRef.current.joints[jn].angle);
                }
              });
            }
            let robotState = {};
            if (filteredNames.length > 0) {
              robotState = { joint_state: { name: filteredNames, position: filteredPositions } };
            }
            // Servicio IK (idéntico a IkSliders.jsx)
            const ikService = new ROSLIB.Service({
              ros: ros,
              name: '/compute_ik',
              serviceType: 'moveit_msgs/srv/GetPositionIK',
            });
            const pose = {
              header: { frame_id: 'base_link' },
              pose: {
                position: { x: p.x, y: p.y, z: p.z },
                orientation: { x: q.x, y: q.y, z: q.z, w: q.w }
              }
            };
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
            ikService.callService(req, (res) => {
              if (res && res.solution && res.solution.joint_state && res.error_code && res.error_code.val === 1) {
                res.solution.joint_state.name.forEach((name, i) => {
                  if (ghostRef.current && ghostRef.current.joints && ghostRef.current.joints[name] !== undefined) {
                    ghostRef.current.setJointValue(name, res.solution.joint_state.position[i]);
                  }
                });
                // Notificar inmediatamente al padre para actualizar sliders FK
                if (onGhostJointsChange && res.solution && res.solution.joint_state) {
                  const updated = {};
                  res.solution.joint_state.name.forEach((name, i) => {
                    updated[name] = res.solution.joint_state.position[i];
                  });
                  onGhostJointsChange(updated);
                }
                // Alcanzable -> verde
                if (sphere && sphere.material && sphere.material.color) {
                  sphere.material.color.set(0x4caf50);
                  sphere.material.opacity = 0.5; // Mantener semitransparente
                }
              } else {
                // No alcanzable -> rojo
                if (sphere && sphere.material && sphere.material.color) {
                  sphere.material.color.set(0xf44336);
                  sphere.material.opacity = 1.0; // Mantener opaco
                }
              }
            });
          };

          // Gestionar orbit y exclusión mutua durante el drag
          translateControl.addEventListener('dragging-changed', (e) => {
            orbit.enabled = !e.value;
            rotateControl.enabled = !e.value; // desactiva rotate mientras se arrastra translate
          });
          rotateControl.addEventListener('dragging-changed', (e) => {
            orbit.enabled = !e.value;
            translateControl.enabled = !e.value; // desactiva translate mientras se arrastra rotate
          });

          // Enviar IK en cambios de objeto para ambos controles
          translateControl.addEventListener('objectChange', sendIKFromSphere);
          rotateControl.addEventListener('objectChange', sendIKFromSphere);

          // Atajos R/T/L/G
          const onKeyDown = (e) => {
            switch (e.key.toLowerCase()) {
              case 'l':
                // Cambiar espacio en ambos controles
                const newSpace = translateControl.space === 'local' ? 'world' : 'local';
                translateControl.setSpace(newSpace);
                rotateControl.setSpace(newSpace);
                break;
              case 'g':
                // Colocar la esfera en el TCP del ghost
                if (ghostRef.current) {
                  let tcp = ghostRef.current.getObjectByName('gripper_mid_point') || ghostRef.current.getObjectByName('base_gripper');
                  if (tcp) {
                    tcp.updateWorldMatrix(true, false);
                    const npos = new THREE.Vector3();
                    const nquat = new THREE.Quaternion();
                    tcp.getWorldPosition(npos);
                    tcp.getWorldQuaternion(nquat);
                    sphere.position.copy(npos);
                    sphere.quaternion.copy(nquat);
                    sphere.updateMatrixWorld(true);
                  }
                }
                break;
              default:
                break;
            }
          };
          window.addEventListener('keydown', onKeyDown);
          // Limpieza
          // Puedes mejorar la limpieza si lo necesitas
        }

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
    const jointNames = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'gripperbase_to_armgearright'];
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
        style={{ width: '100%', height: '100%', margin: '0', padding: '0', position: 'absolute', top: '0', left: '0', overflow: 'hidden' }}
      />
      {showFPS && (
        <div
          style={{
            position: 'absolute',
            bottom: '5px',
            right: '10px',
            backgroundColor: 'rgba(0, 0, 0, 0.5)',
            color: 'white',
            padding: '5px',
            borderRadius: '5px',
            fontSize: '12px',
          }}
        >
          FPS: {fps}
        </div>
      )}
    </div>
  );
};

export default UrdfViewer;
