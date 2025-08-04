import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { ViewportGizmo } from "three-viewport-gizmo";

const GizmoPage = () => {
  const mountRef = useRef(null);
  // Estado para posición y orientación acumulada (quaternion)
  const [pos, setPos] = useState({ x: 1, y: 1, z: 1 });
  // Inicializa con rotación aleatoria
  const [rotQuat, setRotQuat] = useState(() => {
    // Genera ángulos aleatorios en radianes
    const roll = Math.random() * 2 * Math.PI - Math.PI;
    const pitch = Math.random() * 2 * Math.PI - Math.PI;
    const yaw = Math.random() * 2 * Math.PI - Math.PI;
    const euler = new THREE.Euler(roll, pitch, yaw, 'XYZ');
    return new THREE.Quaternion().setFromEuler(euler);
  });

  useEffect(() => {
    // const width = mountRef.current.offsetWidth;
    // const height = mountRef.current.offsetHeight;

    const width = window.innerWidth;
    const height = window.innerHeight;

    // (Eliminada declaración duplicada de camera)

    let renderer, scene, camera, gizmo;
    // Si ya hay un renderer, eliminarlo
    if (mountRef.current) {
      while (mountRef.current.firstChild) {
        mountRef.current.removeChild(mountRef.current.firstChild);
      }
    }
    // Crear escena y cámara
    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
    camera.up.set(0, 0, 1);
    camera.position.set(2, 2, 2);
    renderer = new THREE.WebGLRenderer();
    renderer.setSize(width, height);
    mountRef.current.appendChild(renderer.domElement);
    // Ejes globales
    const arrowX = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 1, 0xff0000, 0.1, 0.05);
    scene.add(arrowX);
    const arrowY = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), 1, 0x00ff00, 0.1, 0.05);
    scene.add(arrowY);
    const arrowZ = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), 1, 0x0000ff, 0.1, 0.05);
    scene.add(arrowZ);
    // Plano Z=0
    const planeGeometry = new THREE.PlaneGeometry(5, 5);
    const planeMaterial = new THREE.MeshBasicMaterial({ color: 0xcccccc, side: THREE.DoubleSide, transparent: true, opacity: 0.7 });
    const plane = new THREE.Mesh(planeGeometry, planeMaterial);
    plane.position.set(0, 0, 0);
    scene.add(plane);
    // Cubo y ejes locales usando quaternion acumulado
    const controlledPos = new THREE.Vector3(pos.x, pos.y, pos.z);
    const controlledQuat = rotQuat;
    const pointGeometry = new THREE.BoxGeometry(0.12, 0.12, 0.12);
    const pointMaterial = new THREE.MeshBasicMaterial({ color: 0x222222 });
    const pointMesh = new THREE.Mesh(pointGeometry, pointMaterial);
    pointMesh.position.copy(controlledPos);
    pointMesh.quaternion.copy(controlledQuat);
    scene.add(pointMesh);
    const axisLength = 0.5;
    const axisHeadLength = 0.12;
    const axisHeadWidth = 0.07;
    const localX = new THREE.Vector3(1, 0, 0).applyQuaternion(controlledQuat);
    const arrowLocalX = new THREE.ArrowHelper(localX, controlledPos, axisLength, 0xff0000, axisHeadLength, axisHeadWidth);
    scene.add(arrowLocalX);
    const localY = new THREE.Vector3(0, 1, 0).applyQuaternion(controlledQuat);
    const arrowLocalY = new THREE.ArrowHelper(localY, controlledPos, axisLength, 0x00ff00, axisHeadLength, axisHeadWidth);
    scene.add(arrowLocalY);
    const localZ = new THREE.Vector3(0, 0, 1).applyQuaternion(controlledQuat);
    const arrowLocalZ = new THREE.ArrowHelper(localZ, controlledPos, axisLength, 0x0000ff, axisHeadLength, axisHeadWidth);
    scene.add(arrowLocalZ);
    scene.background = new THREE.Color(0xf0f0f0);
    gizmo = new ViewportGizmo(camera, renderer, { type: 'sphere' });
    gizmo.attachControls(new OrbitControls(camera, renderer.domElement));
    gizmo.target.set(0, 0, 0);
    camera.lookAt(gizmo.target);
    renderer.setAnimationLoop(() => {
      renderer.toneMapping = THREE.CineonToneMapping;
      renderer.render(scene, camera);
      renderer.toneMapping = THREE.NoToneMapping;
      gizmo.render();
    });
    const handleResize = () => {
      camera.aspect = window.innerWidth / window.innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(window.innerWidth, window.innerHeight);
      gizmo.update();
    };
    window.addEventListener('resize', handleResize);
    return () => {
      window.removeEventListener('resize', handleResize);
      if (renderer) {
        renderer.dispose();
      }
      if (mountRef.current && renderer.domElement) {
        mountRef.current.removeChild(renderer.domElement);
      }
    };
  }, [pos, rotQuat]);

  // Función para aplicar rotación incremental sobre el quaternion actual
  const applyRotation = (axis, angleRad) => {
    setRotQuat(prevQuat => {
      const q = prevQuat.clone();
      let rotAxis;
      if (axis === 'roll') rotAxis = new THREE.Vector3(1, 0, 0);
      if (axis === 'pitch') rotAxis = new THREE.Vector3(0, 1, 0);
      if (axis === 'yaw') rotAxis = new THREE.Vector3(0, 0, 1);
      rotAxis.applyQuaternion(q); // eje local actual
      const deltaQ = new THREE.Quaternion().setFromAxisAngle(rotAxis, angleRad);
      q.premultiply(deltaQ);
      return q;
    });
  };

  // Mostrar los ángulos Euler actuales (solo lectura)
  const euler = new THREE.Euler().setFromQuaternion(rotQuat, 'XYZ');

  // Función para mover el punto en el eje local
  const moveLocal = (axis, delta) => {
    setPos(prevPos => {
      const p = new THREE.Vector3(prevPos.x, prevPos.y, prevPos.z);
      const localDelta = new THREE.Vector3(0, 0, 0);
      if (axis === 'x') localDelta.x = delta;
      if (axis === 'y') localDelta.y = delta;
      if (axis === 'z') localDelta.z = delta;
      localDelta.applyQuaternion(rotQuat); // mover en el sistema local
      p.add(localDelta);
      return { x: p.x, y: p.y, z: p.z };
    });
  };

  return (
    <>
      <div style={{ position: 'absolute', zIndex: 2, left: 20, top: 20, background: 'rgba(255,255,255,0.8)', padding: 12, borderRadius: 8 }}>
        <div>Posición (ejes locales)</div>
        <div style={{ display: 'flex', flexDirection: 'column', gap: 4, marginBottom: 10 }}>
          <div>
            <strong>X:</strong> {pos.x.toFixed(2)}
            <button onClick={() => moveLocal('x', -0.01)} style={{ marginLeft: 8 }}>-</button>
            <button onClick={() => moveLocal('x', 0.01)} style={{ marginLeft: 2 }}>+</button>
          </div>
          <div>
            <strong>Y:</strong> {pos.y.toFixed(2)}
            <button onClick={() => moveLocal('y', -0.01)} style={{ marginLeft: 8 }}>-</button>
            <button onClick={() => moveLocal('y', 0.01)} style={{ marginLeft: 2 }}>+</button>
          </div>
          <div>
            <strong>Z:</strong> {pos.z.toFixed(2)}
            <button onClick={() => moveLocal('z', -0.01)} style={{ marginLeft: 8 }}>-</button>
            <button onClick={() => moveLocal('z', 0.01)} style={{ marginLeft: 2 }}>+</button>
          </div>
        </div>
        <div style={{ marginTop: 10 }}>Orientación acumulada (rad)</div>
        <div style={{ display: 'flex', flexDirection: 'column', gap: 4 }}>
          <div>
            <strong>Roll:</strong> {euler.x.toFixed(2)}
            <button onClick={() => applyRotation('roll', -0.05)} style={{ marginLeft: 8 }}>-</button>
            <button onClick={() => applyRotation('roll', 0.05)} style={{ marginLeft: 2 }}>+</button>
          </div>
          <div>
            <strong>Pitch:</strong> {euler.y.toFixed(2)}
            <button onClick={() => applyRotation('pitch', -0.05)} style={{ marginLeft: 8 }}>-</button>
            <button onClick={() => applyRotation('pitch', 0.05)} style={{ marginLeft: 2 }}>+</button>
          </div>
          <div>
            <strong>Yaw:</strong> {euler.z.toFixed(2)}
            <button onClick={() => applyRotation('yaw', -0.05)} style={{ marginLeft: 8 }}>-</button>
            <button onClick={() => applyRotation('yaw', 0.05)} style={{ marginLeft: 2 }}>+</button>
          </div>
        </div>
      </div>
      <div ref={mountRef} style={{ width: '100%', height: '100vh', margin: '0', padding: '0', position: 'absolute', top: '0', left: '0', overflow: 'hidden' }} />
    </>
  );
};

export default GizmoPage;
