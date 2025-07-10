import React, { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { ViewportGizmo } from "three-viewport-gizmo";

const GizmoPage = () => {
  const mountRef = useRef(null);

  useEffect(() => {
    // const width = mountRef.current.offsetWidth;
    // const height = mountRef.current.offsetHeight;

    const width = window.innerWidth;
    const height = window.innerHeight;

    const camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
    camera.position.set(2, 2, 2);

    const scene = new THREE.Scene();
    
    const animate = () => {
      requestAnimationFrame(animate);

      // Render the scene
      renderer.toneMapping = THREE.CineonToneMapping;
      renderer.render(scene, camera);

      // Render the gizmo
      renderer.toneMapping = THREE.NoToneMapping;
      gizmo.render();
    };

    const renderer = new THREE.WebGLRenderer();
    renderer.setSize(width, height);
    renderer.setAnimationLoop(animate);
    mountRef.current.appendChild(renderer.domElement);

    const geometry = new THREE.BoxGeometry();
    const material = new THREE.MeshBasicMaterial({ color: 0x0000ff });
    const cube = new THREE.Mesh(geometry, material);
    scene.add(cube);

    const gizmo = new ViewportGizmo(camera, renderer, { type: 'sphere' });
    gizmo.attachControls(new OrbitControls(camera, renderer.domElement));
    gizmo.target.set(0, 0, 0);
    camera.lookAt(gizmo.target);


    animate();

    const handleResize = () => {
      camera.aspect = window.innerWidth / window.innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(window.innerWidth, window.innerHeight);

      // Update the gizmo on resize
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
  }, []);

  return <div ref={mountRef} style={{ width: '100%', height: '100vh', margin: '0', padding: '0', position: 'absolute', top: '0', left: '0', overflow: 'hidden' }} />;
};

export default GizmoPage;
