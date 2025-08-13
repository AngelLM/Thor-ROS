import React, { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import { TransformControls } from "three/examples/jsm/controls/TransformControls";
import { ViewportGizmo } from "three-viewport-gizmo";

export default function ThreeTransformCube() {
  const mountRef = useRef(null);
  const sceneRef = useRef(null);
  const rendererRef = useRef(null);

  useEffect(() => {
    const mount = mountRef.current;
    if (!mount) return;

    const width = mountRef.current.offsetWidth;
    const height = mountRef.current.offsetHeight;

    const camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
    camera.up.set(0, 0, 1); // Set Z-UP system
    camera.position.set(-1, 1, 1);

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

    // const renderer = new THREE.WebGLRenderer({ antialias: true });
    // renderer.setPixelRatio(window.devicePixelRatio || 1);
    // renderer.setSize(mount.clientWidth, mount.clientHeight);
    // renderer.outputEncoding = THREE.sRGBEncoding;
    // mount.appendChild(renderer.domElement);

    // const scene = new THREE.Scene();
    // scene.background = new THREE.Color(0xf7fafc);

    // const camera = new THREE.PerspectiveCamera(
    //   50,
    //   mount.clientWidth / mount.clientHeight,
    //   0.1,
    //   1000
    // );
    // camera.position.set(3, 3, 6);

    const hemi = new THREE.HemisphereLight(0xffffff, 0x444444, 0.8);
    hemi.position.set(0, 20, 0);
    scene.add(hemi);

    const dir = new THREE.DirectionalLight(0xffffff, 0.8);
    dir.position.set(5, 10, 7.5);
    scene.add(dir);

    // const grid = new THREE.GridHelper(10, 20, 0xcccccc, 0xeeeeee);
    // scene.add(grid);

    // Gizmo
    const orbit = new OrbitControls(camera, renderer.domElement);
    const gizmo = new ViewportGizmo(camera, renderer, { type: 'sphere' });
    gizmo.attachControls(orbit);
    gizmo.target.set(0, 0, 0.3);
    camera.lookAt(gizmo.target);
    // Forced initial update of gizmo to ensure correct position
    gizmo.update(); // Force initial update of gizmo

    const geometry = new THREE.BoxGeometry(1, 1, 1);
    const material = new THREE.MeshStandardMaterial({ metalness: 0.1, roughness: 0.6 });
    const cube = new THREE.Mesh(geometry, material);
    cube.position.set(0, 0.5, 0);
    scene.add(cube);

    const axes = new THREE.AxesHelper(2);
    scene.add(axes);

    const transform = new TransformControls(camera, renderer.domElement);
    transform.attach(cube);
    transform.setMode("translate");
    transform.showX = true;
    transform.showY = true;
    transform.showZ = true;
    transform.setTranslationSnap(null);
    transform.setRotationSnap(null);
    // Bloquear escalado
    transform.setMode("translate");
    transform.setSpace("world");
    scene.add(transform.getHelper());

    const onDraggingChanged = (event) => {
      orbit.enabled = !event.value;
    };
    transform.addEventListener("dragging-changed", onDraggingChanged);

    // Log pose on change
    const onObjectChange = () => {
      cube.updateMatrixWorld(true);
      const p = new THREE.Vector3();
      const q = new THREE.Quaternion();
      cube.getWorldPosition(p);
      cube.getWorldQuaternion(q);
      console.log("Cube pose:", {
        position: { x: +p.x.toFixed(4), y: +p.y.toFixed(4), z: +p.z.toFixed(4) },
        orientation: { x: +q.x.toFixed(4), y: +q.y.toFixed(4), z: +q.z.toFixed(4), w: +q.w.toFixed(4) }
      });
    };
    transform.addEventListener("objectChange", onObjectChange);

    const onKeyDown = (e) => {
      switch (e.key.toLowerCase()) {
        case "w":
          transform.setMode("translate");
          break;
        case "e":
          transform.setMode("rotate");
          break;
        case "l":
          // Toggle local space
          transform.setSpace(transform.space === "local" ? "world" : "local");
          break;
        default:
          break;
      }
    };
    window.addEventListener("keydown", onKeyDown);

    const onResize = () => {
      camera.aspect = mount.clientWidth / mount.clientHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(mount.clientWidth, mount.clientHeight);
    };
    window.addEventListener("resize", onResize);

    let mounted = true;
    const tick = () => {
      if (!mounted) return;
      orbit.update();
      renderer.render(scene, camera);
      // Render the gizmo
      renderer.toneMapping = THREE.NoToneMapping;
      gizmo.render();
      requestAnimationFrame(tick);
    };
    tick();

    return () => {
      mounted = false;
      transform.removeEventListener("dragging-changed", onDraggingChanged);
      transform.removeEventListener("objectChange", onObjectChange);
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("resize", onResize);
      orbit.dispose();
      transform.dispose();
      geometry.dispose();
      material.dispose();
      renderer.dispose();
      if (renderer.domElement && renderer.domElement.parentNode) {
        renderer.domElement.parentNode.removeChild(renderer.domElement);
      }
    };
  }, []);

  return (
    <div className="w-full h-[70vh] rounded-2xl shadow-md overflow-hidden" style={{ minHeight: 400 }} ref={mountRef}>
      <div className="absolute top-2 left-2 z-10 space-x-2">
        <small className="text-sm text-gray-500">W: mover · E: rotar · L: alternar espacio</small>
      </div>
    </div>
  );
}
