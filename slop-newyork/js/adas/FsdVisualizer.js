// js/adas/FsdVisualizer.js
// ============================================================================
// Tesla FSD-Style 3D Path Ribbon & Minimalist Perception Overlay
// Ultra-lightweight: 1-3 draw calls total, pre-allocated THREE.BufferGeometry
// In-place buffer updates with ZERO GC in the 60 Hz animation loop.
// Dynamic 16-segment optimization in LOW / i3 MODE for solid 60 FPS on Intel HD.
// ============================================================================

import * as THREE from 'three/webgpu';

export class FsdVisualizer {
  constructor(scene = null, options = {}) {
    this.scene = scene;
    this.perfGovernor = options.perfGovernor || null;
    this.enabled = true;
    this.maxSegments = 32;

    // Palette: Tesla FSD Glow Colors
    this.COLOR_TESLA_BLUE = { r: 0.0, g: 0.72, b: 1.0 };  // #00b8ff
    this.COLOR_AMBER      = { r: 0.98, g: 0.62, b: 0.05 }; // #f59e0b
    this.COLOR_RED        = { r: 0.95, g: 0.15, b: 0.15 }; // #ef4444

    // 1. Dynamic 3D Trajectory Ribbon (Draw Call #1)
    // Pre-allocate typed arrays for maximum 32 segments (66 vertices)
    const vertCount = (this.maxSegments + 1) * 2;
    this.ribbonPositions = new Float32Array(vertCount * 3);
    this.ribbonColors = new Float32Array(vertCount * 3);
    this.ribbonIndices = new Uint16Array(this.maxSegments * 6);

    for (let i = 0; i < this.maxSegments; i++) {
      const i0 = i * 2;
      const i1 = i * 2 + 1;
      const i2 = (i + 1) * 2;
      const i3 = (i + 1) * 2 + 1;
      const base = i * 6;
      this.ribbonIndices[base]     = i0;
      this.ribbonIndices[base + 1] = i1;
      this.ribbonIndices[base + 2] = i2;
      this.ribbonIndices[base + 3] = i2;
      this.ribbonIndices[base + 4] = i1;
      this.ribbonIndices[base + 5] = i3;
    }

    this.ribbonGeo = new THREE.BufferGeometry();
    this.posAttr = new THREE.BufferAttribute(this.ribbonPositions, 3);
    this.colorAttr = new THREE.BufferAttribute(this.ribbonColors, 3);
    this.posAttr.setUsage(THREE.DynamicDrawUsage);
    this.colorAttr.setUsage(THREE.DynamicDrawUsage);
    this.ribbonGeo.setAttribute('position', this.posAttr);
    this.ribbonGeo.setAttribute('color', this.colorAttr);
    this.ribbonGeo.setIndex(new THREE.BufferAttribute(this.ribbonIndices, 1));

    // Zero-shadow, double-sided glowing unlit ribbon
    const MatClass = THREE.MeshBasicNodeMaterial || THREE.MeshBasicMaterial;
    this.ribbonMat = new MatClass({
      transparent: true,
      opacity: 0.68,
      depthWrite: false,
      side: THREE.DoubleSide,
      vertexColors: true
    });
    this.ribbonMesh = new THREE.Mesh(this.ribbonGeo, this.ribbonMat);
    this.ribbonMesh.frustumCulled = false;
    this.ribbonMesh.castShadow = false;
    this.ribbonMesh.receiveShadow = false;
    this.ribbonMesh.renderOrder = 998;

    // 2. 3D Red Stop-Line Bar (Draw Call #2, only when active)
    const stopGeo = new THREE.PlaneGeometry(3.6, 0.45);
    stopGeo.rotateX(-Math.PI / 2);
    this.stopMat = new MatClass({
      color: 0xef4444,
      transparent: true,
      opacity: 0.90,
      depthWrite: false,
      side: THREE.DoubleSide
    });
    this.stopLineMesh = new THREE.Mesh(stopGeo, this.stopMat);
    this.stopLineMesh.visible = false;
    this.stopLineMesh.renderOrder = 999;
    this.stopLineMesh.castShadow = false;
    this.stopLineMesh.receiveShadow = false;

    // 3. Minimalist Perception Highlights (Reused floor-ring markers)
    // 3a. Lead Vehicle Marker
    const leadGeo = new THREE.RingGeometry(1.5, 1.85, 24);
    leadGeo.rotateX(-Math.PI / 2);
    this.leadMat = new MatClass({
      color: 0x00f0ff,
      transparent: true,
      opacity: 0.85,
      depthWrite: false,
      side: THREE.DoubleSide
    });
    this.leadMarkerMesh = new THREE.Mesh(leadGeo, this.leadMat);
    this.leadMarkerMesh.visible = false;
    this.leadMarkerMesh.renderOrder = 997;
    this.leadMarkerMesh.castShadow = false;
    this.leadMarkerMesh.receiveShadow = false;

    // 3b. BSM Blind-Spot Indicator Arc
    const bsmGeo = new THREE.RingGeometry(2.0, 2.45, 16, 1, 0, Math.PI);
    bsmGeo.rotateX(-Math.PI / 2);
    this.bsmMat = new MatClass({
      color: 0xef4444,
      transparent: true,
      opacity: 0.85,
      depthWrite: false,
      side: THREE.DoubleSide
    });
    this.bsmMarkerMesh = new THREE.Mesh(bsmGeo, this.bsmMat);
    this.bsmMarkerMesh.visible = false;
    this.bsmMarkerMesh.renderOrder = 997;
    this.bsmMarkerMesh.castShadow = false;
    this.bsmMarkerMesh.receiveShadow = false;

    if (this.scene) {
      this.scene.add(this.ribbonMesh);
      this.scene.add(this.stopLineMesh);
      this.scene.add(this.leadMarkerMesh);
      this.scene.add(this.bsmMarkerMesh);
    }
  }

  update(car, driverState = {}, routePlanner = null, perceptionScanner = null) {
    if (!this.enabled || !car || !car.position) {
      this.setVisible(false);
