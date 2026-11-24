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
      return;
    }
    this.ribbonMesh.visible = true;

    // Performance adaptation: 16 segments in LOW / i3 MODE, 32 segments in HIGH / AUTO
    const isLowMode = this.perfGovernor?.profile === 'LOW';
    const activeSegments = isLowMode ? 16 : this.maxSegments;
    this.ribbonGeo.setDrawRange(0, activeSegments * 6);

    // 1. Determine Ribbon Color State
    const isAebOrEmergency = driverState.driveMode === 5 ||
                             driverState.aebActive ||
                             driverState.state === 'AEB_BRAKING';
    const isRedStop = driverState.driveMode === 6 ||
                      driverState.state === 'STOPPED' ||
                      !!perceptionScanner?.activeRedStopObstacle;
    const isLaneChanging = driverState.laneChangeActive ||
                           (driverState.currentLaneIndex !== driverState.targetLaneIndex);
    const isNudging = Math.abs(driverState.lateralNudge || 0) > 0.08;
    const isYielding = !!driverState.isYielding;

    let baseColor = this.COLOR_TESLA_BLUE;
    if (isAebOrEmergency || isRedStop) {
      baseColor = this.COLOR_RED;
    } else if (isLaneChanging || isNudging || isYielding) {
      baseColor = this.COLOR_AMBER;
    }

    // 2. Generate 3D Trajectory Ribbon Points
    const halfWidth = 0.90; // 1.8m road path ribbon
    const carY = (car.position.y || 0) + 0.04; // subtle elevation above asphalt to prevent z-fighting
    const px = car.position.x;
    const pz = car.position.z;
    const yaw = car.heading || 0;

    let hasRoutePath = false;

    if (routePlanner && routePlanner.hasActiveRoute() && routePlanner.activeRoute) {
      const route = routePlanner.activeRoute;
      const cIdx = routePlanner.currentIndex || 0;
      const len = route.length;
      if (len > cIdx) {
        hasRoutePath = true;
        const stepSkip = Math.max(1, Math.floor(len / (activeSegments * 2)));
        let posIdx = 0;
        let colorIdx = 0;

        for (let i = 0; i <= activeSegments; i++) {
          const wIdx = Math.min(len - 1, cIdx + i * stepSkip);
          const wp = route[wIdx];
          const wx = wp.x;
          const wz = wp.z;
          const wHeading = wp.heading || yaw;

          // Normal vector perpendicular to heading (right side)
          const nx = Math.cos(wHeading);
          const nz = -Math.sin(wHeading);

          // Alpha fade factor along horizon
          const t = i / activeSegments;
          const alphaFade = Math.max(0.18, 1.0 - t * 0.75);

          // Left vertex
          this.ribbonPositions[posIdx]     = wx - nx * halfWidth;
          this.ribbonPositions[posIdx + 1] = carY;
          this.ribbonPositions[posIdx + 2] = wz - nz * halfWidth;

          this.ribbonColors[colorIdx]     = baseColor.r * alphaFade;
          this.ribbonColors[colorIdx + 1] = baseColor.g * alphaFade;
          this.ribbonColors[colorIdx + 2] = baseColor.b * alphaFade;

          posIdx += 3;
          colorIdx += 3;

          // Right vertex
          this.ribbonPositions[posIdx]     = wx + nx * halfWidth;
          this.ribbonPositions[posIdx + 1] = carY;
          this.ribbonPositions[posIdx + 2] = wz + nz * halfWidth;

          this.ribbonColors[colorIdx]     = baseColor.r * alphaFade;
          this.ribbonColors[colorIdx + 1] = baseColor.g * alphaFade;
          this.ribbonColors[colorIdx + 2] = baseColor.b * alphaFade;

          posIdx += 3;
          colorIdx += 3;
        }
      }
    }

    if (!hasRoutePath) {
      // Predict Stanley steering arc ahead of the car
      const steer = car.steerAngle || 0;
      const curvature = Math.tan(steer) / 2.87; // 2.87m wheelbase
      const ds = Math.max(0.9, Math.min(1.8, 0.6 + 0.08 * (car.speed || 10)));

      let curX = px;
      let curZ = pz;
      let curH = yaw;

      let posIdx = 0;
      let colorIdx = 0;

      for (let i = 0; i <= activeSegments; i++) {
        const nx = Math.cos(curH);
        const nz = -Math.sin(curH);
        const t = i / activeSegments;
        const alphaFade = Math.max(0.18, 1.0 - t * 0.75);

        // Left vertex
        this.ribbonPositions[posIdx]     = curX - nx * halfWidth;
        this.ribbonPositions[posIdx + 1] = carY;
        this.ribbonPositions[posIdx + 2] = curZ - nz * halfWidth;

        this.ribbonColors[colorIdx]     = baseColor.r * alphaFade;
        this.ribbonColors[colorIdx + 1] = baseColor.g * alphaFade;
        this.ribbonColors[colorIdx + 2] = baseColor.b * alphaFade;

        posIdx += 3;
        colorIdx += 3;

        // Right vertex
        this.ribbonPositions[posIdx]     = curX + nx * halfWidth;
        this.ribbonPositions[posIdx + 1] = carY;
        this.ribbonPositions[posIdx + 2] = curZ + nz * halfWidth;

        this.ribbonColors[colorIdx]     = baseColor.r * alphaFade;
        this.ribbonColors[colorIdx + 1] = baseColor.g * alphaFade;
        this.ribbonColors[colorIdx + 2] = baseColor.b * alphaFade;

        posIdx += 3;
        colorIdx += 3;

        // Step forward along steering arc
        curH += curvature * ds;
        curX += Math.sin(curH) * ds;
        curZ += Math.cos(curH) * ds;
      }
    }

    this.posAttr.needsUpdate = true;
    this.colorAttr.needsUpdate = true;

    // 3. 3D Red Stop-Line Bar
    if (perceptionScanner?.activeRedStopObstacle) {
      const obs = perceptionScanner.activeRedStopObstacle;
      this.stopLineMesh.visible = true;
      this.stopLineMesh.position.set(obs.x, carY + 0.01, obs.z);
      this.stopLineMesh.rotation.y = obs.heading !== undefined ? obs.heading : yaw;
    } else if (routePlanner && routePlanner.hasActiveRoute() && routePlanner._trackingRef.distToGoal <= 28.0) {
      const lastWp = routePlanner.activeRoute[routePlanner.activeRoute.length - 1];
      this.stopLineMesh.visible = true;
      this.stopLineMesh.position.set(lastWp.x, carY + 0.01, lastWp.z);
      this.stopLineMesh.rotation.y = lastWp.heading || yaw;
    } else {
      this.stopLineMesh.visible = false;
    }

    // 4. Minimalist Lead Vehicle Perception Marker
    const leadDist = driverState.leadDistance || perceptionScanner?.leadDist || 999;
    if (leadDist < 70.0) {
      const lx = px + Math.sin(yaw) * (leadDist + 2.4);
      const lz = pz + Math.cos(yaw) * (leadDist + 2.4);
      this.leadMarkerMesh.visible = true;
      this.leadMarkerMesh.position.set(lx, carY + 0.01, lz);

      if (isAebOrEmergency) {
        this.leadMat.color.setHex(0xef4444); // Red during emergency brake
      } else {
        this.leadMat.color.setHex(0x00f0ff); // Cyan for ACC tracking
      }
    } else {
      this.leadMarkerMesh.visible = false;
    }

    // 5. Minimalist BSM Blind-Spot Vehicle Indicator
    const leftBsm = !!driverState.leftMirrorBlindSpot;
    const rightBsm = !!driverState.rightMirrorBlindSpot;
    if (leftBsm || rightBsm) {
      this.bsmMarkerMesh.visible = true;
      const sideSign = leftBsm ? -1 : 1;
      const bx = px + Math.cos(yaw) * (sideSign * 2.0);
      const bz = pz - Math.sin(yaw) * (sideSign * 2.0);
      this.bsmMarkerMesh.position.set(bx, carY + 0.01, bz);
      this.bsmMarkerMesh.rotation.y = yaw + (leftBsm ? 0 : Math.PI);
    } else {
      this.bsmMarkerMesh.visible = false;
    }
  }

