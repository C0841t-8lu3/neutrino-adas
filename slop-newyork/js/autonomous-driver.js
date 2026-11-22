// js/autonomous-driver.js
// ============================================================================
// QNX 8.0 NEUTRINO RTOS - ADAS LEVEL 3 AUTONOMOUS DRIVING & SAFETY SYSTEM
// Real-World Simulation Features:
// - Real-Time Dual Side-View Mirrors (Picture-in-Picture Perspective)
// - Active Blind-Spot Monitoring (BSM) Radar LEDs
// - Accident Prevention Lane-Change Abort Supervisor
// - Amber Turn Signals on Car Fascias & Mirror Indicator Frames
// - Dual-Tone Acoustic Automotive Horn
// - 60 Hz Bidirectional WebSocket Telemetry Bridge to QNX 8.0 VM
// ============================================================================

import { signalPhase } from './city-config.js';

export class AutonomousDriver {
  constructor(controllerManager, trafficSystem, scene = null) {
    this.controllerManager = controllerManager;
    this.trafficSystem = trafficSystem;
    this.scene = scene;

    this.enabled = true;
    this.state = 'CRUISING';
    this.statusMessage = 'Connecting to QNX Neutrino RTOS (SDP 8.0)...';
    this.qnxConnected = false;
    this.lastPacketTime = 0;
    this.rttMs = 0;

    // Navigation and Kinematics
    this.targetSpeedMph = 32;
    this.targetSpeed = this.targetSpeedMph * 0.44704;
    this.minSafeGap = 6.0;
    this.timeHeadway = 1.4;
    this.currentLaneIndex = 1; // 0: Left, 1: Center, 2: Right
    this.targetLaneIndex = 1;

    // Radar Perception Readings
    this.leadDist = 999;
    this.leadSpeed = 0;
    this.leadRelSpeed = 0;
    this.ttc = 999;
    this.closestObstacleName = null;
    this.leftBsmAlert = false;
    this.rightBsmAlert = false;
    this.leftBlinker = false;
    this.rightBlinker = false;
    this.hazardLights = false;
    this.hornActive = false;

    // Actuator Outputs from QNX RTOS
    this.throttle = 0;
    this.brake = 0;
    this.steerInput = 0;

    // Audio Context for Horn
    this.audioCtx = null;
    this.lastHornPlayTime = 0;

    // DOM & Canvas elements
    this.hudElement = null;
    this.leftMirrorCanvas = null;
    this.rightMirrorCanvas = null;
    this.leftMirrorCtx = null;
    this.rightMirrorCtx = null;

    this.createHudAndMirrors();
    this.bindKeyboard();
    this.initWebSocket();
  }

  // Initialize Web Audio API for authentic dual-tone car horn (410 Hz low horn + 505 Hz high horn)
  initAudio() {
    if (!this.audioCtx) {
      const AudioContextClass = window.AudioContext || window.webkitAudioContext;
      if (AudioContextClass) {
        this.audioCtx = new AudioContextClass();
      }
    }
    if (this.audioCtx && this.audioCtx.state === 'suspended') {
      this.audioCtx.resume();
    }
  }

  playHornSound(duration = 0.35) {
    try {
      this.initAudio();
      if (!this.audioCtx) return;

      const now = this.audioCtx.currentTime;
      if (now - this.lastHornPlayTime < 0.25) return;
      this.lastHornPlayTime = now;

      // European/American dual-reed car horn frequencies
      const freq1 = 410; // F4
      const freq2 = 505; // B4

      const osc1 = this.audioCtx.createOscillator();
      const osc2 = this.audioCtx.createOscillator();
      const gain = this.audioCtx.createGain();
      const filter = this.audioCtx.createBiquadFilter();

      osc1.type = 'sawtooth';
      osc2.type = 'sawtooth';
      osc1.frequency.setValueAtTime(freq1, now);
      osc2.frequency.setValueAtTime(freq2, now);

      filter.type = 'bandpass';
      filter.frequency.setValueAtTime(1400, now);
      filter.Q.setValueAtTime(2.2, now);

      gain.gain.setValueAtTime(0.01, now);
      gain.gain.exponentialRampToValueAtTime(0.45, now + 0.03);
      gain.gain.setValueAtTime(0.40, now + duration - 0.06);
      gain.gain.exponentialRampToValueAtTime(0.001, now + duration);

      osc1.connect(filter);
      osc2.connect(filter);
      filter.connect(gain);
      gain.connect(this.audioCtx.destination);

      osc1.start(now);
      osc2.start(now);
      osc1.stop(now + duration);
      osc2.stop(now + duration);
    } catch (e) {
      console.warn('[Audio] Horn sound failed:', e);
    }
  }

  // WebSocket Telemetry Link to QNX RTOS Bridge on Rust Host (port 8089)
  initWebSocket() {
    const wsUrl = 'ws://127.0.0.1:8089/';
    try {
      this.ws = new WebSocket(wsUrl);

      this.ws.onopen = () => {
        this.qnxConnected = true;
        this.statusMessage = 'QNX 8.0 RTOS Safety Kernel Online (60Hz Socket)';
        console.log('[QNX Link] WebSocket Bridge Connected to QNX RTOS on VM!');
      };

      this.ws.onmessage = (event) => {
        try {
          const res = JSON.parse(event.data);
          this.applyQnxResponse(res);
        } catch (e) {
          console.warn('[QNX Link] JSON parse error:', e);
        }
      };

      this.ws.onclose = () => {
        this.qnxConnected = false;
        this.statusMessage = 'QNX Host Disconnected. Reconnecting...';
        setTimeout(() => this.initWebSocket(), 2000);
      };

      this.ws.onerror = () => {
        this.qnxConnected = false;
      };
    } catch (e) {
      console.warn('[QNX Link] WebSocket init error:', e);
      setTimeout(() => this.initWebSocket(), 2000);
    }
  }

  applyQnxResponse(res) {
    if (!res) return;

    this.throttle = res.throttle || 0;
    this.brake = res.brake || 0;
    this.steerInput = res.steer_cmd || 0;
    this.leftBlinker = !!res.left_turn_signal;
    this.rightBlinker = !!res.right_turn_signal;
    this.hazardLights = !!res.hazard_lights;
    this.leftBsmAlert = !!res.left_mirror_blind_spot;
    this.rightBsmAlert = !!res.right_mirror_blind_spot;
    this.leadDist = (res.lead_distance !== undefined) ? res.lead_distance : 999;
    this.leadRelSpeed = res.lead_rel_speed || 0;
    this.ttc = (res.time_to_collision !== undefined) ? res.time_to_collision : 999;

    if (res.status_message) {
      this.statusMessage = res.status_message;
    }

    // Drive mode string mapping
    const MODES = ['MANUAL', 'CRUISING', 'ACC_FOLLOWING', 'LANE_CHANGING', 'INTERSECTION_TURN', 'AEB_BRAKING', 'STOPPED'];
    this.state = MODES[res.drive_mode] || 'CRUISING';

    // Acoustic horn trigger from QNX RTOS
    if (res.horn_active) {
      this.hornActive = true;
      this.playHornSound(0.32);
    } else {
      this.hornActive = false;
    }
  }

  // Request Lane Change with QNX Safety Verification
  requestLaneChange(dir) {
    // dir: -1 (left), +1 (right)
    this.initAudio();
    const newLane = Math.max(0, Math.min(2, this.currentLaneIndex + dir));
    this.targetLaneIndex = newLane;
    console.log(`[QNX Lane Change Request] Target Lane: ${newLane} (Current: ${this.currentLaneIndex})`);
  }

  toggle() {
    this.initAudio();
    this.enabled = !this.enabled;
    this.updateHud();
    if (!this.enabled) {
      this.controllerManager.keys.forward = false;
      this.controllerManager.keys.backward = false;
      this.controllerManager.keys.left = false;
      this.controllerManager.keys.right = false;
    }
  }

  bindKeyboard() {
    window.addEventListener('keydown', (e) => {
      this.initAudio();
      if (e.code === 'KeyP') {
        this.toggle();
      } else if (e.code === 'KeyQ' || e.code === 'ArrowLeft') {
        this.requestLaneChange(-1); // Left lane change
      } else if (e.code === 'KeyE' || e.code === 'ArrowRight') {
        this.requestLaneChange(+1); // Right lane change
      } else if (e.code === 'KeyH') {
        this.playHornSound(0.4);
      }
    });
  }

  // Create Mirror Canvases and Dashboard HUD
  createHudAndMirrors() {
    // 1. LEFT SIDE-VIEW MIRROR (Top-Left Dock)
    const leftMirrorBox = document.createElement('div');
    leftMirrorBox.id = 'mirror-left-box';
    leftMirrorBox.style.position = 'fixed';
    leftMirrorBox.style.top = '72px';
    leftMirrorBox.style.left = '20px';
    leftMirrorBox.style.zIndex = '990';
    leftMirrorBox.style.width = '210px';
    leftMirrorBox.style.height = '115px';
    leftMirrorBox.style.borderRadius = '14px';
    leftMirrorBox.style.overflow = 'hidden';
    leftMirrorBox.style.border = '2px solid #334155';
    leftMirrorBox.style.background = '#0a0f1d';
    leftMirrorBox.style.boxShadow = '0 10px 30px rgba(0,0,0,0.8)';
    leftMirrorBox.style.pointerEvents = 'none';

    leftMirrorBox.innerHTML = `
      <canvas id="mirror-left-canvas" width="210" height="115" style="display:block; width:100%; height:100%;"></canvas>
      <div id="bsm-left-led" style="position:absolute; top:8px; left:10px; display:flex; align-items:center; gap:4px; padding:3px 6px; border-radius:6px; background:rgba(15,23,42,0.85); border:1px solid #475569; transition:all 0.15s;">
        <span id="bsm-left-icon" style="font-size:12px; color:#64748b;">🚗⚡</span>
        <span id="bsm-left-text" style="font-size:10px; font-weight:800; font-family:monospace; color:#64748b;">BSM L</span>
      </div>
      <div id="turn-left-indicator" style="position:absolute; bottom:6px; left:10px; font-size:14px; font-weight:900; color:#f59e0b; opacity:0; transition:opacity 0.1s;">
        ◀ ◀
      </div>
    `;
    document.body.appendChild(leftMirrorBox);

    // 2. RIGHT SIDE-VIEW MIRROR (Top-Right Dock)
    const rightMirrorBox = document.createElement('div');
    rightMirrorBox.id = 'mirror-right-box';
    rightMirrorBox.style.position = 'fixed';
    rightMirrorBox.style.top = '72px';
    rightMirrorBox.style.right = '20px';
    rightMirrorBox.style.zIndex = '990';
    rightMirrorBox.style.width = '210px';
    rightMirrorBox.style.height = '115px';
    rightMirrorBox.style.borderRadius = '14px';
    rightMirrorBox.style.overflow = 'hidden';
    rightMirrorBox.style.border = '2px solid #334155';
    rightMirrorBox.style.background = '#0a0f1d';
    rightMirrorBox.style.boxShadow = '0 10px 30px rgba(0,0,0,0.8)';
    rightMirrorBox.style.pointerEvents = 'none';

    rightMirrorBox.innerHTML = `
      <canvas id="mirror-right-canvas" width="210" height="115" style="display:block; width:100%; height:100%;"></canvas>
      <div id="bsm-right-led" style="position:absolute; top:8px; right:10px; display:flex; align-items:center; gap:4px; padding:3px 6px; border-radius:6px; background:rgba(15,23,42,0.85); border:1px solid #475569; transition:all 0.15s;">
        <span id="bsm-right-text" style="font-size:10px; font-weight:800; font-family:monospace; color:#64748b;">BSM R</span>
        <span id="bsm-right-icon" style="font-size:12px; color:#64748b;">⚡🚗</span>
      </div>
      <div id="turn-right-indicator" style="position:absolute; bottom:6px; right:10px; font-size:14px; font-weight:900; color:#f59e0b; opacity:0; transition:opacity 0.1s;">
        ▶ ▶
      </div>
    `;
    document.body.appendChild(rightMirrorBox);

    this.leftMirrorCanvas = document.getElementById('mirror-left-canvas');
    this.rightMirrorCanvas = document.getElementById('mirror-right-canvas');
    if (this.leftMirrorCanvas) this.leftMirrorCtx = this.leftMirrorCanvas.getContext('2d');
    if (this.rightMirrorCanvas) this.rightMirrorCtx = this.rightMirrorCanvas.getContext('2d');

    // 3. MAIN QNX RTOS AUTOPILOT HUD (Bottom-Left)
    const hud = document.createElement('div');
    hud.id = 'hud-autopilot';
    hud.style.position = 'fixed';
    hud.style.bottom = '95px';
    hud.style.left = '20px';
    hud.style.zIndex = '999';
    hud.style.background = 'rgba(15, 23, 42, 0.90)';
    hud.style.backdropFilter = 'blur(14px)';
    hud.style.border = '1px solid rgba(255, 255, 255, 0.18)';
    hud.style.borderRadius = '16px';
    hud.style.padding = '14px 18px';
    hud.style.color = '#f8fafc';
    hud.style.fontFamily = "system-ui, -apple-system, sans-serif";
    hud.style.fontSize = '12px';
    hud.style.boxShadow = '0 16px 36px rgba(0,0,0,0.7)';
    hud.style.minWidth = '290px';
    hud.style.pointerEvents = 'auto';

    hud.innerHTML = `
      <div style="display:flex; align-items:center; justify-content:space-between; margin-bottom:10px;">
        <div style="display:flex; align-items:center; gap:6px;">
          <span style="display:inline-block; width:8px; height:8px; border-radius:50%; background:#10b981;" id="qnx-dot"></span>
          <span style="font-weight:800; letter-spacing:0.06em; font-size:11px; text-transform:uppercase; color:#38bdf8;">
            QNX 8.0 RTOS ADAS L3
          </span>
        </div>
        <button id="btn-toggle-ap" style="cursor:pointer; background:#06b6d4; color:#fff; border:none; border-radius:6px; padding:3px 10px; font-size:11px; font-weight:700; transition:all 0.2s;">
          ON [P]
        </button>
      </div>

      <!-- Turn Signal and BSM Status Bar -->
      <div style="display:flex; align-items:center; justify-content:space-between; background:rgba(0,0,0,0.35); padding:6px 10px; border-radius:8px; margin-bottom:10px;">
        <span id="hud-turn-l" style="font-size:14px; font-weight:900; color:#475569; transition:color 0.1s;">◀ BLINK</span>
        <div style="display:flex; gap:8px;">
          <button id="btn-lane-left" style="cursor:pointer; background:#1e293b; color:#94a3b8; border:1px solid #475569; border-radius:4px; padding:2px 6px; font-size:10px; font-weight:700;">
            ← Lane [Q]
          </button>
          <button id="btn-lane-right" style="cursor:pointer; background:#1e293b; color:#94a3b8; border:1px solid #475569; border-radius:4px; padding:2px 6px; font-size:10px; font-weight:700;">
            Lane → [E]
          </button>
        </div>
        <span id="hud-turn-r" style="font-size:14px; font-weight:900; color:#475569; transition:color 0.1s;">BLINK ▶</span>
      </div>

      <div style="display:flex; justify-content:space-between; align-items:baseline; margin-bottom:4px;">
        <span style="color:#94a3b8;">Drive Mode:</span>
        <span id="ap-state" style="font-weight:700; color:#10b981;">CRUISING</span>
      </div>
      <div style="display:flex; justify-content:space-between; align-items:baseline; margin-bottom:4px;">
        <span style="color:#94a3b8;">Speed / Limit:</span>
        <span id="ap-speed" style="font-family:monospace; font-weight:700; color:#38bdf8;">0.0 / 32 mph</span>
      </div>
      <div style="display:flex; justify-content:space-between; align-items:baseline; margin-bottom:4px;">
        <span style="color:#94a3b8;">Forward Radar:</span>
        <span id="ap-radar" style="font-family:monospace; font-weight:700; color:#cbd5e1;">-- m</span>
      </div>
      <div style="display:flex; justify-content:space-between; align-items:baseline; margin-bottom:6px;">
        <span style="color:#94a3b8;">Blind Spots:</span>
        <span id="ap-bsm-text" style="font-family:monospace; font-weight:700; color:#10b981;">L: CLEAR | R: CLEAR</span>
      </div>
      <div id="ap-status-banner" style="font-size:10.5px; color:#cbd5e1; background:rgba(30,41,59,0.7); padding:5px 8px; border-radius:6px; border-left:3px solid #06b6d4; line-height:1.3;">
        Initializing QNX Safety Kernel...
      </div>
    `;

    document.body.appendChild(hud);
    this.hudElement = hud;

    // Attach HUD button listeners
    const btnToggle = document.getElementById('btn-toggle-ap');
    if (btnToggle) btnToggle.addEventListener('click', () => this.toggle());

    const btnLaneL = document.getElementById('btn-lane-left');
    if (btnLaneL) btnLaneL.addEventListener('click', () => this.requestLaneChange(-1));

    const btnLaneR = document.getElementById('btn-lane-right');
    if (btnLaneR) btnLaneR.addEventListener('click', () => this.requestLaneChange(+1));
  }

  // Render Real-Time Rear Perspective Mirror
  renderSideMirror(ctx, canvas, car, isLeft, bsmActive) {
    if (!ctx || !canvas || !car) return;

    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);

    // 1. Sky and Asphalt Ground Perspective
    const horizonY = h * 0.44;
    const skyGrad = ctx.createLinearGradient(0, 0, 0, horizonY);
    skyGrad.addColorStop(0, '#0c1626');
    skyGrad.addColorStop(1, '#1e293b');
    ctx.fillStyle = skyGrad;
    ctx.fillRect(0, 0, w, horizonY);

    const roadGrad = ctx.createLinearGradient(0, horizonY, 0, h);
    roadGrad.addColorStop(0, '#1a222d');
    roadGrad.addColorStop(1, '#0f141c');
    ctx.fillStyle = roadGrad;
    ctx.fillRect(0, horizonY, w, h - horizonY);

    // 2. Road Lanes & Perspective Lines
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 1.5;
    // Vanishing point shifted slightly outwards to reflect mirror angle
    const vpX = isLeft ? w * 0.38 : w * 0.62;

    // Curbs
    ctx.beginPath();
    ctx.moveTo(vpX - 40, horizonY);
    ctx.lineTo(0, h);
    ctx.moveTo(vpX + 40, horizonY);
    ctx.lineTo(w, h);
    ctx.stroke();

    // Dashed center lane markings
    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = 2.0;
    ctx.setLineDash([8, 8]);
    ctx.beginPath();
    ctx.moveTo(vpX, horizonY);
    ctx.lineTo(isLeft ? w * 0.25 : w * 0.75, h);
    ctx.stroke();
    ctx.setLineDash([]);

    // 3. Project Traffic Vehicles onto Mirror Glass
    const px = car.position.x;
    const pz = car.position.z;
    const sinH = Math.sin(car.heading);
    const cosH = Math.cos(car.heading);

    const traffic = [];
    if (this.trafficSystem?.cars) {
      for (const c of this.trafficSystem.cars) {
        if (Number.isFinite(c.x) && Number.isFinite(c.z)) traffic.push(c);
      }
    }
    if (this.trafficSystem?.buses) {
      for (const b of this.trafficSystem.buses) {
        if (Number.isFinite(b.x) && Number.isFinite(b.z)) traffic.push({ ...b, isBus: true });
      }
    }

    // Mirror orientation: facing rearward with slight outward deflection
    const mirrorAngleOffset = isLeft ? -0.12 : 0.12;
    const mirrorSin = Math.sin(car.heading + Math.PI + mirrorAngleOffset);
    const mirrorCos = Math.cos(car.heading + Math.PI + mirrorAngleOffset);

    for (const v of traffic) {
      const dx = v.x - px;
      const dz = v.z - pz;

      // Distance along mirror optical axis (rearward)
      const mirrorDepth = dx * mirrorSin + dz * mirrorCos;
      // Lateral distance in mirror view plane
      const mirrorLat = dx * mirrorCos - dz * mirrorSin;

      // Render vehicles behind ego car up to 55m
      if (mirrorDepth > 2.0 && mirrorDepth < 55.0) {
        const perspective = 70.0 / mirrorDepth;
        const screenX = vpX + (mirrorLat * perspective * 2.8);
        const screenY = horizonY + (1.2 * perspective * 1.8);

        const carW = (v.isBus ? 3.2 : 2.0) * perspective;
        const carH = (v.isBus ? 3.0 : 1.6) * perspective;

        if (screenX > -carW && screenX < w + carW && screenY > horizonY && screenY < h + carH) {
          // Approaching Vehicle Body
          const distColor = mirrorDepth < 10.0 ? '#ef4444' : mirrorDepth < 22.0 ? '#f59e0b' : '#38bdf8';
          ctx.fillStyle = distColor;
          ctx.strokeStyle = '#f8fafc';
          ctx.lineWidth = 1.0;

          // Vehicle 3D block
          ctx.fillRect(screenX - carW * 0.5, screenY - carH, carW, carH);
          ctx.strokeRect(screenX - carW * 0.5, screenY - carH, carW, carH);

          // Headlights or Taillights
          ctx.fillStyle = '#fef08a';
          const lightR = Math.max(1.5, carW * 0.12);
          ctx.beginPath();
          ctx.arc(screenX - carW * 0.35, screenY - carH * 0.3, lightR, 0, Math.PI * 2);
          ctx.arc(screenX + carW * 0.35, screenY - carH * 0.3, lightR, 0, Math.PI * 2);
          ctx.fill();

          // Distance callout
          if (mirrorDepth < 30.0) {
            ctx.fillStyle = '#ffffff';
            ctx.font = 'bold 9px monospace';
            ctx.textAlign = 'center';
            ctx.fillText(`${mirrorDepth.toFixed(0)}m`, screenX, screenY - carH - 3);
          }
        }
      }
    }

    // 4. Subtle Convex Mirror Vignette & Reflection Glare
    const grad = ctx.createRadialGradient(w * 0.5, h * 0.5, w * 0.3, w * 0.5, h * 0.5, w * 0.6);
    grad.addColorStop(0, 'rgba(0,0,0,0)');
    grad.addColorStop(1, 'rgba(0,0,0,0.45)');
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, w, h);

    // Convex mirror curved grid lines
    ctx.strokeStyle = 'rgba(255,255,255,0.06)';
    ctx.lineWidth = 1.0;
    ctx.beginPath();
    ctx.arc(w * 0.5, h * 0.5, w * 0.45, 0, Math.PI * 2);
    ctx.stroke();

    // 5. Blind-Spot Warning Border Alert Pulse
    if (bsmActive) {
      ctx.strokeStyle = 'rgba(239, 68, 68, 0.85)';
      ctx.lineWidth = 3.5;
      ctx.strokeRect(2, 2, w - 4, h - 4);
    }
  }

  // Perception Scan for Radar Telemetry
  scanPerception(car, elapsed = 0) {
    const px = car.position.x;
    const pz = car.position.z;
    const sin = Math.sin(car.heading);
    const cos = Math.cos(car.heading);

    const traffic = [];
    if (this.trafficSystem?.cars) {
      for (const c of this.trafficSystem.cars) {
        if (Number.isFinite(c.x) && Number.isFinite(c.z)) traffic.push({ ...c, kind: 'Car', isBus: false });
      }
    }
    if (this.trafficSystem?.buses) {
      for (const b of this.trafficSystem.buses) {
        if (Number.isFinite(b.x) && Number.isFinite(b.z)) traffic.push({ ...b, kind: 'Bus', isBus: true });
      }
    }

    // Scan for upcoming Red Traffic Signals on the current avenue corridor
    const effectiveTime = elapsed + (window.__BENCHMARK_TIME_OFFSET__ || 0);
    const aveSignal = signalPhase(effectiveTime, true);
    let redStopObstacle = null;

    if ((aveSignal === 'red' || aveSignal === 'amber') && this.trafficSystem?.network?.nodes) {
      let minFwd = 999.0;
      let targetNode = null;

      for (const node of this.trafficSystem.network.nodes) {
        if (!node.signal) continue;
        const dx = node.x - px;
        const dz = node.z - pz;
        const fwd = dx * sin + dz * cos;
        const lat = dx * cos - dz * sin;

        // Find the closest intersection node directly ahead in the avenue corridor
        if (fwd > 1.5 && fwd < 85.0 && Math.abs(lat) < 8.0) {
          if (fwd < minFwd) {
            minFwd = fwd;
            targetNode = node;
          }
        }
      }

      if (targetNode) {
        const stopLineFwd = minFwd - (targetNode.hz || 3.0) - 2.5;
        const obsFwd = Math.max(1.8, stopLineFwd);
        redStopObstacle = {
          x: px,
          z: pz + (cos * obsFwd) + (sin * 0.0),
          v: 0.0,
          half: 1.0,
          kind: 'Red Light Stop Line',
          isBus: false
        };
        traffic.push(redStopObstacle);

        if (!this._lastLoggedSignal || Math.abs(minFwd - this._lastLoggedSignal) > 15.0) {
          this._lastLoggedSignal = minFwd;
          console.log(`[QNX] Red light detected ahead at distance ${minFwd.toFixed(1)}m, stop line at ${obsFwd.toFixed(1)}m`);
        }
      }
    }
    this.activeRedStopObstacle = redStopObstacle;

    let nearestDist = 999;
    let nearestRelSpeed = 0;
    let nearestName = null;

    for (const v of traffic) {
      const dx = v.x - px;
      const dz = v.z - pz;
      const fwd = dx * sin + dz * cos;
      const lat = dx * cos - dz * sin;

      // Forward radar detection cone (75m ahead, within lane corridor ~2.3m)
      if (fwd > 1.2 && fwd < 75.0 && Math.abs(lat) < 2.3) {
        const dist = fwd - (v.half || 2.0) - 2.3;
        if (dist < nearestDist) {
          nearestDist = Math.max(0.1, dist);
          nearestRelSpeed = (v.v || 0) - car.speed;
          nearestName = v.kind;
        }
      }
    }

    this.leadDist = nearestDist;
    this.leadRelSpeed = nearestRelSpeed;
    this.closestObstacleName = nearestName;

    if (this.leadRelSpeed < -0.1 && this.leadDist < 75.0) {
      this.ttc = this.leadDist / (-this.leadRelSpeed);
    } else {
      this.ttc = 999;
    }

    return traffic;
  }

  // Update Loop called every frame (60 FPS)
  update(dt, elapsed = null) {
    if (this.controllerManager.activeMode !== 'car') {
      if (this.hudElement) this.hudElement.style.display = 'none';
      const m1 = document.getElementById('mirror-left-box');
      const m2 = document.getElementById('mirror-right-box');
      if (m1) m1.style.display = 'none';
      if (m2) m2.style.display = 'none';
      return;
    }

    if (this.hudElement) this.hudElement.style.display = 'block';
    const m1 = document.getElementById('mirror-left-box');
    const m2 = document.getElementById('mirror-right-box');
    if (m1) m1.style.display = 'block';
    if (m2) m2.style.display = 'block';

    const car = this.controllerManager.entities?.car;
    if (!car) return;

    this.elapsed = elapsed !== null ? elapsed : (this.elapsed || 0) + dt;

    // 1. Perception Radar Scan
    const trafficList = this.scanPerception(car, this.elapsed);

    // 2. Transmit Real-Time Telemetry to QNX RTOS Bridge
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      // Package closest obstacles for QNX L3 Kernel (prioritize red-light barrier + sorted nearest obstacles)
      const obsPayload = [];
      const px = car.position.x;
      const pz = car.position.z;

      if (this.activeRedStopObstacle) {
        obsPayload.push({
          x: Math.round(this.activeRedStopObstacle.x * 10) / 10,
          z: Math.round(this.activeRedStopObstacle.z * 10) / 10,
          speed: 0.0,
          type: 0
        });
      }

      const sortedTraffic = trafficList
        .filter(t => t.kind !== 'Red Light Stop Line')
        .map(t => {
          const dx = t.x - px;
          const dz = t.z - pz;
          return { t, distSq: dx * dx + dz * dz };
        })
        .filter(item => item.distSq < 7500)
        .sort((a, b) => a.distSq - b.distSq);

      for (const item of sortedTraffic) {
        if (obsPayload.length >= 24) break;
        obsPayload.push({
          x: Math.round(item.t.x * 10) / 10,
          z: Math.round(item.t.z * 10) / 10,
          speed: Math.round((item.t.v || 0) * 10) / 10,
          type: item.t.isBus ? 1 : 0
        });
      }

      const telemetry = {
        x: Math.round(car.position.x * 100) / 100,
        y: Math.round(car.position.y * 100) / 100,
        z: Math.round(car.position.z * 100) / 100,
        speed: Math.round(car.speed * 100) / 100,
        heading: Math.round(car.heading * 1000) / 1000,
        steer_angle: Math.round((car.steerAngle || 0) * 1000) / 1000,
        current_lane: this.currentLaneIndex,
        target_lane: this.targetLaneIndex,
        dt: Math.round(dt * 1000) / 1000,
        obs: obsPayload
      };

      this.ws.send(JSON.stringify(telemetry));
    }

    // 3. Fallback Autonomous Controls if QNX link is offline
    if (!this.qnxConnected) {
      // Local fallback ACC & AEB logic
      if ((this.ttc < 1.7 && this.leadDist < 30.0) || (this.leadDist < 6.5 && car.speed > 1.0)) {
        this.throttle = -1.0;
        this.brake = 1.0;
        this.state = 'AEB_BRAKING';
        this.statusMessage = 'Fallback AEB Braking Active';
      } else if (this.leadDist < 45.0) {
        const desiredGap = this.minSafeGap + Math.max(0, car.speed) * this.timeHeadway;
        const gapError = this.leadDist - desiredGap;
        const targetAccel = 0.5 * gapError + 0.85 * this.leadRelSpeed;
        this.throttle = Math.max(-0.85, Math.min(1.0, targetAccel / 2.2));
        this.brake = this.throttle < 0 ? -this.throttle : 0;
        this.state = 'ACC_FOLLOWING';
        this.statusMessage = `Fallback ACC Following (${this.leadDist.toFixed(1)}m)`;
      } else {
        const speedError = this.targetSpeed - car.speed;
        this.throttle = Math.max(-0.35, Math.min(1.0, speedError * 0.45));
        this.brake = 0;
        this.state = 'CRUISING';
        this.statusMessage = 'Autopilot Cruising (Searching for QNX Link)';
      }
    }

    // 4. Actuate 3D Vehicle Controls
    if (this.enabled) {
      if (this.brake > 0.05 && car.speed > 0.2) {
        // Forward motion with brake active: apply brakes via backward key
        this.controllerManager.keys.forward = false;
        this.controllerManager.keys.backward = Math.min(1.0, this.brake);
      } else if (car.speed <= 0.2 && (this.brake > 0.05 || this.state === 'STOPPED')) {
        // Vehicle at rest holding brake: zero all throttle and zero speed to prevent rolling or reversing
        this.controllerManager.keys.forward = false;
        this.controllerManager.keys.backward = false;
        car.speed = 0;
        car.vx = 0;
        car.vz = 0;
      } else {
        this.controllerManager.keys.forward = this.throttle > 0 ? this.throttle : false;
        this.controllerManager.keys.backward = this.throttle < 0 ? -this.throttle : false;
      }
      this.controllerManager.keys.left = this.steerInput > 0 ? this.steerInput : false;
      this.controllerManager.keys.right = this.steerInput < 0 ? -this.steerInput : false;
    }

    // 5. Update 3D Turn Signals and Brake Lights on Car Fascias
    if (car.carState) {
      car.carState.value.x = (this.brake > 0.1 || (car.speed * this.throttle < 0)) ? 1 : 0; // Brake lights
      car.carState.value.y = (this.leftBlinker || this.hazardLights) ? 1 : 0; // Left blinker
      car.carState.value.z = (this.rightBlinker || this.hazardLights) ? 1 : 0; // Right blinker
    }

    // 6. Render Real-Time Side-View Mirrors
    this.renderSideMirror(this.leftMirrorCtx, this.leftMirrorCanvas, car, true, this.leftBsmAlert);
    this.renderSideMirror(this.rightMirrorCtx, this.rightMirrorCanvas, car, false, this.rightBsmAlert);

    // 7. Update HUD & Mirror Warning Indicators
    this.updateHud();
  }

  updateHud() {
    if (!this.hudElement) return;

    const elBtn = document.getElementById('btn-toggle-ap');
    const elDot = document.getElementById('qnx-dot');
    const elState = document.getElementById('ap-state');
    const elSpeed = document.getElementById('ap-speed');
    const elRadar = document.getElementById('ap-radar');
    const elBsmText = document.getElementById('ap-bsm-text');
    const elBanner = document.getElementById('ap-status-banner');
    const elTurnL = document.getElementById('hud-turn-l');
    const elTurnR = document.getElementById('hud-turn-r');

    // QNX Status Dot
    if (elDot) {
      elDot.style.background = this.qnxConnected ? '#10b981' : '#ef4444';
      elDot.style.boxShadow = this.qnxConnected ? '0 0 8px #10b981' : '0 0 8px #ef4444';
    }

    if (elBtn) {
      elBtn.textContent = this.enabled ? 'ON [P]' : 'OFF [P]';
      elBtn.style.background = this.enabled ? '#06b6d4' : '#475569';
    }

    if (elState) {
      elState.textContent = this.enabled ? this.state.replace('_', ' ') : 'MANUAL';
      if (!this.enabled) {
        elState.style.color = '#94a3b8';
      } else if (this.state === 'AEB_BRAKING') {
        elState.style.color = '#ef4444';
      } else if (this.state === 'ACC_FOLLOWING') {
        elState.style.color = '#f59e0b';
      } else {
        elState.style.color = '#10b981';
      }
    }

    const car = this.controllerManager.entities?.car;
    const speedMph = car ? (Math.abs(car.speed) * 2.23694).toFixed(1) : '0.0';
    if (elSpeed) {
      elSpeed.textContent = `${speedMph} / ${this.targetSpeedMph} mph`;
    }

    if (elRadar) {
      if (this.leadDist < 70) {
        elRadar.textContent = `${this.leadDist.toFixed(1)} m (${this.closestObstacleName || 'Car'})`;
        elRadar.style.color = this.leadDist < 12 ? '#ef4444' : '#fbbf24';
      } else {
        elRadar.textContent = 'Clear (>70m)';
        elRadar.style.color = '#10b981';
      }
    }

    if (elBsmText) {
      const leftStr = this.leftBsmAlert ? '<span style="color:#ef4444;">ALERT</span>' : '<span style="color:#10b981;">CLEAR</span>';
      const rightStr = this.rightBsmAlert ? '<span style="color:#ef4444;">ALERT</span>' : '<span style="color:#10b981;">CLEAR</span>';
      elBsmText.innerHTML = `L: ${leftStr} | R: ${rightStr}`;
    }

    if (elBanner) {
      elBanner.textContent = this.statusMessage;
      if (this.statusMessage.includes('PREVENTED') || this.statusMessage.includes('Blocked')) {
        elBanner.style.borderLeftColor = '#ef4444';
        elBanner.style.color = '#fca5a5';
      } else if (this.statusMessage.includes('AEB')) {
        elBanner.style.borderLeftColor = '#ef4444';
        elBanner.style.color = '#fca5a5';
      } else if (this.statusMessage.includes('ACC')) {
        elBanner.style.borderLeftColor = '#f59e0b';
        elBanner.style.color = '#fef08a';
      } else {
        elBanner.style.borderLeftColor = '#06b6d4';
        elBanner.style.color = '#cbd5e1';
      }
    }

    // Turn Signal Indicators on HUD
    if (elTurnL) {
      elTurnL.style.color = (this.leftBlinker || this.hazardLights) ? '#f59e0b' : '#334155';
    }
    if (elTurnR) {
      elTurnR.style.color = (this.rightBlinker || this.hazardLights) ? '#f59e0b' : '#334155';
    }

    // Mirror BSM LED Icons
    const bsmL = document.getElementById('bsm-left-led');
    const bsmLText = document.getElementById('bsm-left-text');
    const bsmLIcon = document.getElementById('bsm-left-icon');
    if (bsmL) {
      if (this.leftBsmAlert) {
        bsmL.style.background = '#ef4444';
        bsmL.style.borderColor = '#ffffff';
        bsmL.style.boxShadow = '0 0 12px #ef4444';
        if (bsmLText) bsmLText.style.color = '#ffffff';
        if (bsmLIcon) bsmLIcon.style.color = '#ffffff';
      } else {
        bsmL.style.background = 'rgba(15,23,42,0.85)';
        bsmL.style.borderColor = '#475569';
        bsmL.style.boxShadow = 'none';
        if (bsmLText) bsmLText.style.color = '#64748b';
        if (bsmLIcon) bsmLIcon.style.color = '#64748b';
      }
    }

    const bsmR = document.getElementById('bsm-right-led');
    const bsmRText = document.getElementById('bsm-right-text');
    const bsmRIcon = document.getElementById('bsm-right-icon');
    if (bsmR) {
      if (this.rightBsmAlert) {
        bsmR.style.background = '#ef4444';
        bsmR.style.borderColor = '#ffffff';
        bsmR.style.boxShadow = '0 0 12px #ef4444';
        if (bsmRText) bsmRText.style.color = '#ffffff';
        if (bsmRIcon) bsmRIcon.style.color = '#ffffff';
      } else {
        bsmR.style.background = 'rgba(15,23,42,0.85)';
        bsmR.style.borderColor = '#475569';
        bsmR.style.boxShadow = 'none';
        if (bsmRText) bsmRText.style.color = '#64748b';
        if (bsmRIcon) bsmRIcon.style.color = '#64748b';
      }
    }

    // Mirror Turn Arrows
    const arrowL = document.getElementById('turn-left-indicator');
    if (arrowL) arrowL.style.opacity = (this.leftBlinker || this.hazardLights) ? '1' : '0';

    const arrowR = document.getElementById('turn-right-indicator');
    if (arrowR) arrowR.style.opacity = (this.rightBlinker || this.hazardLights) ? '1' : '0';
  }
}
