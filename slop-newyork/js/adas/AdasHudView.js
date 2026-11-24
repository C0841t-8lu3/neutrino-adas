// js/adas/AdasHudView.js
// ============================================================================
// ADAS HUD & Dual Side-Mirror DOM View Component
// Encapsulates all DOM nodes, CSS glassmorphism styling, and telemetry display.
// Features Watchdog Safe Stop banner, Tesla FSD Profiles (CHILL | STANDARD | HURRY),
// Tesla FSD Status Strip, sensor noise toggle, and Flight Recorder export buttons.
// ============================================================================

export class AdasHudView {
  constructor(callbacks = {}) {
    this.callbacks = callbacks; // { onToggle, onLaneChangeLeft, onLaneChangeRight, onProfileChange, onToggleFsdVis, onToggleSensors, onToggleSurface, onExportJson, onExportCsv }
    this.hudElement = null;
    this.leftMirrorBox = null;
    this.rightMirrorBox = null;
    this.leftMirrorCanvas = null;
    this.rightMirrorCanvas = null;
    this.leftMirrorCtx = null;
    this.rightMirrorCtx = null;
    this.lastHudUpdate = 0;

    if (typeof document !== 'undefined') {
      this.createHudAndMirrors();
    }
  }

  _setDomText(el, val) {
    if (!el) return;
    if (el._cachedText !== val) {
      el._cachedText = val;
      el.textContent = val;
    }
  }

  _setDomHtml(el, val) {
    if (!el) return;
    if (el._cachedHtml !== val) {
      el._cachedHtml = val;
      el.innerHTML = val;
    }
  }

  _setStyle(el, prop, val) {
    if (!el) return;
    if (el._cachedStyles === undefined) el._cachedStyles = {};
    if (el._cachedStyles[prop] !== val) {
      el._cachedStyles[prop] = val;
      el.style[prop] = val;
    }
  }

  createHudAndMirrors() {
    // 1. LEFT SIDE-VIEW MIRROR (Top-Left Dock)
    const leftBox = document.createElement('div');
    leftBox.id = 'mirror-left-box';
    leftBox.style.position = 'fixed';
    leftBox.style.top = '72px';
    leftBox.style.left = '20px';
    leftBox.style.zIndex = '990';
    leftBox.style.width = '210px';
    leftBox.style.height = '115px';
    leftBox.style.borderRadius = '14px';
    leftBox.style.overflow = 'hidden';
    leftBox.style.border = '2px solid #334155';
    leftBox.style.background = '#0a0f1d';
    leftBox.style.boxShadow = '0 10px 30px rgba(0,0,0,0.8)';
    leftBox.style.pointerEvents = 'none';

    leftBox.innerHTML = `
      <canvas id="mirror-left-canvas" width="210" height="115" style="display:block; width:100%; height:100%;"></canvas>
      <div id="bsm-left-led" style="position:absolute; top:8px; left:10px; display:flex; align-items:center; gap:4px; padding:3px 6px; border-radius:6px; background:rgba(15,23,42,0.85); border:1px solid #475569; transition:all 0.15s;">
        <span id="bsm-left-icon" style="font-size:12px; color:#64748b;">🚗⚡</span>
        <span id="bsm-left-text" style="font-size:10px; font-weight:800; font-family:monospace; color:#64748b;">BSM L</span>
      </div>
      <div id="turn-left-indicator" style="position:absolute; bottom:6px; left:10px; font-size:14px; font-weight:900; color:#f59e0b; opacity:0; transition:opacity 0.1s;">
        ◀ ◀
      </div>
    `;
    document.body.appendChild(leftBox);
    this.leftMirrorBox = leftBox;

    // 2. RIGHT SIDE-VIEW MIRROR (Top-Right Dock)
    const rightBox = document.createElement('div');
    rightBox.id = 'mirror-right-box';
    rightBox.style.position = 'fixed';
    rightBox.style.top = '72px';
    rightBox.style.right = '20px';
    rightBox.style.zIndex = '990';
    rightBox.style.width = '210px';
    rightBox.style.height = '115px';
    rightBox.style.borderRadius = '14px';
    rightBox.style.overflow = 'hidden';
    rightBox.style.border = '2px solid #334155';
    rightBox.style.background = '#0a0f1d';
    rightBox.style.boxShadow = '0 10px 30px rgba(0,0,0,0.8)';
    rightBox.style.pointerEvents = 'none';

    rightBox.innerHTML = `
      <canvas id="mirror-right-canvas" width="210" height="115" style="display:block; width:100%; height:100%;"></canvas>
      <div id="bsm-right-led" style="position:absolute; top:8px; right:10px; display:flex; align-items:center; gap:4px; padding:3px 6px; border-radius:6px; background:rgba(15,23,42,0.85); border:1px solid #475569; transition:all 0.15s;">
        <span id="bsm-right-text" style="font-size:10px; font-weight:800; font-family:monospace; color:#64748b;">BSM R</span>
        <span id="bsm-right-icon" style="font-size:12px; color:#64748b;">⚡🚗</span>
      </div>
      <div id="turn-right-indicator" style="position:absolute; bottom:6px; right:10px; font-size:14px; font-weight:900; color:#f59e0b; opacity:0; transition:opacity 0.1s;">
        ▶ ▶
      </div>
    `;
    document.body.appendChild(rightBox);
    this.rightMirrorBox = rightBox;

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
    hud.style.minWidth = '315px';
    hud.style.pointerEvents = 'auto';

    hud.innerHTML = `
      <!-- QNX Safety Watchdog Banner -->
      <div id="qnx-watchdog-banner" style="display:none; background:#dc2626; color:#ffffff; font-weight:800; font-size:11px; padding:6px 10px; border-radius:6px; margin-bottom:8px; text-align:center; box-shadow:0 0 14px #ef4444; letter-spacing:0.04em;">
        ⚠️ QNX WATCHDOG FAULT — SAFE STOP
      </div>

      <!-- Header: QNX Kernel Indicator & Autopilot State Toggle -->
      <div style="display:flex; align-items:center; justify-content:space-between; margin-bottom:8px;">
        <div style="display:flex; align-items:center; gap:6px;">
          <span style="display:inline-block; width:8px; height:8px; border-radius:50%; background:#10b981;" id="qnx-dot"></span>
          <span style="font-weight:800; letter-spacing:0.06em; font-size:11px; text-transform:uppercase; color:#38bdf8;">
            QNX 8.0 RTOS ADAS L3
          </span>
        </div>
        <button id="btn-toggle-ap" style="cursor:pointer; background:#06b6d4; color:#fff; border:none; border-radius:6px; padding:3px 10px; font-size:11px; font-weight:700; transition:all 0.2s;">
          ON
        </button>
      </div>

      <!-- Tesla FSD Driving Profile Selector (CHILL | STANDARD | HURRY) [P] -->
      <div id="fsd-profile-container" style="display:flex; background:rgba(0,0,0,0.45); border-radius:10px; padding:3px; border:1px solid rgba(255,255,255,0.12); margin-bottom:10px; gap:2px;">
        <button id="btn-profile-chill" style="flex:1; cursor:pointer; background:transparent; color:#94a3b8; border:none; border-radius:8px; padding:4px 0; font-size:10px; font-weight:800; letter-spacing:0.04em; transition:all 0.2s;" title="Chill Mode: 1.8s headway, gentle acceleration">
          CHILL
        </button>
        <button id="btn-profile-standard" style="flex:1; cursor:pointer; background:#0284c7; color:#ffffff; border:none; border-radius:8px; padding:4px 0; font-size:10px; font-weight:800; letter-spacing:0.04em; transition:all 0.2s; box-shadow:0 0 10px rgba(2,132,199,0.5);" title="Standard Mode: 1.3s headway, proactive passing">
          STANDARD
        </button>
        <button id="btn-profile-hurry" style="flex:1; cursor:pointer; background:transparent; color:#94a3b8; border:none; border-radius:8px; padding:4px 0; font-size:10px; font-weight:800; letter-spacing:0.04em; transition:all 0.2s;" title="Hurry Mode: 1.0s headway, decisive assertiveness [P]">
          HURRY
        </button>
      </div>

      <!-- Tesla FSD Status Strip: Icon, Speed km/h + Limit Sign, Next Turn Pill, Vis Toggle -->
      <div style="display:flex; align-items:center; justify-content:space-between; background:rgba(2,132,199,0.12); border:1px solid rgba(56,189,248,0.25); border-radius:10px; padding:6px 10px; margin-bottom:10px;">
        <div style="display:flex; align-items:center; gap:8px;">
          <div id="fsd-active-badge" style="display:flex; align-items:center; gap:4px; font-weight:900; font-size:11px; color:#38bdf8; letter-spacing:0.04em;">
            <span style="font-size:14px; color:#38bdf8;">⚡</span> FSD
          </div>
          <div style="display:flex; align-items:baseline; gap:3px;">
            <span id="fsd-speed-kmh" style="font-family:monospace; font-size:18px; font-weight:900; color:#ffffff;">0</span>
            <span style="font-size:10px; color:#94a3b8; font-weight:700;">km/h</span>
          </div>
          <!-- Speed limit sign -->
          <div id="fsd-speed-limit-badge" style="display:inline-flex; flex-direction:column; align-items:center; justify-content:center; width:28px; height:32px; border:1.5px solid #000; border-radius:4px; background:#fff; color:#000; font-weight:900; line-height:1; box-shadow:0 1px 3px rgba(0,0,0,0.5);" title="Speed Limit (km/h)">
            <span style="font-size:6px; font-weight:900;">LIMIT</span>
            <span id="fsd-speed-limit-val" style="font-size:12px; font-weight:900;">50</span>
          </div>
        </div>
        <div style="display:flex; align-items:center; gap:6px;">
          <!-- Next Turn Pill -->
          <div id="fsd-turn-pill" style="display:flex; align-items:center; gap:4px; background:rgba(15,23,42,0.8); border:1px solid rgba(255,255,255,0.15); border-radius:6px; padding:3px 7px; font-size:10px; font-weight:700; color:#e2e8f0;">
            <span id="fsd-turn-icon" style="color:#38bdf8;">⬆</span>
            <span id="fsd-turn-text">Follow Road</span>
          </div>
          <!-- FSD 3D Visualizer Toggle Button -->
          <button id="btn-toggle-fsd-vis" style="cursor:pointer; background:#1e293b; color:#38bdf8; border:1px solid #0284c7; border-radius:6px; padding:3px 7px; font-size:10px; font-weight:700; transition:all 0.2s;" title="Toggle 3D Tesla FSD Trajectory Ribbon [V]">
            Vis [V]: ON
          </button>
        </div>
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
      <div id="ap-status-banner" style="font-size:10.5px; color:#cbd5e1; background:rgba(30,41,59,0.7); padding:5px 8px; border-radius:6px; border-left:3px solid #06b6d4; line-height:1.3; margin-bottom:8px;">
        Initializing QNX Safety Kernel...
      </div>

      <!-- SIL Realism & Telemetry Controls -->
      <div style="display:flex; gap:4px; flex-wrap:wrap; border-top:1px solid rgba(255,255,255,0.1); padding-top:8px;">
        <button id="btn-toggle-sensors" style="cursor:pointer; background:#1e293b; color:#94a3b8; border:1px solid #475569; border-radius:4px; padding:3px 6px; font-size:10px; font-weight:600;">
          Sensors: Ground Truth
        </button>
        <button id="btn-toggle-surface" style="cursor:pointer; background:#1e293b; color:#94a3b8; border:1px solid #475569; border-radius:4px; padding:3px 6px; font-size:10px; font-weight:600;">
          Road: Dry (μ=0.95)
        </button>
        <button id="btn-export-json" style="cursor:pointer; background:#1e293b; color:#38bdf8; border:1px solid #0284c7; border-radius:4px; padding:3px 6px; font-size:10px; font-weight:600;" title="Export 60s flight recorder log to JSON">
          Log JSON
        </button>
        <button id="btn-export-csv" style="cursor:pointer; background:#1e293b; color:#38bdf8; border:1px solid #0284c7; border-radius:4px; padding:3px 6px; font-size:10px; font-weight:600;" title="Export 60s flight recorder log to CSV">
          Log CSV
        </button>
      </div>
    `;

    document.body.appendChild(hud);
    this.hudElement = hud;

    // Attach HUD button listeners
    const btnToggle = document.getElementById('btn-toggle-ap');
    if (btnToggle) btnToggle.addEventListener('click', () => this.callbacks.onToggle && this.callbacks.onToggle());

    const btnChill = document.getElementById('btn-profile-chill');
    if (btnChill) btnChill.addEventListener('click', () => this.callbacks.onProfileChange && this.callbacks.onProfileChange(0));

    const btnStd = document.getElementById('btn-profile-standard');
    if (btnStd) btnStd.addEventListener('click', () => this.callbacks.onProfileChange && this.callbacks.onProfileChange(1));

    const btnHurry = document.getElementById('btn-profile-hurry');
    if (btnHurry) btnHurry.addEventListener('click', () => this.callbacks.onProfileChange && this.callbacks.onProfileChange(2));

    const btnVis = document.getElementById('btn-toggle-fsd-vis');
    if (btnVis) btnVis.addEventListener('click', () => this.callbacks.onToggleFsdVis && this.callbacks.onToggleFsdVis());

    const btnLaneL = document.getElementById('btn-lane-left');
    if (btnLaneL) btnLaneL.addEventListener('click', () => this.callbacks.onLaneChangeLeft && this.callbacks.onLaneChangeLeft());

    const btnLaneR = document.getElementById('btn-lane-right');
    if (btnLaneR) btnLaneR.addEventListener('click', () => this.callbacks.onLaneChangeRight && this.callbacks.onLaneChangeRight());

    const btnSensors = document.getElementById('btn-toggle-sensors');
    if (btnSensors) btnSensors.addEventListener('click', () => this.callbacks.onToggleSensors && this.callbacks.onToggleSensors());

    const btnSurface = document.getElementById('btn-toggle-surface');
    if (btnSurface) btnSurface.addEventListener('click', () => this.callbacks.onToggleSurface && this.callbacks.onToggleSurface());

    const btnExportJson = document.getElementById('btn-export-json');
    if (btnExportJson) btnExportJson.addEventListener('click', () => this.callbacks.onExportJson && this.callbacks.onExportJson());

    const btnExportCsv = document.getElementById('btn-export-csv');
    if (btnExportCsv) btnExportCsv.addEventListener('click', () => this.callbacks.onExportCsv && this.callbacks.onExportCsv());
  }

  setVisibility(visible) {
    const disp = visible ? 'block' : 'none';
    if (this.hudElement) this.hudElement.style.display = disp;
    if (this.leftMirrorBox) this.leftMirrorBox.style.display = disp;
    if (this.rightMirrorBox) this.rightMirrorBox.style.display = disp;
  }

  updateHud(data, force = false) {
    if (!this.hudElement) return;

    const now = (typeof performance !== 'undefined' ? performance.now() : Date.now());
    // Throttle DOM text/style updates to ~14 Hz (70ms) unless forced
    if (!force && now - this.lastHudUpdate < 70) {
      return;
    }
    this.lastHudUpdate = now;

    const elBtn = document.getElementById('btn-toggle-ap');
    const elDot = document.getElementById('qnx-dot');
    const elState = document.getElementById('ap-state');
    const elSpeed = document.getElementById('ap-speed');
    const elRadar = document.getElementById('ap-radar');
    const elBsmText = document.getElementById('ap-bsm-text');
    const elBanner = document.getElementById('ap-status-banner');
    const elTurnL = document.getElementById('hud-turn-l');
    const elTurnR = document.getElementById('hud-turn-r');
    const elWatchdog = document.getElementById('qnx-watchdog-banner');
    const elSensorsBtn = document.getElementById('btn-toggle-sensors');
    const elSurfaceBtn = document.getElementById('btn-toggle-surface');

    // Watchdog Safe Stop Alert Banner
    if (elWatchdog) {
      this._setStyle(elWatchdog, 'display', data.watchdogFault ? 'block' : 'none');
    }

    // QNX Status Dot
    if (elDot) {
      this._setStyle(elDot, 'background', data.qnxConnected ? '#10b981' : '#ef4444');
      this._setStyle(elDot, 'boxShadow', data.qnxConnected ? '0 0 8px #10b981' : '0 0 8px #ef4444');
    }

    if (elBtn) {
      this._setDomText(elBtn, data.enabled ? 'ON' : 'OFF');
      this._setStyle(elBtn, 'background', data.enabled ? '#06b6d4' : '#475569');
    }

    // Profile selector pill updates
    const p = (data.driveProfile !== undefined) ? data.driveProfile : 1;
    const btnChill = document.getElementById('btn-profile-chill');
    const btnStd = document.getElementById('btn-profile-standard');
    const btnHurry = document.getElementById('btn-profile-hurry');
    if (btnChill && btnStd && btnHurry) {
      this._setStyle(btnChill, 'background', p === 0 ? '#0284c7' : 'transparent');
      this._setStyle(btnChill, 'color', p === 0 ? '#ffffff' : '#94a3b8');
      this._setStyle(btnChill, 'boxShadow', p === 0 ? '0 0 10px rgba(2,132,199,0.5)' : 'none');

      this._setStyle(btnStd, 'background', p === 1 ? '#0284c7' : 'transparent');
      this._setStyle(btnStd, 'color', p === 1 ? '#ffffff' : '#94a3b8');
      this._setStyle(btnStd, 'boxShadow', p === 1 ? '0 0 10px rgba(2,132,199,0.5)' : 'none');

      this._setStyle(btnHurry, 'background', p === 2 ? '#0284c7' : 'transparent');
      this._setStyle(btnHurry, 'color', p === 2 ? '#ffffff' : '#94a3b8');
      this._setStyle(btnHurry, 'boxShadow', p === 2 ? '0 0 10px rgba(2,132,199,0.5)' : 'none');
    }

    // Tesla FSD Status Strip Updates
    const elSpeedKmh = document.getElementById('fsd-speed-kmh');
    if (elSpeedKmh) {
      const kmh = Math.round((data.speedMps || 0) * 3.6);
      this._setDomText(elSpeedKmh, String(kmh));
    }
    const elTurnIcon = document.getElementById('fsd-turn-icon');
    const elTurnText = document.getElementById('fsd-turn-text');
    if (elTurnIcon && elTurnText) {
      if (data.nextTurnHint === 'LEFT') {
        this._setDomText(elTurnIcon, '↰');
        this._setDomText(elTurnText, `Left in ${data.nextTurnDistance || 0}m`);
      } else if (data.nextTurnHint === 'RIGHT') {
        this._setDomText(elTurnIcon, '↱');
        this._setDomText(elTurnText, `Right in ${data.nextTurnDistance || 0}m`);
      } else if (data.nextTurnHint === 'ARRIVING' || data.nextTurnHint === 'ARRIVED') {
        this._setDomText(elTurnIcon, '🏁');
        this._setDomText(elTurnText, 'Arriving');
      } else {
        this._setDomText(elTurnIcon, '⬆');
        this._setDomText(elTurnText, data.hasActiveRoute ? 'Follow Route' : 'Cruising');
      }
    }

    const elVisBtn = document.getElementById('btn-toggle-fsd-vis');
    if (elVisBtn && data.fsdVisEnabled !== undefined) {
      this._setDomText(elVisBtn, `Vis [V]: ${data.fsdVisEnabled ? 'ON' : 'OFF'}`);
      this._setStyle(elVisBtn, 'color', data.fsdVisEnabled ? '#38bdf8' : '#94a3b8');
      this._setStyle(elVisBtn, 'borderColor', data.fsdVisEnabled ? '#0284c7' : '#475569');
    }

    if (elState) {
      this._setDomText(elState, data.enabled ? data.state.replace('_', ' ') : 'MANUAL');
      const stateColor = !data.enabled ? '#94a3b8' : data.state === 'AEB_BRAKING' ? '#ef4444' : data.state === 'ACC_FOLLOWING' ? '#f59e0b' : '#10b981';
      this._setStyle(elState, 'color', stateColor);
    }

    if (elSpeed) {
      this._setDomText(elSpeed, `${data.speedMph} / ${data.targetSpeedMph} mph`);
    }

    if (elRadar) {
      if (data.leadDist < 70) {
        this._setDomText(elRadar, `${data.leadDist.toFixed(1)} m (${data.closestObstacleName || 'Car'})`);
        this._setStyle(elRadar, 'color', data.leadDist < 12 ? '#ef4444' : '#fbbf24');
      } else {
        this._setDomText(elRadar, 'Clear (>70m)');
        this._setStyle(elRadar, 'color', '#10b981');
      }
    }

    if (elBsmText) {
      const leftStr = data.leftBsmAlert ? '<span style="color:#ef4444;">ALERT</span>' : '<span style="color:#10b981;">CLEAR</span>';
      const rightStr = data.rightBsmAlert ? '<span style="color:#ef4444;">ALERT</span>' : '<span style="color:#10b981;">CLEAR</span>';
      this._setDomHtml(elBsmText, `L: ${leftStr} | R: ${rightStr}`);
    }

    if (elBanner) {
      this._setDomText(elBanner, data.statusMessage);
      if (data.statusMessage.includes('PREVENTED') || data.statusMessage.includes('Blocked') || data.statusMessage.includes('AEB') || data.statusMessage.includes('WATCHDOG')) {
        this._setStyle(elBanner, 'borderLeftColor', '#ef4444');
        this._setStyle(elBanner, 'color', '#fca5a5');
      } else if (data.statusMessage.includes('ACC')) {
        this._setStyle(elBanner, 'borderLeftColor', '#f59e0b');
        this._setStyle(elBanner, 'color', '#fef08a');
      } else {
        this._setStyle(elBanner, 'borderLeftColor', '#06b6d4');
        this._setStyle(elBanner, 'color', '#cbd5e1');
      }
    }

    // Turn Signal Indicators on HUD
    if (elTurnL) {
      this._setStyle(elTurnL, 'color', (data.leftBlinker || data.hazardLights) ? '#f59e0b' : '#475569');
    }
    if (elTurnR) {
      elTurnR.style.color = (data.rightBlinker || data.hazardLights) ? '#f59e0b' : '#334155';
    }

    // Mirror BSM LED Icons
    const bsmL = document.getElementById('bsm-left-led');
    const bsmLText = document.getElementById('bsm-left-text');
    const bsmLIcon = document.getElementById('bsm-left-icon');
    if (bsmL) {
      if (data.leftBsmAlert) {
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
      if (data.rightBsmAlert) {
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
    if (arrowL) arrowL.style.opacity = (data.leftBlinker || data.hazardLights) ? '1' : '0';

    const arrowR = document.getElementById('turn-right-indicator');
    if (arrowR) arrowR.style.opacity = (data.rightBlinker || data.hazardLights) ? '1' : '0';
  }
}
