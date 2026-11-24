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
    if (btnToggle) btnToggle.addEventListener('click', () => this.callbacks.onToggle && this.callbacks.onToggle());

    const btnLaneL = document.getElementById('btn-lane-left');
    if (btnLaneL) btnLaneL.addEventListener('click', () => this.callbacks.onLaneChangeLeft && this.callbacks.onLaneChangeLeft());

    const btnLaneR = document.getElementById('btn-lane-right');
    if (btnLaneR) btnLaneR.addEventListener('click', () => this.callbacks.onLaneChangeRight && this.callbacks.onLaneChangeRight());
  }

  setVisibility(visible) {
    const disp = visible ? 'block' : 'none';
    if (this.hudElement) this.hudElement.style.display = disp;
    if (this.leftMirrorBox) this.leftMirrorBox.style.display = disp;
    if (this.rightMirrorBox) this.rightMirrorBox.style.display = disp;
  }

  updateHud(data) {
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
      elDot.style.background = data.qnxConnected ? '#10b981' : '#ef4444';
      elDot.style.boxShadow = data.qnxConnected ? '0 0 8px #10b981' : '0 0 8px #ef4444';
    }

    if (elBtn) {
      elBtn.textContent = data.enabled ? 'ON [P]' : 'OFF [P]';
      elBtn.style.background = data.enabled ? '#06b6d4' : '#475569';
    }

    if (elState) {
      elState.textContent = data.enabled ? data.state.replace('_', ' ') : 'MANUAL';
      if (!data.enabled) {
        elState.style.color = '#94a3b8';
      } else if (data.state === 'AEB_BRAKING') {
        elState.style.color = '#ef4444';
      } else if (data.state === 'ACC_FOLLOWING') {
        elState.style.color = '#f59e0b';
      } else {
        elState.style.color = '#10b981';
      }
    }

    if (elSpeed) {
      elSpeed.textContent = `${data.speedMph} / ${data.targetSpeedMph} mph`;
    }

    if (elRadar) {
      if (data.leadDist < 70) {
        elRadar.textContent = `${data.leadDist.toFixed(1)} m (${data.closestObstacleName || 'Car'})`;
        elRadar.style.color = data.leadDist < 12 ? '#ef4444' : '#fbbf24';
      } else {
        elRadar.textContent = 'Clear (>70m)';
        elRadar.style.color = '#10b981';
      }
    }

    if (elBsmText) {
      const leftStr = data.leftBsmAlert ? '<span style="color:#ef4444;">ALERT</span>' : '<span style="color:#10b981;">CLEAR</span>';
      const rightStr = data.rightBsmAlert ? '<span style="color:#ef4444;">ALERT</span>' : '<span style="color:#10b981;">CLEAR</span>';
      elBsmText.innerHTML = `L: ${leftStr} | R: ${rightStr}`;
    }

    if (elBanner) {
      elBanner.textContent = data.statusMessage;
      if (data.statusMessage.includes('PREVENTED') || data.statusMessage.includes('Blocked')) {
        elBanner.style.borderLeftColor = '#ef4444';
        elBanner.style.color = '#fca5a5';
      } else if (data.statusMessage.includes('AEB')) {
        elBanner.style.borderLeftColor = '#ef4444';
        elBanner.style.color = '#fca5a5';
      } else if (data.statusMessage.includes('ACC')) {
        elBanner.style.borderLeftColor = '#f59e0b';
        elBanner.style.color = '#fef08a';
      } else {
        elBanner.style.borderLeftColor = '#06b6d4';
        elBanner.style.color = '#cbd5e1';
      }
    }

    // Turn Signal Indicators on HUD
    if (elTurnL) {
      elTurnL.style.color = (data.leftBlinker || data.hazardLights) ? '#f59e0b' : '#334155';
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
