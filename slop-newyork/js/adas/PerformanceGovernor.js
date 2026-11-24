// js/adas/PerformanceGovernor.js
// ============================================================================
// Adaptive Frame-Budget Governor & Real-Time Performance Telemetry Overlay
// Dynamically adjusts WebGPU DPR, cull scale & mirror update rates for solid 60 FPS.
// Supports quality profiles: AUTO, LOW / i3 MODE, HIGH (?perf=low, ?perf=high).
// ============================================================================

export class PerformanceGovernor {
  constructor(renderer, options = {}) {
    this.renderer = renderer;
    this.targetFrameBudgetMs = 16.66; // 60 FPS target
    this.alpha = 2 / (60 + 1); // 60-frame EMA smoothing
    this.emaFrameMs = 16.0;

    // Quality Profiles: 'AUTO', 'LOW', 'HIGH'
    const urlParams = typeof window !== 'undefined' ? new URLSearchParams(window.location.search) : null;
    const perfParam = urlParams ? urlParams.get('perf') : null;
    if (perfParam === 'low' || perfParam === 'i3') {
      this.profile = 'LOW';
    } else if (perfParam === 'high') {
      this.profile = 'HIGH';
    } else {
      this.profile = options.profile || 'AUTO';
    }

    // Budget Tuning State
    this.baseDpr = Math.min(typeof window !== 'undefined' ? (window.devicePixelRatio || 1) : 1, 1.25);
    this.currentDpr = this.profile === 'LOW' ? 0.75 : 1.0;
    this.minDpr = this.profile === 'LOW' ? 0.60 : 0.75;
    this.maxDpr = this.profile === 'LOW' ? 0.75 : 1.0;
    this.lastAppliedDpr = this.currentDpr;

    this.cullScale = this.profile === 'LOW' ? 0.62 : 1.0; // 0.60 to 1.0
    this.mirrorSkipInterval = this.profile === 'LOW' ? 4 : 2; // 2 = 30 Hz staggered, 4 = 15 Hz staggered

    // Telemetry stats
    this.fps = 60;
    this.drawCalls = 0;
    this.qnxRttUs = 0;
    this.activeObstacles = 0;
    this.triangles = 0;

    // DOM Telemetry Pill
    this.pillElement = null;
    this.visible = true;
    this.compactMode = false;
    this.lastPillUpdate = 0;
    this._lastText = '';

    if (typeof document !== 'undefined') {
      this.createTelemetryPill();
      this.bindKeys();
    }
  }

  setProfile(profile) {
    if (profile !== 'AUTO' && profile !== 'LOW' && profile !== 'HIGH') return;
    this.profile = profile;

    if (this.profile === 'LOW') {
      this.minDpr = 0.60;
      this.maxDpr = 0.75;
      this.currentDpr = 0.75;
      this.cullScale = 0.62;
      this.mirrorSkipInterval = 4;
    } else if (this.profile === 'HIGH') {
      this.minDpr = 0.85;
      this.maxDpr = 1.0;
      this.currentDpr = 1.0;
      this.cullScale = 1.0;
      this.mirrorSkipInterval = 2;
    } else {
      // AUTO
      this.minDpr = 0.75;
      this.maxDpr = 1.0;
      this.mirrorSkipInterval = 2;
    }

    if (this.renderer && typeof this.renderer.setPixelRatio === 'function') {
      this.renderer.setPixelRatio(this.currentDpr);
      this.lastAppliedDpr = this.currentDpr;
    }
    this.updatePill(true);
  }

  cycleProfile() {
    if (this.profile === 'AUTO') this.setProfile('LOW');
    else if (this.profile === 'LOW') this.setProfile('HIGH');
    else this.setProfile('AUTO');
  }

  createTelemetryPill() {
    const pill = document.createElement('div');
    pill.id = 'perf-governor-pill';
    pill.style.position = 'fixed';
    pill.style.top = '14px';
    pill.style.left = '50%';
    pill.style.transform = 'translateX(-50%)';
    pill.style.zIndex = '1000';
    pill.style.display = 'flex';
    pill.style.alignItems = 'center';
    pill.style.gap = '8px';
    pill.style.padding = '4px 12px';
    pill.style.borderRadius = '20px';
    pill.style.background = 'rgba(15, 23, 42, 0.82)';
    pill.style.backdropFilter = 'blur(10px)';
    pill.style.border = '1px solid rgba(255, 255, 255, 0.15)';
    pill.style.boxShadow = '0 6px 20px rgba(0, 0, 0, 0.45)';
    pill.style.fontFamily = 'ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace';
    pill.style.fontSize = '11px';
    pill.style.fontWeight = '600';
    pill.style.color = '#e2e8f0';
    pill.style.cursor = 'pointer';
    pill.style.userSelect = 'none';
    pill.title = 'Click to cycle Quality Profile (AUTO / LOW / HIGH) / Press F3';

    pill.addEventListener('click', (e) => {
      // Click cycles quality profile
      this.cycleProfile();
    });

    document.body.appendChild(pill);
    this.pillElement = pill;
    this.updatePill(true);
  }

  bindKeys() {
    window.addEventListener('keydown', (e) => {
      if (e.code === 'F3') {
        e.preventDefault();
        this.visible = !this.visible;
        if (this.pillElement) {
          this.pillElement.style.display = this.visible ? 'flex' : 'none';
        }
      }
    });
  }

  update(rawDtMs, telemetry = {}) {
    const frameMs = Math.max(1.0, Math.min(100.0, rawDtMs));
    this.emaFrameMs = (1.0 - this.alpha) * this.emaFrameMs + this.alpha * frameMs;
    this.fps = Math.round(1000.0 / Math.max(1.0, this.emaFrameMs));

    if (telemetry.drawCalls !== undefined) this.drawCalls = telemetry.drawCalls;
    if (telemetry.triangles !== undefined) this.triangles = telemetry.triangles;
    if (telemetry.qnxRttUs !== undefined) this.qnxRttUs = telemetry.qnxRttUs;
    if (telemetry.activeObstacles !== undefined) this.activeObstacles = telemetry.activeObstacles;

    // Quality Profile Logic
    if (this.profile === 'LOW') {
      this.currentDpr = 0.75;
      this.cullScale = 0.62;
      this.mirrorSkipInterval = 4;
    } else if (this.profile === 'HIGH') {
      this.currentDpr = 1.0;
      this.cullScale = 1.0;
      this.mirrorSkipInterval = 2;
    } else {
      // Dynamic Frame-Budget Governor Adaptation (AUTO mode)
      if (this.emaFrameMs > 16.66) {
        // Over budget: smooth step down DPR and cull distance
        this.currentDpr = Math.max(this.minDpr, this.currentDpr - 0.004);
        this.cullScale = Math.max(0.70, this.cullScale - 0.005);
        this.mirrorSkipInterval = 3;
      } else if (this.emaFrameMs < 14.5) {
        // Healthy headroom (>68 FPS potential): restore DPR and cull distance
        this.currentDpr = Math.min(this.maxDpr, this.currentDpr + 0.002);
        this.cullScale = Math.min(1.0, this.cullScale + 0.003);
        this.mirrorSkipInterval = 2;
      }
    }

    // Apply DPR changes when stepping beyond 0.04 delta
    if (this.renderer && typeof this.renderer.setPixelRatio === 'function') {
      if (Math.abs(this.currentDpr - this.lastAppliedDpr) >= 0.04) {
        this.lastAppliedDpr = this.currentDpr;
        this.renderer.setPixelRatio(this.currentDpr);
      }
    }

    // Throttled UI Pill Update (10 Hz)
    const now = (typeof performance !== 'undefined' ? performance.now() : Date.now());
    if (now - this.lastPillUpdate >= 100) {
      this.lastPillUpdate = now;
      this.updatePill();
    }

    return {
      profile: this.profile,
      dpr: this.currentDpr,
      cullScale: this.cullScale,
      mirrorSkipInterval: this.mirrorSkipInterval,
      fps: this.fps,
      emaFrameMs: this.emaFrameMs
    };
  }

  updatePill(force = false) {
    if (!this.pillElement || !this.visible) return;

    let text = '';
    const fpsColor = this.fps >= 55 ? '#10b981' : this.fps >= 35 ? '#f59e0b' : '#ef4444';
    const rttText = this.qnxRttUs > 0 ? `${this.qnxRttUs}µs` : 'offline';
    const profColor = this.profile === 'LOW' ? '#f59e0b' : this.profile === 'HIGH' ? '#a855f7' : '#38bdf8';
