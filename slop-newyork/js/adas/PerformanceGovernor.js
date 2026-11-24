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
