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
