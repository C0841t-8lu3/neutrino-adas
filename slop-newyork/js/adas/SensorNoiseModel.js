// js/adas/SensorNoiseModel.js
// ============================================================================
// Realistic ADAS Radar & Camera Sensor Simulation Model
// Models range-dependent Gaussian noise, line-of-sight occlusion, single-frame
// dropouts, and an optional Sensor-Side Alpha-Beta / Kalman Tracker.
// ============================================================================

export class SensorNoiseModel {
  constructor(options = {}) {
    this.enabled = options.enabled || false;
    this.trackerEnabled = options.trackerEnabled !== undefined ? options.trackerEnabled : true;
    this.dropoutRate = options.dropoutRate !== undefined ? options.dropoutRate : 0.02; // 2% single-frame drop
    this.seedValue = options.seed || 12345;
    this.rng = this._createMulberry32(this.seedValue);

    // Scratch buffers to avoid GC allocations
    this._processedObstacles = [];
    this._tracks = new Map(); // trackId -> { x, z, vx, vz, speed, type, misses, age }
    this._nextTrackId = 1;

    // Occlusion beam limits (FOV +/- 45 deg, max range 120m)
    this.maxRadarRange = 120.0;
    this.maxRadarFovRad = Math.PI * 0.25; // 45 deg half-FOV

    // Metrics for HUD diagnostics
    this.lastDroppedCount = 0;
    this.lastOccludedCount = 0;
    this.lastRawCount = 0;
    this.lastFilteredCount = 0;
  }

  _createMulberry32(seed) {
    let s = (seed >>> 0) || 12345;
    return () => {
      let t = (s += 0x6d2b79f5);
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  seed(newSeed) {
    this.seedValue = newSeed;
    this.rng = this._createMulberry32(newSeed);
  }

  gaussian(mean = 0, std = 1) {
    let u = 0, v = 0;
    while (u === 0) u = this.rng();
    while (v === 0) v = this.rng();
    const z0 = Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);
    return mean + z0 * std;
  }

  setRealistic(enabled) {
    this.enabled = Boolean(enabled);
    if (!this.enabled) {
      this._tracks.clear();
    }
  }

  setTracker(enabled) {
    this.trackerEnabled = Boolean(enabled);
  }

  setDropoutRate(rate) {
    this.dropoutRate = Math.max(0.0, Math.min(1.0, rate));
  }

  reset() {
    this._tracks.clear();
    this.lastDroppedCount = 0;
    this.lastOccludedCount = 0;
    this.lastRawCount = 0;
    this.lastFilteredCount = 0;
    this.seed(this.seedValue);
