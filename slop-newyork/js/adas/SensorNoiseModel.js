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
  }

  /**
   * Applies realistic sensor degradation: line-of-sight occlusion, range noise,
   * stochastic packet dropouts, and optional Alpha-Beta state estimation.
   */
  process(rawObstacles, ego, dt = 0.016) {
    this.lastRawCount = rawObstacles ? rawObstacles.length : 0;
    if (!this.enabled || !rawObstacles || rawObstacles.length === 0) {
      this.lastDroppedCount = 0;
      this.lastOccludedCount = 0;
      this.lastFilteredCount = this.lastRawCount;
      return rawObstacles || [];
    }

    const egoX = ego.position ? ego.position.x : (ego.x || 0);
    const egoZ = ego.position ? ego.position.z : (ego.z || 0);
    const egoHeading = ego.heading || 0;
    const cosH = Math.cos(egoHeading);
    const sinH = Math.sin(egoHeading);

    // 1. Transform obstacles into Ego-Relative Polar Coordinates
    const candidates = [];
    for (let i = 0; i < rawObstacles.length; i++) {
      const o = rawObstacles[i];
      const dx = o.x - egoX;
      const dz = o.z - egoZ;

      // Longitudinal (forward) and lateral (rightward) in ego frame
      const forwardDist = (dx * sinH) + (dz * cosH);
      const lateralDist = (dx * cosH) - (dz * sinH);

      // Radar range and relative azimuth
      const range = Math.hypot(forwardDist, lateralDist);
      if (forwardDist < -2.0 || range > this.maxRadarRange) continue; // Behind ego or beyond max range

      const azimuth = Math.atan2(lateralDist, forwardDist);
      if (Math.abs(azimuth) > this.maxRadarFovRad) continue; // Outside FOV

      const halfW = (o.type === 1 ? 1.4 : o.type === 2 ? 0.4 : 1.0);
      const angWidth = Math.atan2(halfW, Math.max(1.0, forwardDist));

      candidates.push({
        orig: o,
        forwardDist,
        lateralDist,
        range,
        azimuth,
        angMin: azimuth - angWidth,
        angMax: azimuth + angWidth,
        occluded: false,
        dropped: false
      });
    }

    // 2. Line-of-Sight Occlusion Check (Closer objects shadow farther objects behind them)
    candidates.sort((a, b) => a.range - b.range);
    let occludedCount = 0;
    const shadowFrustums = [];

    for (let i = 0; i < candidates.length; i++) {
      const c = candidates[i];
      // Check if candidate is covered by any closer shadow frustum
      for (let j = 0; j < shadowFrustums.length; j++) {
        const s = shadowFrustums[j];
        if (c.azimuth >= s.min && c.azimuth <= s.max) {
          c.occluded = true;
          occludedCount++;
          break;
        }
      }

      if (!c.occluded) {
        // Broaden shadow for vehicles / trucks to occlude objects directly behind them
        const shadowSpan = (c.orig.type === 1) ? 0.06 : 0.04;
        shadowFrustums.push({
          min: c.azimuth - shadowSpan,
          max: c.azimuth + shadowSpan
        });
      }
    }
    this.lastOccludedCount = occludedCount;

    // 3. Packet Dropout & Range-Dependent Gaussian Measurement Noise
    let droppedCount = 0;
    const measuredList = [];

    for (let i = 0; i < candidates.length; i++) {
      const c = candidates[i];
      if (c.occluded) continue;

      // Stochastic radar reflection drop
      if (this.rng() < this.dropoutRate) {
        c.dropped = true;
        droppedCount++;
        continue;
      }

      // Range-dependent noise: farther objects have wider uncertainty
      const sigmaZ = 0.15 + 0.012 * c.range;
      const sigmaX = 0.12 + 0.006 * c.range;
      const sigmaV = 0.25;

      const noisyForward = Math.max(0.5, c.forwardDist + this.gaussian(0, sigmaZ));
      const noisyLateral = c.lateralDist + this.gaussian(0, sigmaX);
      const noisySpeed = Math.max(0, (c.orig.speed || 0) + this.gaussian(0, sigmaV));

      // Reconstruct noisy world coordinates
      const noisyWorldX = egoX + (noisyLateral * cosH) + (noisyForward * sinH);
      const noisyWorldZ = egoZ + (-noisyLateral * sinH) + (noisyForward * cosH);

      measuredList.push({
        orig: c.orig,
        x: Math.round(noisyWorldX * 100) / 100,
        z: Math.round(noisyWorldZ * 100) / 100,
