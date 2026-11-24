// js/adas/ActuatorDynamics.js
// ============================================================================
// Realistic Physical Actuator Dynamics & Tire-Road Friction Adhesion Model
// Simulates steering lag (~90ms), throttle response (~100ms), hydraulic brake
// pressure buildup (~85ms), physical steering-rate limits, and surface friction.
// ============================================================================

export class ActuatorDynamics {
  constructor(options = {}) {
    // Actuator first-order time constants (seconds)
    this.steerLagTau = options.steerLagTau !== undefined ? options.steerLagTau : 0.090; // 90ms
    this.throttleLagTau = options.throttleLagTau !== undefined ? options.throttleLagTau : 0.100; // 100ms
    this.brakeLagTau = options.brakeLagTau !== undefined ? options.brakeLagTau : 0.085; // 85ms

    // Physical rate limits
    this.steerRateLimit = options.steerRateLimit !== undefined ? options.steerRateLimit : 0.85; // rad/s max wheel turn rate

    // Road surface friction coefficient (mu)
    // Dry Asphalt = 0.95, Wet Road = 0.55
    this.surfaceType = options.surfaceType || 'dry';
    this.frictionMu = this.surfaceType === 'wet' ? 0.55 : 0.95;

    // Actual physical actuator states
    this.actualSteer = 0.0;
    this.actualThrottle = 0.0;
    this.actualBrake = 0.0;

    // Active bypass flag (e.g. for deterministic unit benchmarks)
    this.enabled = options.enabled !== undefined ? options.enabled : true;
  }

  setSurface(type) {
    if (type === 'wet') {
      this.surfaceType = 'wet';
      this.frictionMu = 0.55;
    } else {
      this.surfaceType = 'dry';
      this.frictionMu = 0.95;
    }
  }

  setFriction(mu) {
    this.frictionMu = Math.max(0.1, Math.min(1.2, mu));
    this.surfaceType = this.frictionMu < 0.75 ? 'wet' : 'dry';
  }

  setEnabled(enabled) {
    this.enabled = Boolean(enabled);
  }

  reset() {
    this.actualSteer = 0.0;
    this.actualThrottle = 0.0;
    this.actualBrake = 0.0;
  }

  /**
   * Applies first-order physical lag, rate limiting, and road adhesion.
   *
   * @param {number} cmdThrottle QNX commanded throttle (0.0 to 1.0)
   * @param {number} cmdBrake QNX commanded brake (0.0 to 1.0)
   * @param {number} cmdSteer QNX commanded steering (-1.0 to +1.0)
   * @param {number} dt Time step (seconds)
   * @returns {{ throttle: number, brake: number, steer: number, frictionMu: number }}
   */
  step(cmdThrottle, cmdBrake, cmdSteer, dt = 0.016) {
    if (!this.enabled) {
      this.actualThrottle = cmdThrottle;
      this.actualBrake = cmdBrake;
      this.actualSteer = cmdSteer;
      return {
        throttle: cmdThrottle,
        brake: cmdBrake,
        steer: cmdSteer,
        frictionMu: this.frictionMu
      };
    }

    const safeDt = Math.max(0.001, Math.min(0.05, dt));

    // 1. Steering First-Order Lag + Physical Rate Limit
    const targetSteer = Math.max(-1.0, Math.min(1.0, cmdSteer));
    const steerAlpha = 1.0 - Math.exp(-safeDt / Math.max(0.01, this.steerLagTau));
    let rawDeltaSteer = (targetSteer - this.actualSteer) * steerAlpha;
