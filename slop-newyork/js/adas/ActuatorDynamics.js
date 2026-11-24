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

    // Apply physical steering-rate clamping
    const maxSteerStep = this.steerRateLimit * safeDt;
    rawDeltaSteer = Math.max(-maxSteerStep, Math.min(maxSteerStep, rawDeltaSteer));
    this.actualSteer += rawDeltaSteer;

    // 2. Powertrain Throttle Lag
    const targetThrottle = Math.max(0.0, Math.min(1.0, cmdThrottle));
    const throttleAlpha = 1.0 - Math.exp(-safeDt / Math.max(0.01, this.throttleLagTau));
    this.actualThrottle += (targetThrottle - this.actualThrottle) * throttleAlpha;

    // 3. Hydraulic Brake Buildup Lag + Friction-Limited Braking Authority
    const targetBrake = Math.max(0.0, Math.min(1.0, cmdBrake));
    const brakeAlpha = 1.0 - Math.exp(-safeDt / Math.max(0.01, this.brakeLagTau));
    this.actualBrake += (targetBrake - this.actualBrake) * brakeAlpha;

    // On low-friction wet surfaces, effective braking deceleration is scaled by (mu / 0.95)
    // requiring longer stopping distance
    const effectiveBrake = this.actualBrake * (this.frictionMu / 0.95);

    return {
      throttle: this.actualThrottle,
      brake: effectiveBrake,
      steer: this.actualSteer,
      frictionMu: this.frictionMu
    };
  }
}
