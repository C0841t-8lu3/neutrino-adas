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
