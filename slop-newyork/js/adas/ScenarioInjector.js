// js/adas/ScenarioInjector.js
// ============================================================================
// Euro NCAP Active Safety Scenario Injector & Stochastic Traffic Hazard Layer
// Simulates realistic high-risk driving scenarios:
//   1. Cut-In Hazard (adjacent lane abrupt merge)
//   2. Sudden Pedestrian Jaywalk (darting across corridor, tests AEB + Horn)
//   3. Lead Emergency Brake (CCRb -6.0 m/s² hard braking)
//   4. Blind-Spot Ambush (rear-quarter vehicle + commanded lane change -> BSM abort)
//   5. Sensor Dropout / Wet Road Stress Test (μ = 0.55 + 15% radar dropout)
// ============================================================================

export class ScenarioInjector {
  constructor(options = {}) {
    this.trafficSystem = options.trafficSystem || null;
    this.autonomousDriver = options.autonomousDriver || null;
    this.controllerManager = options.controllerManager || null;
    this.routePlanner = options.routePlanner || null;

    this.unpredictableEnabled = false;
    this.activeScenario = null;
    this.scenarioTimer = 0;
    this.lastStatus = 'Ready';
    this.panelElement = null;
    this.visible = true;

    // Scratch vector and target references
    this._injectedActor = null;

    if (typeof document !== 'undefined' && typeof window !== 'undefined') {
      this.createScenarioPanel();
      this.bindKeys();
    }
  }

  setTrafficSystem(trafficSystem) {
    this.trafficSystem = trafficSystem;
  }

  setAutonomousDriver(autonomousDriver) {
    this.autonomousDriver = autonomousDriver;
  }

  toggleUnpredictableTraffic(forceState = null) {
    this.unpredictableEnabled = forceState !== null ? Boolean(forceState) : !this.unpredictableEnabled;
    this.lastStatus = `Unpredictable Traffic: ${this.unpredictableEnabled ? 'ON' : 'OFF'}`;
    this.updatePanel();
    return this.unpredictableEnabled;
  }

  getEgoCar() {
    return this.controllerManager?.entities?.car || null;
  }

  /**
   * Triggers one of the 5 Euro NCAP ADAS safety test scenarios.
   * @param {number} id Scenario ID (1 to 5)
   */
  triggerScenario(id) {
    this.cancelActiveScenario();
    const ego = this.getEgoCar();
    if (!ego && id !== 5) {
      this.lastStatus = 'Error: Ego car not found';
      this.updatePanel();
      return false;
    }

    const sin = ego ? Math.sin(ego.heading) : 0;
    const cos = ego ? Math.cos(ego.heading) : 1;
    const px = ego ? ego.position.x : 0;
    const pz = ego ? ego.position.z : 0;
    const egoSpeed = ego ? Math.max(5.0, ego.speed) : 10.0;

    switch (id) {
      case 1: {
        // Scenario 1: Cut-In Hazard
        // Spawns/commands adjacent car at +14m fwd, +3.6m lat, merging aggressively into ego lane
        const actor = this.findOrBorrowVehicle();
        if (actor) {
          actor.x = px + 14.0 * sin + 3.6 * cos;
          actor.z = pz + 14.0 * cos - 3.6 * sin;
          actor.speed = egoSpeed * 1.05;
          actor.v = actor.speed;
          actor.yaw = ego.heading;
          actor.tx = sin;
          actor.tz = cos;
