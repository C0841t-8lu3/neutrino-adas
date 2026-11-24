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

