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
          actor.dl = 3.6;
          actor.dlv = -2.6; // fast lateral merge rate towards ego line
          this._injectedActor = actor;
        }
        this.activeScenario = {
          id: 1,
          name: 'Cut-In Hazard',
          duration: 4.5,
          startTime: (typeof performance !== 'undefined' ? performance.now() : Date.now())
        };
        this.lastStatus = 'Scenario 1: Cut-In Hazard Active';
        break;
      }

      case 2: {
        // Scenario 2: Sudden Pedestrian Jaywalk
        // Sidewalk walker darts across path at +16m fwd, +6m lat -> 0m lat at 2.8 m/s
        const ped = this.findOrBorrowPedestrian();
        if (ped) {
          ped.placed = true;
          ped.x = px + 16.0 * sin + 6.0 * cos;
          ped.z = pz + 16.0 * cos - 6.0 * sin;
          ped.speed = 2.8;
          ped.v = 2.8;
          // Vector pointing across ego corridor (from right to left)
          ped.ux = -cos;
          ped.uz = sin;
          ped.dir = 1;
          ped.crossesStreets = true;
          this._injectedActor = ped;
        }
        this.activeScenario = {
          id: 2,
          name: 'Pedestrian Jaywalk (AEB + Horn)',
          duration: 5.0,
          startTime: (typeof performance !== 'undefined' ? performance.now() : Date.now())
        };
        this.lastStatus = 'Scenario 2: Pedestrian Jaywalk Active';
        break;
      }

      case 3: {
        // Scenario 3: Lead Emergency Brake (Euro NCAP CCRb)
        // Commands lead vehicle 18m ahead to slam on brakes at -6.0 m/s²
        const actor = this.findOrBorrowVehicle();
        if (actor) {
          actor.x = px + 18.0 * sin + 0.0 * cos;
          actor.z = pz + 18.0 * cos - 0.0 * sin;
          actor.speed = egoSpeed;
          actor.v = actor.speed;
          actor.yaw = ego.heading;
          actor.tx = sin;
          actor.tz = cos;
          actor.dl = 0.0;
          actor.dlv = 0.0;
          actor.panicAcc = -6.0;
          actor.panicUntil = (typeof performance !== 'undefined' ? performance.now() : Date.now()) + 5000;
          this._injectedActor = actor;
        }
        this.activeScenario = {
          id: 3,
          name: 'Lead Emergency Brake (CCRb)',
          duration: 4.0,
          startTime: (typeof performance !== 'undefined' ? performance.now() : Date.now())
        };
        this.lastStatus = 'Scenario 3: Lead CCRb Emergency Brake Active';
        break;
      }

      case 4: {
        // Scenario 4: Blind-Spot Ambush
        // Positions car in left rear blind spot (-3.5m fwd, -3.2m lat), matching ego speed,
        // and requests an ego lane change to verify QNX BSM interlock prevention
        const actor = this.findOrBorrowVehicle();
        if (actor) {
          actor.x = px - 3.5 * sin - 3.2 * cos;
          actor.z = pz - 3.5 * cos + 3.2 * sin;
          actor.speed = egoSpeed * 1.02;
          actor.v = actor.speed;
          actor.yaw = ego.heading;
          actor.tx = sin;
          actor.tz = cos;
          actor.dl = -3.2;
          actor.dlv = 0.0;
          this._injectedActor = actor;
        }
        if (this.autonomousDriver) {
          // Command lane change towards the blind spot
          this.autonomousDriver.requestLaneChange(-1);
        }
        this.activeScenario = {
          id: 4,
          name: 'Blind-Spot Ambush (BSM Abort)',
          duration: 4.5,
          startTime: (typeof performance !== 'undefined' ? performance.now() : Date.now())
        };
        this.lastStatus = 'Scenario 4: Blind-Spot Ambush Active (BSM Check)';
        break;
      }

      case 5: {
        // Scenario 5: Sensor Dropout / Wet Road Stress Test
        // Combines wet-road friction (μ = 0.55) with realistic radar noise & 15% dropout
        if (this.autonomousDriver) {
          if (this.autonomousDriver.actuatorDynamics) {
            this.autonomousDriver.actuatorDynamics.setSurface('wet');
          }
          if (this.autonomousDriver.perceptionScanner?.sensorNoiseModel) {
            const snm = this.autonomousDriver.perceptionScanner.sensorNoiseModel;
            snm.setRealistic(true);
            snm.setDropoutRate(0.15);
          }
        }
        this.activeScenario = {
          id: 5,
          name: 'Wet Road + Radar Dropout Stress Test',
          duration: 10.0,
          startTime: (typeof performance !== 'undefined' ? performance.now() : Date.now())
        };
        this.lastStatus = 'Scenario 5: Wet Road (μ=0.55) + 15% Dropout Active';
        break;
      }

      default:
        return false;
    }

    this.scenarioTimer = 0;
    this.updatePanel();
    return true;
  }

  cancelActiveScenario() {
    if (!this.activeScenario) return;

    if (this.activeScenario.id === 5 && this.autonomousDriver) {
      if (this.autonomousDriver.actuatorDynamics) {
        this.autonomousDriver.actuatorDynamics.setSurface('dry');
      }
      if (this.autonomousDriver.perceptionScanner?.sensorNoiseModel) {
        this.autonomousDriver.perceptionScanner.sensorNoiseModel.setDropoutRate(0.02);
      }
    }

    this.activeScenario = null;
    this._injectedActor = null;
    this.lastStatus = 'Scenario Complete / Ready';
    this.updatePanel();
  }

  findOrBorrowVehicle() {
    if (!this.trafficSystem?.cars || this.trafficSystem.cars.length === 0) return null;
    return this.trafficSystem.cars[0];
  }

  findOrBorrowPedestrian() {
    if (!this.trafficSystem?.peds || this.trafficSystem.peds.length === 0) return null;
    return this.trafficSystem.peds[0];
  }

  update(dt) {
    // 1. Progress active test scenario
    if (this.activeScenario) {
      this.scenarioTimer += dt;

      // Handle scenario-specific actor dynamics
      if (this.activeScenario.id === 1 && this._injectedActor) {
        // Cut-In: vehicle slides inward laterally towards 0
        if (this._injectedActor.dl > 0.05) {
          this._injectedActor.dl += this._injectedActor.dlv * dt;
        } else {
          this._injectedActor.dl = 0;
          this._injectedActor.dlv = 0;
        }
      } else if (this.activeScenario.id === 2 && this._injectedActor) {
        // Jaywalker continues across lane
        this._injectedActor.x += this._injectedActor.ux * this._injectedActor.v * dt;
        this._injectedActor.z += this._injectedActor.uz * this._injectedActor.v * dt;
      } else if (this.activeScenario.id === 3 && this._injectedActor) {
        // Hard braking: decelerate at 6 m/s² down to 0
        this._injectedActor.speed = Math.max(0.0, this._injectedActor.speed - 6.0 * dt);
        this._injectedActor.v = this._injectedActor.speed;
        this._injectedActor.x += this._injectedActor.tx * this._injectedActor.speed * dt;
        this._injectedActor.z += this._injectedActor.tz * this._injectedActor.speed * dt;
      } else if (this.activeScenario.id === 4 && this._injectedActor) {
        // Pace vehicle keeping position relative to ego
        const ego = this.getEgoCar();
        if (ego) {
          const sin = Math.sin(ego.heading);
          const cos = Math.cos(ego.heading);
          this._injectedActor.x = ego.position.x - 3.5 * sin - 3.2 * cos;
          this._injectedActor.z = ego.position.z - 3.5 * cos + 3.2 * sin;
          this._injectedActor.speed = ego.speed;
          this._injectedActor.v = ego.speed;
        }
      }

      if (this.scenarioTimer >= this.activeScenario.duration) {
        this.cancelActiveScenario();
      }
    }

    // 2. Stochastic Unpredictable Traffic Layer
    if (this.unpredictableEnabled && this.trafficSystem) {
      const ego = this.getEgoCar();
      const px = ego ? ego.position.x : 0;
      const pz = ego ? ego.position.z : 0;

      // Unpredictable Vehicles
      if (this.trafficSystem.cars) {
        const cars = this.trafficSystem.cars;
        const count = Math.min(cars.length, 30);
        for (let i = 0; i < count; i++) {
          const c = cars[i];
          if (!c || c === this._injectedActor) continue;
          const dx = c.x - px;
          const dz = c.z - pz;
          if (dx * dx + dz * dz < 3600) { // within 60m
            // Random small speed variance (±0.4 m/s)
            c.speed = Math.max(2.0, c.speed + (Math.random() - 0.5) * 0.4 * dt * 60);
            if (c.v !== undefined) c.v = c.speed;

            // Occasional minor lane swerve
            if (Math.random() < 0.01) {
              c.dl = (Math.random() - 0.5) * 0.6;
            }
          }
        }
      }

      // Unpredictable Pedestrians
      if (this.trafficSystem.peds) {
        const peds = this.trafficSystem.peds;
        const count = Math.min(peds.length, 25);
        for (let i = 0; i < count; i++) {
          const p = peds[i];
          if (!p || p === this._injectedActor || !p.placed) continue;
          const dx = p.x - px;
          const dz = p.z - pz;
          if (dx * dx + dz * dz < 1600) { // within 40m
            // Hesitation probability
            if (Math.random() < 0.005) {
              p.v = Math.random() < 0.5 ? 0.0 : p.speed;
            }
          }
        }
      }
    }
  }

  createScenarioPanel() {
    if (typeof document === 'undefined' || typeof document.createElement !== 'function' || !document.body) return;

    const panel = document.createElement('div');
    panel.id = 'adas-scenario-panel';
    panel.style.position = 'fixed';
    panel.style.bottom = '24px';
    panel.style.left = '16px';
    panel.style.zIndex = '999';
    panel.style.padding = '8px 12px';
    panel.style.borderRadius = '12px';
    panel.style.background = 'rgba(15, 23, 42, 0.88)';
    panel.style.backdropFilter = 'blur(10px)';
    panel.style.border = '1px solid rgba(255, 255, 255, 0.16)';
    panel.style.boxShadow = '0 8px 24px rgba(0, 0, 0, 0.5)';
    panel.style.fontFamily = 'ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace';
    panel.style.fontSize = '11px';
    panel.style.color = '#e2e8f0';
    panel.style.display = 'flex';
    panel.style.flexDirection = 'column';
    panel.style.gap = '6px';
    panel.style.userSelect = 'none';

    document.body.appendChild(panel);
    this.panelElement = panel;
    this.updatePanel();
  }

  updatePanel() {
    if (!this.panelElement) return;

    const activeText = this.activeScenario
      ? `<span style="color:#f59e0b;font-weight:700;">[RUNNING: ${this.activeScenario.name} (${Math.max(0, this.activeScenario.duration - this.scenarioTimer).toFixed(1)}s)]</span>`
      : `<span style="color:#10b981;">[STATUS: ${this.lastStatus}]</span>`;

    const unpredText = this.unpredictableEnabled
      ? '<span style="color:#ef4444;font-weight:700;">ON</span>'
      : '<span style="color:#94a3b8;">OFF</span>';

    this.panelElement.innerHTML = `
      <div style="display:flex;align-items:center;justify-content:space-between;gap:12px;">
        <span style="font-weight:700;color:#38bdf8;">TEST SCENARIOS</span>
        ${activeText}
      </div>
      <div style="display:flex;gap:4px;flex-wrap:wrap;">
        <button id="scen-btn-1" style="background:#1e293b;border:1px solid #475569;color:#e2e8f0;padding:3px 7px;border-radius:6px;cursor:pointer;font-size:10px;">1: Cut-In</button>
        <button id="scen-btn-2" style="background:#1e293b;border:1px solid #475569;color:#e2e8f0;padding:3px 7px;border-radius:6px;cursor:pointer;font-size:10px;">2: Jaywalk</button>
        <button id="scen-btn-3" style="background:#1e293b;border:1px solid #475569;color:#e2e8f0;padding:3px 7px;border-radius:6px;cursor:pointer;font-size:10px;">3: CCRb Brake</button>
        <button id="scen-btn-4" style="background:#1e293b;border:1px solid #475569;color:#e2e8f0;padding:3px 7px;border-radius:6px;cursor:pointer;font-size:10px;">4: BSM Ambush</button>
        <button id="scen-btn-5" style="background:#1e293b;border:1px solid #475569;color:#e2e8f0;padding:3px 7px;border-radius:6px;cursor:pointer;font-size:10px;">5: Wet Stress</button>
        <button id="scen-btn-unpred" style="background:#1e293b;border:1px solid #475569;color:#e2e8f0;padding:3px 7px;border-radius:6px;cursor:pointer;font-size:10px;">Chaos: ${unpredText}</button>
      </div>
    `;

    const b1 = this.panelElement.querySelector('#scen-btn-1');
    const b2 = this.panelElement.querySelector('#scen-btn-2');
    const b3 = this.panelElement.querySelector('#scen-btn-3');
    const b4 = this.panelElement.querySelector('#scen-btn-4');
    const b5 = this.panelElement.querySelector('#scen-btn-5');
    const bUn = this.panelElement.querySelector('#scen-btn-unpred');

    if (b1) b1.onclick = () => this.triggerScenario(1);
    if (b2) b2.onclick = () => this.triggerScenario(2);
    if (b3) b3.onclick = () => this.triggerScenario(3);
    if (b4) b4.onclick = () => this.triggerScenario(4);
    if (b5) b5.onclick = () => this.triggerScenario(5);
    if (bUn) bUn.onclick = () => this.toggleUnpredictableTraffic();
  }

  bindKeys() {
    if (typeof window === 'undefined') return;
    window.addEventListener('keydown', (e) => {
      // Avoid firing when typing in an input
      if (e.target && (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA')) return;

      if (e.code === 'Digit1') this.triggerScenario(1);
      else if (e.code === 'Digit2') this.triggerScenario(2);
      else if (e.code === 'Digit3') this.triggerScenario(3);
      else if (e.code === 'Digit4') this.triggerScenario(4);
      else if (e.code === 'Digit5') this.triggerScenario(5);
    });
  }

  destroy() {
    if (this.panelElement && this.panelElement.parentNode) {
      this.panelElement.parentNode.removeChild(this.panelElement);
    }
  }
}
