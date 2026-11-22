// js/driving-metrics.js
// ============================================================================
// QNX 8.0 ADAS DRIVING REALISM & SAFETY BENCHMARK MODULE
//
// Instruments and logs concrete, quantitative driving metrics:
// 1. Lane-keeping offset (mean, max, RMS distance from lane center over time)
// 2. Steering angle vs. road curvature (over/understeering error)
// 3. Braking distance before obstacles / intersections (min gap & harsh braking)
// 4. Collision count & near-miss events (TTC < 1.5s)
// 5. Traffic light & intersection signal compliance
// ============================================================================

import { signalPhase } from './city-config.js';

export class DrivingMetricsRecorder {
  constructor(controllerManager, trafficSystem) {
    this.controllerManager = controllerManager;
    this.trafficSystem = trafficSystem;

    // Benchmark Route Configuration
    this.routeDuration = 25.0; // Fixed duration in seconds
    this.startPos = { x: 3.6, y: 0.0, z: 410.6, heading: 0.0 }; // 5th Ave Southbound (37th St)
    this.targetLaneX = 3.6; // Lane 1 center on 5th Ave

    // State
    this.running = false;
    this.routeTime = 0;
    this.frameCount = 0;
    this.completed = false;
    this.prevSpeed = 0;

    // 1. Lane Keeping Metrics
    this.laneOffsets = []; // absolute offsets (m)
    this.laneOffsetsSigned = []; // signed offsets (m)
    this.maxLaneOffset = 0;
    this.laneOffsetSumSq = 0;

    // 2. Steering & Curvature Metrics
    this.steerErrors = []; // degrees
    this.headingErrors = []; // degrees
    this.maxSteerError = 0;

    // 3. Braking & Clearance Metrics
    this.minObstacleClearance = 999.0;
    this.harshBrakingCount = 0;
    this.harshBrakeCooldown = 0;

    // 4. Collisions & Near-Misses
    this.collisionCount = 0;
    this.nearMissCount = 0;
    this.nearMissCooldown = 0;
    this.curbDepartureActive = false;

    // 5. Traffic Light / Signal Compliance
    this.intersectionsEncountered = 0;
    this.redLightViolations = 0;
    this.signalComplianceObserved = new Set();

    // Summary Result Object
    this.lastReport = null;

    // DOM HUD Element
    this.hudElement = null;
    this.createHud();
    this.hookCollisions();

    // Auto-start benchmark if URL has ?benchmark=1 or ?test_route=1
    this.autoStartRequested = false;
    this._warmupFrames = 0;
    const params = new URLSearchParams(window.location.search);
    this.timeOffset = parseFloat(params.get('time_offset') || '0.0');
    if (this.timeOffset > 0) {
      window.__BENCHMARK_TIME_OFFSET__ = this.timeOffset;
    }
    if (params.has('benchmark') || params.has('test_route')) {
      const dur = parseFloat(params.get('duration') || '25.0');
      if (dur > 0) this.routeDuration = dur;
      this.autoStartRequested = true;
      window.__BENCHMARK_STATUS__ = 'STARTUP_PENDING';
      console.log(`[BENCHMARK] Auto-start armed for route duration ${this.routeDuration}s (time_offset: ${this.timeOffset}s)`);
    }
  }

  hookCollisions() {
    // Intercept controller collision events
    const origListener = this.controllerManager._collisionListener;
    this.controllerManager.setCollisionListener((event) => {
      if (this.running && event.speed > 0.5) {
        this.collisionCount++;
        console.warn(`[METRICS] Collision detected: kind=${event.kind}, speed=${event.speed.toFixed(2)} m/s, total=${this.collisionCount}`);
      }
      if (typeof origListener === 'function') origListener(event);
    });
  }

  createHud() {
    const hud = document.createElement('div');
    hud.id = 'metrics-benchmark-hud';
    hud.style.position = 'fixed';
    hud.style.top = '72px';
    hud.style.left = '245px';
    hud.style.zIndex = '999';
    hud.style.background = 'rgba(15, 23, 42, 0.92)';
    hud.style.backdropFilter = 'blur(12px)';
    hud.style.border = '1px solid rgba(56, 189, 248, 0.35)';
    hud.style.borderRadius = '12px';
    hud.style.padding = '12px 16px';
    hud.style.color = '#f8fafc';
    hud.style.fontFamily = 'monospace';
    hud.style.fontSize = '11px';
    hud.style.minWidth = '290px';
    hud.style.boxShadow = '0 12px 30px rgba(0,0,0,0.7)';
    hud.style.pointerEvents = 'auto';

    hud.innerHTML = `
      <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:8px; border-bottom:1px solid #334155; padding-bottom:6px;">
        <span style="font-weight:800; color:#38bdf8; letter-spacing:0.05em;">QNX ADAS BENCHMARK</span>
        <button id="btn-run-benchmark" style="cursor:pointer; background:#0284c7; color:#fff; border:none; border-radius:4px; padding:2px 8px; font-size:10px; font-weight:700;">
          RUN TEST
        </button>
      </div>
      <div style="display:flex; justify-content:space-between; margin-bottom:3px;">
        <span style="color:#94a3b8;">Route Time:</span>
        <span id="m-time" style="font-weight:700; color:#f1f5f9;">0.0 / 25.0s</span>
      </div>
      <div style="display:flex; justify-content:space-between; margin-bottom:3px;">
        <span style="color:#94a3b8;">Lane Offset (RMS / Max):</span>
        <span id="m-lane" style="font-weight:700; color:#38bdf8;">0.00 / 0.00 m</span>
      </div>
      <div style="display:flex; justify-content:space-between; margin-bottom:3px;">
        <span style="color:#94a3b8;">Steer Error (Mean / Max):</span>
        <span id="m-steer" style="font-weight:700; color:#38bdf8;">0.0° / 0.0°</span>
      </div>
      <div style="display:flex; justify-content:space-between; margin-bottom:3px;">
        <span style="color:#94a3b8;">Min Clearance:</span>
        <span id="m-clearance" style="font-weight:700; color:#10b981;">-- m</span>
      </div>
      <div style="display:flex; justify-content:space-between; margin-bottom:3px;">
        <span style="color:#94a3b8;">Collisions / Near-Misses:</span>
        <span id="m-colls" style="font-weight:700; color:#10b981;">0 / 0</span>
      </div>
      <div style="display:flex; justify-content:space-between; margin-bottom:6px;">
        <span style="color:#94a3b8;">Red-Light Compliance:</span>
        <span id="m-signals" style="font-weight:700; color:#10b981;">100% (0 viol)</span>
      </div>
      <div id="m-status" style="font-size:10px; color:#94a3b8; text-align:center; padding-top:4px; border-top:1px dashed #334155;">
        Ready. Click 'RUN TEST' to execute fixed route.
      </div>
    `;

    document.body.appendChild(hud);
    this.hudElement = hud;

    const btn = document.getElementById('btn-run-benchmark');
    if (btn) btn.addEventListener('click', () => this.startBenchmark());
  }

  startBenchmark() {
    console.log('[BENCHMARK] Starting Fixed Test Route...');
    const car = this.controllerManager.entities?.car;
    if (!car) {
      console.error('[BENCHMARK] Car entity not found!');
      return;
    }

    // Teleport car to exact route start position on 5th Ave
    this.controllerManager.setMode('car');
    this.controllerManager.teleportActiveTo(this.startPos.x, this.startPos.z, this.startPos.heading, this.startPos.y);
    car.speed = 0;
    car.vx = 0;
    car.vz = 0;
    car.steerAngle = 0;

    // Reset metric accumulators
    this.routeTime = 0;
    this.frameCount = 0;
    this.prevSpeed = 0;
    this.laneOffsets = [];
    this.laneOffsetsSigned = [];
    this.maxLaneOffset = 0;
    this.laneOffsetSumSq = 0;
    this.steerErrors = [];
    this.headingErrors = [];
    this.maxSteerError = 0;
    this.minObstacleClearance = 999.0;
    this.harshBrakingCount = 0;
    this.harshBrakeCooldown = 0;
    this.collisionCount = 0;
    this.nearMissCount = 0;
    this.nearMissCooldown = 0;
    this.curbDepartureActive = false;
    this.intersectionsEncountered = 0;
    this.redLightViolations = 0;
    this.signalComplianceObserved.clear();
    this.completed = false;
    this.running = true;
    window.__BENCHMARK_STATUS__ = 'RUNNING';
    if (this.autonomousDriver) this.autonomousDriver.enabled = true;

    const statusEl = document.getElementById('m-status');
    if (statusEl) {
      statusEl.textContent = 'RUNNING BENCHMARK...';
      statusEl.style.color = '#38bdf8';
    }
  }

  update(dt, elapsed, autonomousDriver) {
    if (autonomousDriver) this.autonomousDriver = autonomousDriver;

    if (!this.running) {
      if (this.autoStartRequested && !this.completed) {
        this._warmupFrames = (this._warmupFrames || 0) + 1;
        window.__BENCHMARK_STATUS__ = `WARMING_UP (${this._warmupFrames} frames)`;
        if (this._warmupFrames >= 15) {
          this.autoStartRequested = false;
          console.log('[BENCHMARK] Warmup complete, launching benchmark route now!');
          this.startBenchmark();
        }
      }
      return;
    }

    const car = this.controllerManager.entities?.car;
    if (!car) return;

    this.routeTime += dt;
    this.frameCount++;

    // 1. Lane-keeping offset metric
    // Compute distance from 5th Ave target lane (targetLaneX = 3.6m on 5th Ave)
    const currentX = car.position.x;
    const currentZ = car.position.z;
    const laneOffsetSigned = currentX - this.targetLaneX;
    const laneOffsetAbs = Math.abs(laneOffsetSigned);

    this.laneOffsets.push(laneOffsetAbs);
    this.laneOffsetsSigned.push(laneOffsetSigned);
    this.laneOffsetSumSq += laneOffsetAbs * laneOffsetAbs;
    if (laneOffsetAbs > this.maxLaneOffset) this.maxLaneOffset = laneOffsetAbs;

    // Curb departure detection (5th Ave roadway width is ~14.6m, centerline x=0.4 => east curb x=7.7, west curb x=-6.9)
    const offRoadEast = currentX > 7.7;
    const offRoadWest = currentX < -6.9;
    if ((offRoadEast || offRoadWest) && !this.curbDepartureActive) {
      this.curbDepartureActive = true;
      this.collisionCount++;
      console.warn(`[METRICS] Curb departure / Sidewalk collision! x=${currentX.toFixed(2)}, total_collisions=${this.collisionCount}`);
    } else if (!offRoadEast && !offRoadWest) {
      this.curbDepartureActive = false;
    }

    // 2. Steering angle vs road curvature metric
    // 5th Ave runs straight along Manhattan grid south (dx=0, dz=1, road heading = 0 rad, curvature kappa = 0)
    // Desired steering angle for straight road is 0.0 rad.
    const actualSteerDeg = (car.steerAngle || 0) * (180.0 / Math.PI);
    const steerErrorDeg = Math.abs(actualSteerDeg);
    this.steerErrors.push(steerErrorDeg);
    if (steerErrorDeg > this.maxSteerError) this.maxSteerError = steerErrorDeg;

    let headingErr = car.heading; // road heading is 0
    while (headingErr > Math.PI) headingErr -= 2 * Math.PI;
    while (headingErr < -Math.PI) headingErr += 2 * Math.PI;
    this.headingErrors.push(Math.abs(headingErr) * (180.0 / Math.PI));

    // 3. Braking and clearance metrics
    const leadDist = autonomousDriver ? autonomousDriver.leadDist : 999.0;
    if (leadDist < this.minObstacleClearance) {
      this.minObstacleClearance = leadDist;
    }

    const accel = dt > 0.001 ? (car.speed - this.prevSpeed) / dt : 0;
    this.prevSpeed = car.speed;

    if (this.harshBrakeCooldown > 0) this.harshBrakeCooldown -= dt;
    if (accel < -4.0 && car.speed > 2.0 && this.harshBrakeCooldown <= 0) {
      this.harshBrakingCount++;
      this.harshBrakeCooldown = 1.0;
    }

    // 4. Near-miss metric (TTC < 1.5s while clearance < 12.0m)
    const ttc = autonomousDriver ? autonomousDriver.ttc : 999.0;
    if (this.nearMissCooldown > 0) this.nearMissCooldown -= dt;
    if (ttc < 1.5 && leadDist < 12.0 && car.speed > 2.0 && this.nearMissCooldown <= 0) {
      this.nearMissCount++;
      this.nearMissCooldown = 1.5;
    }

    // 5. Traffic light & intersection signal compliance
    // 5th Ave intersections occur every ~80m along z
    // Check signal nodes in road network near car
    const currentAvenuePhase = signalPhase(elapsed + (this.timeOffset || 0), true);
    if (this.trafficSystem?.network?.nodes) {
      for (const node of this.trafficSystem.network.nodes) {
        if (!node.signal) continue;
        const dz = node.z - currentZ;
        // Node is right ahead of car (0m to 6m)
        if (dz > -2.0 && dz < 6.0 && Math.abs(node.x - currentX) < 10.0) {
          const key = `node_${Math.round(node.x)}_${Math.round(node.z)}`;
          if (!this.signalComplianceObserved.has(key)) {
            this.signalComplianceObserved.add(key);
            this.intersectionsEncountered++;
            if (currentAvenuePhase === 'red') {
              if (car.speed > 1.2) {
                this.redLightViolations++;
                console.warn(`[METRICS] Red Light Violation at ${key}! Speed: ${car.speed.toFixed(1)} m/s`);
              }
            }
          }
        }
      }
    }

    // Live HUD updates
    this.updateHudLive();
    window.__BENCHMARK_STATUS__ = `RUNNING (${this.routeTime.toFixed(1)}/${this.routeDuration.toFixed(1)}s, ${this.frameCount}f, colls=${this.collisionCount})`;

    // Check if route complete
    if (this.routeTime >= this.routeDuration) {
      this.finishBenchmark();
    }
  }

  updateHudLive() {
    const elTime = document.getElementById('m-time');
    const elLane = document.getElementById('m-lane');
    const elSteer = document.getElementById('m-steer');
    const elClear = document.getElementById('m-clearance');
    const elColls = document.getElementById('m-colls');
    const elSignals = document.getElementById('m-signals');

    if (elTime) elTime.textContent = `${this.routeTime.toFixed(1)} / ${this.routeDuration.toFixed(1)}s`;

    const count = this.laneOffsets.length;
    if (count > 0 && elLane) {
      const rms = Math.sqrt(this.laneOffsetSumSq / count);
      elLane.textContent = `${rms.toFixed(2)} / ${this.maxLaneOffset.toFixed(2)} m`;
      elLane.style.color = rms > 1.2 ? '#ef4444' : rms > 0.5 ? '#f59e0b' : '#38bdf8';
    }

    if (count > 0 && elSteer) {
      const meanSteer = this.steerErrors.reduce((a, b) => a + b, 0) / count;
      elSteer.textContent = `${meanSteer.toFixed(1)}° / ${this.maxSteerError.toFixed(1)}°`;
      elSteer.style.color = meanSteer > 12.0 ? '#ef4444' : '#38bdf8';
    }

    if (elClear) {
      elClear.textContent = this.minObstacleClearance < 900 ? `${this.minObstacleClearance.toFixed(1)} m` : '-- m';
      elClear.style.color = this.minObstacleClearance < 4.0 ? '#ef4444' : '#10b981';
    }

    if (elColls) {
      elColls.textContent = `${this.collisionCount} / ${this.nearMissCount}`;
      elColls.style.color = this.collisionCount > 0 ? '#ef4444' : this.nearMissCount > 0 ? '#f59e0b' : '#10b981';
    }

    if (elSignals) {
      const total = this.intersectionsEncountered;
      const rate = total > 0 ? ((total - this.redLightViolations) / total * 100).toFixed(0) : '100';
      elSignals.textContent = `${rate}% (${this.redLightViolations} viol)`;
      elSignals.style.color = this.redLightViolations > 0 ? '#ef4444' : '#10b981';
    }
  }

  finishBenchmark() {
    this.running = false;
    this.completed = true;

    const count = Math.max(1, this.laneOffsets.length);
    const meanLaneOffset = this.laneOffsets.reduce((a, b) => a + b, 0) / count;
    const rmsLaneOffset = Math.sqrt(this.laneOffsetSumSq / count);
    const meanSteerError = this.steerErrors.reduce((a, b) => a + b, 0) / count;
    const meanHeadingError = this.headingErrors.reduce((a, b) => a + b, 0) / count;

    const complianceRate = this.intersectionsEncountered > 0
      ? ((this.intersectionsEncountered - this.redLightViolations) / this.intersectionsEncountered) * 100
      : 100.0;

    const report = {
      timestamp: new Date().toISOString(),
      route_duration_s: Math.round(this.routeTime * 100) / 100,
      frames_sampled: count,
      lane_keeping: {
        rms_offset_m: Math.round(rmsLaneOffset * 1000) / 1000,
        mean_offset_m: Math.round(meanLaneOffset * 1000) / 1000,
        max_offset_m: Math.round(this.maxLaneOffset * 1000) / 1000
      },
      steering_vs_curvature: {
        mean_steer_error_deg: Math.round(meanSteerError * 100) / 100,
        max_steer_error_deg: Math.round(this.maxSteerError * 100) / 100,
        mean_heading_error_deg: Math.round(meanHeadingError * 100) / 100
      },
      braking_and_clearance: {
        min_obstacle_clearance_m: this.minObstacleClearance < 900 ? Math.round(this.minObstacleClearance * 10) / 10 : 999.0,
        harsh_braking_events: this.harshBrakingCount
      },
      safety_incidents: {
        collision_count: this.collisionCount,
        near_miss_count: this.nearMissCount
      },
      signal_compliance: {
        intersections_encountered: this.intersectionsEncountered,
        red_light_violations: this.redLightViolations,
        compliance_rate_pct: Math.round(complianceRate * 10) / 10
      }
    };

    this.lastReport = report;
    window.__DRIVING_METRICS_REPORT__ = report;
    window.__BENCHMARK_STATUS__ = 'FINISHED';

    try {
      localStorage.setItem('adas-last-metrics', JSON.stringify(report));
    } catch (e) {}

    // Formatted Console Output for Automation & Log Parsers
    console.log('__DRIVING_METRICS_REPORT__ ' + JSON.stringify(report));
    console.log(
      `\n============================================================\n` +
      `       QNX 8.0 ADAS DRIVING REALISM BENCHMARK REPORT        \n` +
      `============================================================\n` +
      `Route Duration:             ${report.route_duration_s} s (${count} frames)\n` +
      `Lane-Keeping Offset:\n` +
      `  - RMS Error:              ${report.lane_keeping.rms_offset_m} m\n` +
      `  - Mean Error:             ${report.lane_keeping.mean_offset_m} m\n` +
      `  - Max Error:              ${report.lane_keeping.max_offset_m} m\n` +
      `Steering vs Curvature:\n` +
      `  - Mean Steer Error:       ${report.steering_vs_curvature.mean_steer_error_deg}°\n` +
      `  - Max Steer Error:        ${report.steering_vs_curvature.max_steer_error_deg}°\n` +
      `  - Mean Heading Error:     ${report.steering_vs_curvature.mean_heading_error_deg}°\n` +
      `Braking & Clearance:\n` +
      `  - Min Obstacle Gap:       ${report.braking_and_clearance.min_obstacle_clearance_m} m\n` +
      `  - Harsh Braking Events:   ${report.braking_and_clearance.harsh_braking_events}\n` +
      `Safety Incidents:\n` +
      `  - Collision Count:        ${report.safety_incidents.collision_count}\n` +
      `  - Near-Miss Events:       ${report.safety_incidents.near_miss_count}\n` +
      `Signal Compliance:\n` +
      `  - Intersections Passed:   ${report.signal_compliance.intersections_encountered}\n` +
      `  - Red Light Violations:   ${report.signal_compliance.red_light_violations}\n` +
      `  - Compliance Rate:        ${report.signal_compliance.compliance_rate_pct} %\n` +
      `============================================================\n`
    );

    const statusEl = document.getElementById('m-status');
    if (statusEl) {
      statusEl.textContent = `BENCHMARK COMPLETE (${report.route_duration_s}s). Collisions: ${report.safety_incidents.collision_count}`;
      statusEl.style.color = report.safety_incidents.collision_count > 0 ? '#ef4444' : '#10b981';
    }
  }
}
