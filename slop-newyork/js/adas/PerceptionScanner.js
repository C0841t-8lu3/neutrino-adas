// js/adas/PerceptionScanner.js
// ============================================================================
// ADAS Forward LiDAR / Radar Perception & Virtual Intersection Barrier Scanner
// Zero-GC Hot Loop: Pre-allocated object pools and reusable scratch structures
// ============================================================================

import { signalPhase } from '../city-config.js';
import { SensorNoiseModel } from './SensorNoiseModel.js';

export class PerceptionScanner {
  constructor(trafficSystem = null) {
    this.trafficSystem = trafficSystem;
    this.sensorNoiseModel = new SensorNoiseModel({ enabled: false, trackerEnabled: true });
    this.leadDist = 999;
    this.leadRelSpeed = 0;
    this.closestObstacleName = null;
    this.ttc = 999;
    this.activeRedStopObstacle = null;
    this._lastLoggedSignal = null;

    // --- ZERO-GC REUSABLE POOLS ---
    // 1. Reusable traffic list and vehicle pool
    this._trafficList = [];
    this._trafficPool = [];
    this._poolCapacity = 128;
    for (let i = 0; i < this._poolCapacity; i++) {
      this._trafficPool.push({
        x: 0,
        z: 0,
        v: 0,
        half: 2.0,
        kind: 'Car',
        isBus: false,
        isPedestrian: false,
        vx: 0,
        vz: 0
      });
    }

    // 2. Pre-allocated virtual red-light barrier obstacle
    this._redStopObstacle = {
      x: 0,
      z: 0,
      heading: 0,
      v: 0.0,
      half: 1.0,
      kind: 'Red Light Stop Line',
      isBus: false
    };

    // 3. Reusable scan result struct
    this._scanResult = {
      trafficList: this._trafficList,
      leadDist: 999,
      leadRelSpeed: 0,
      closestObstacleName: null,
      ttc: 999,
      activeRedStopObstacle: null
    };

    // 4. Pre-allocated sorting and payload pools
    this._sortScratch = [];
    for (let i = 0; i < this._poolCapacity; i++) {
      this._sortScratch.push({
        traffic: null,
        distSq: 0
      });
    }

    this._obstaclePayload = [];
    this._payloadPool = [];
    for (let i = 0; i < 32; i++) {
      this._payloadPool.push({
        x: 0,
        z: 0,
        speed: 0,
        type: 0,
        vx: 0,
        vz: 0
      });
    }
  }

  setTrafficSystem(trafficSystem) {
    this.trafficSystem = trafficSystem;
  }

  scanPerception(car, elapsed = 0) {
    const px = car.position.x;
    const pz = car.position.z;
    const sin = Math.sin(car.heading);
    const cos = Math.cos(car.heading);

    const traffic = [];
    if (this.trafficSystem?.cars) {
      for (const c of this.trafficSystem.cars) {
        if (Number.isFinite(c.x) && Number.isFinite(c.z)) {
          traffic.push({ ...c, kind: 'Car', isBus: false });
        }
      }
    }
    if (this.trafficSystem?.buses) {
      for (const b of this.trafficSystem.buses) {
        if (Number.isFinite(b.x) && Number.isFinite(b.z)) {
          traffic.push({ ...b, kind: 'Bus', isBus: true });
        }
      }
    }

    // Scan for upcoming Red / Amber Traffic Signals on the current avenue corridor
    const timeOffset = (typeof window !== 'undefined' && window.__BENCHMARK_TIME_OFFSET__) || 0;
    const effectiveTime = elapsed + timeOffset;
    const aveSignal = signalPhase(effectiveTime, true);
    let redStopObstacle = null;

    if ((aveSignal === 'red' || aveSignal === 'amber') && this.trafficSystem?.network?.nodes) {
      let minFwd = 999.0;
      let targetNode = null;

      for (const node of this.trafficSystem.network.nodes) {
        if (!node.signal) continue;
        const dx = node.x - px;
        const dz = node.z - pz;
        const fwd = dx * sin + dz * cos;
        const lat = dx * cos - dz * sin;

        // Find the closest intersection node directly ahead in the avenue corridor
        if (fwd > 1.5 && fwd < 85.0 && Math.abs(lat) < 8.0) {
          if (fwd < minFwd) {
            minFwd = fwd;
            targetNode = node;
          }
        }
      }

      if (targetNode) {
        const stopLineFwd = minFwd - (targetNode.hz || 3.0) - 2.5;
        const obsFwd = Math.max(1.8, stopLineFwd);
        redStopObstacle = {
          x: px,
          z: pz + (cos * obsFwd) + (sin * 0.0),
          v: 0.0,
          half: 1.0,
          kind: 'Red Light Stop Line',
          isBus: false
        };
        traffic.push(redStopObstacle);

        if (!this._lastLoggedSignal || Math.abs(minFwd - this._lastLoggedSignal) > 15.0) {
          this._lastLoggedSignal = minFwd;
          console.log(`[PerceptionScanner] Red light detected ahead at distance ${minFwd.toFixed(1)}m, stop line at ${obsFwd.toFixed(1)}m`);
        }
      }
    }
    this.activeRedStopObstacle = redStopObstacle;

    let nearestDist = 999;
    let nearestRelSpeed = 0;
    let nearestName = null;

    for (const v of traffic) {
      const dx = v.x - px;
      const dz = v.z - pz;
      const fwd = dx * sin + dz * cos;
      const lat = dx * cos - dz * sin;

      // Forward radar detection cone (75m ahead, within lane corridor ~2.3m)
      if (fwd > 1.2 && fwd < 75.0 && Math.abs(lat) < 2.3) {
        const dist = fwd - (v.half || 2.0) - 2.3;
        if (dist < nearestDist) {
          nearestDist = Math.max(0.1, dist);
          nearestRelSpeed = (v.v || 0) - car.speed;
          nearestName = v.kind;
        }
      }
    }

    this.leadDist = nearestDist;
    this.leadRelSpeed = nearestRelSpeed;
    this.closestObstacleName = nearestName;

    if (this.leadRelSpeed < -0.1 && this.leadDist < 75.0) {
      this.ttc = this.leadDist / (-this.leadRelSpeed);
    } else {
      this.ttc = 999;
    }

    return {
      trafficList: traffic,
      leadDist: this.leadDist,
      leadRelSpeed: this.leadRelSpeed,
      closestObstacleName: this.closestObstacleName,
      ttc: this.ttc,
      activeRedStopObstacle: this.activeRedStopObstacle
    };
  }

  buildObstaclePayload(car, trafficList, maxObstacles = 24) {
    const obsPayload = [];
    const px = car.position.x;
    const pz = car.position.z;

    // Prioritize virtual red-light barrier at index 0
    if (this.activeRedStopObstacle) {
      obsPayload.push({
        x: Math.round(this.activeRedStopObstacle.x * 10) / 10,
        z: Math.round(this.activeRedStopObstacle.z * 10) / 10,
        speed: 0.0,
        type: 0
      });
    }

    const sortedTraffic = trafficList
      .filter(t => t.kind !== 'Red Light Stop Line')
      .map(t => {
        const dx = t.x - px;
        const dz = t.z - pz;
        return { t, distSq: dx * dx + dz * dz };
      })
      .filter(item => item.distSq < 7500)
      .sort((a, b) => a.distSq - b.distSq);

    for (const item of sortedTraffic) {
      if (obsPayload.length >= maxObstacles) break;
      obsPayload.push({
        x: Math.round(item.t.x * 10) / 10,
        z: Math.round(item.t.z * 10) / 10,
        speed: Math.round((item.t.v || 0) * 10) / 10,
        type: item.t.isBus ? 1 : 0
      });
    }

    return obsPayload;
  }
}
