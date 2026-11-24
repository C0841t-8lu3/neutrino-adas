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

  _acquireTrafficItem(index) {
    if (index >= this._trafficPool.length) {
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
    return this._trafficPool[index];
  }

  scanPerception(car, elapsed = 0, routePlanner = null) {
    const px = car.position.x;
    const pz = car.position.z;
    const sin = Math.sin(car.heading);
    const cos = Math.cos(car.heading);

    this._trafficList.length = 0;
    let poolIdx = 0;

    if (this.trafficSystem?.cars) {
      const cars = this.trafficSystem.cars;
      const count = cars.length;
      for (let i = 0; i < count; i++) {
        const c = cars[i];
        if (Number.isFinite(c.x) && Number.isFinite(c.z)) {
          const item = this._acquireTrafficItem(poolIdx++);
          item.x = c.x;
          item.z = c.z;
          item.v = c.v || 0;
          item.half = c.half || 2.0;
          item.kind = 'Car';
          item.isBus = false;
          item.isPedestrian = false;
          item.vx = 0;
          item.vz = 0;
          this._trafficList.push(item);
        }
      }
    }

    if (this.trafficSystem?.buses) {
      const buses = this.trafficSystem.buses;
      const count = buses.length;
      for (let i = 0; i < count; i++) {
        const b = buses[i];
        if (Number.isFinite(b.x) && Number.isFinite(b.z)) {
          const item = this._acquireTrafficItem(poolIdx++);
          item.x = b.x;
          item.z = b.z;
          item.v = b.v || 0;
          item.half = b.half || 3.5;
          item.kind = 'Bus';
          item.isBus = true;
          item.isPedestrian = false;
          item.vx = 0;
          item.vz = 0;
          this._trafficList.push(item);
        }
      }
    }

    // Include dynamic sidewalk & crosswalk pedestrians within perception range
    if (this.trafficSystem?.peds) {
      const peds = this.trafficSystem.peds;
      const count = peds.length;
      for (let i = 0; i < count; i++) {
        const p = peds[i];
        if (p.placed && Number.isFinite(p.x) && Number.isFinite(p.z)) {
          const dx = p.x - px;
          const dz = p.z - pz;
          const distSq = dx * dx + dz * dz;
          if (distSq < 2500) { // within 50m
            const item = this._acquireTrafficItem(poolIdx++);
            item.x = p.x;
            item.z = p.z;
            item.v = p.v || 0;
            item.half = 0.5;
            item.kind = 'Pedestrian';
            item.isBus = false;
            item.isPedestrian = true;
            item.vx = (p.ux || 0) * (p.v || 0) * (p.dir || 1);
            item.vz = (p.uz || 0) * (p.v || 0) * (p.dir || 1);
            this._trafficList.push(item);
          }
        }
      }
    }

    // Scan for upcoming Red / Amber Traffic Signals matching the vehicle's current travel axis
    const timeOffset = (typeof window !== 'undefined' && window.__BENCHMARK_TIME_OFFSET__) || 0;
    const effectiveTime = elapsed + timeOffset;

    // Determine current travel axis: Avenue (N/S, |sin(heading)| < 0.6) vs Cross-Street (E/W)
    const isAvenue = Math.abs(sin) < 0.6;
    let redStopObstacle = null;

    // Intersection box-clearing rule: Never stop for red lights once inside an intersection
    const inIntersection = !!(car.in_intersection || (routePlanner && routePlanner._trackingRef && routePlanner._trackingRef.inIntersection));

    if (!inIntersection && this.trafficSystem?.network) {
      let bestDist = 999.0;
      let stopX = 0;
      let stopZ = 0;
      let stopHeading = car.heading;
      let targetFound = false;

      // 1. Precise Edge-based Stop-Line Detection (s = edge.len - 0.35 before crosswalk)
      if (this.trafficSystem.network.edges && this.trafficSystem.network.edges.length > 0) {
        const edges = this.trafficSystem.network.edges;
        for (let i = 0; i < edges.length; i++) {
          const e = edges[i];
          if (!e.to || !e.to.signal) continue;

          // Check if vehicle is along or approaching this edge in the travel direction
          const dx = px - e.sx;
          const dz = pz - e.sz;
          const s = dx * e.dx + dz * e.dz;
          const rx = e.rx !== undefined ? e.rx : -e.dz;
          const rz = e.rz !== undefined ? e.rz : e.dx;
          const lat = dx * rx + dz * rz;

          // Must be close laterally to the edge corridor (< 8.0m) and pointing roughly in same direction
          const headingDot = sin * e.dx + cos * e.dz;
          if (headingDot < 0.6 || Math.abs(lat) > 8.0) continue;

          const stopS = e.len - 0.35; // Painted stop line before the crosswalk
          const dStopLine = stopS - s;

          if (dStopLine > 0.5 && dStopLine < 65.0) {
            // Signal phase for this edge's axis
            const edgeIsAvenue = (e.axis === 'ave');
            const phase = signalPhase(effectiveTime, edgeIsAvenue);

            if (phase === 'red') {
              if (dStopLine < bestDist) {
                bestDist = dStopLine;
                stopX = e.sx + e.dx * stopS;
                stopZ = e.sz + e.dz * stopS;
                stopHeading = Math.atan2(e.dx, e.dz);
                targetFound = true;
              }
            } else if (phase === 'amber') {
              // Yellow-Light Dilemma Zone
              const dStop = (car.speed * car.speed) / (2 * 2.8) + 2.5;
              if (dStopLine >= dStop) {
                if (dStopLine < bestDist) {
                  bestDist = dStopLine;
                  stopX = e.sx + e.dx * stopS;
                  stopZ = e.sz + e.dz * stopS;
                  stopHeading = Math.atan2(e.dx, e.dz);
                  targetFound = true;
                }
              }
              // If dStopLine < dStop: dilemma zone, commit and proceed through!
            }
          }
        }
      }

      // 2. Fallback Node Scan (for mock road networks without detailed edges)
      if (!targetFound && this.trafficSystem.network.nodes) {
        const nodes = this.trafficSystem.network.nodes;
        const currentPhase = signalPhase(effectiveTime, isAvenue);

        if (currentPhase === 'red' || currentPhase === 'amber') {
          let minFwd = 999.0;
          let targetNode = null;
          for (let i = 0; i < nodes.length; i++) {
            const node = nodes[i];
            if (!node.signal) continue;
            const dx = node.x - px;
            const dz = node.z - pz;
            const fwd = dx * sin + dz * cos;
            const lat = dx * cos - dz * sin;

            if (fwd > 1.5 && fwd < 65.0 && Math.abs(lat) < 7.0) {
              if (fwd < minFwd) {
                minFwd = fwd;
                targetNode = node;
              }
            }
          }

          if (targetNode) {
            const stopLineFwd = minFwd - (targetNode.hz || 3.0) - 2.5 - 0.35;
            let shouldStop = (currentPhase === 'red');
            if (currentPhase === 'amber') {
              const dStop = (car.speed * car.speed) / (2 * 2.8) + 2.5;
              shouldStop = (stopLineFwd >= dStop);
            }
            if (shouldStop && stopLineFwd > 0.5) {
              bestDist = stopLineFwd;
              stopX = px + sin * stopLineFwd;
              stopZ = pz + cos * stopLineFwd;
              stopHeading = car.heading;
              targetFound = true;
            }
          }
        }
      }

      if (targetFound) {
        this._redStopObstacle.x = stopX;
        this._redStopObstacle.z = stopZ;
        this._redStopObstacle.heading = stopHeading;
        this._redStopObstacle.v = 0.0;
        this._redStopObstacle.half = 1.0;
        this._redStopObstacle.kind = 'Red Light Stop Line';
        this._redStopObstacle.isBus = false;
        redStopObstacle = this._redStopObstacle;
        this._trafficList.push(this._redStopObstacle);

        if (!this._lastLoggedSignal || Math.abs(bestDist - this._lastLoggedSignal) > 15.0) {
          this._lastLoggedSignal = bestDist;
          console.log(`[PerceptionScanner] Direction-matched red/amber signal ahead at distance ${bestDist.toFixed(1)}m`);
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
