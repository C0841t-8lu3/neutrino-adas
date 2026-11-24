// js/adas/RoutePlanner.js
// ============================================================================
// OOP Road-Graph A* Route Planner for Autonomous Manhattan Driving
// Respects directed one-way streets, lane transitions, and Bezier turn curves
// ============================================================================

import { getLocationLabel } from '../city-config.js';

export class RoutePlanner {
  constructor(roadNetwork = null) {
    this.roadNetwork = roadNetwork;
    this.activeRoute = null; // ordered array of waypoints
    this.currentIndex = 0;
    this.startPos = null;
    this.goalPos = null;
    this.destinationName = '';
    this.originName = '';
    this.totalDistance = 0;
    this.offRouteThreshold = 18.0; // meters
    this.lastRerouteTime = 0;

    // Reusable tracking reference result object (Zero-GC)
    this._trackingRef = {
      hasRoute: false,
      refX: 0,
      refZ: 0,
      refHeading: 0,
      curvature: 0,
      nextCurvature: 0,
      distToGoal: 0,
      totalDistance: 0,
      speedLimit: 14.0,
      turnHint: 'STRAIGHT', // 'STRAIGHT' | 'LEFT' | 'RIGHT' | 'ARRIVING' | 'ARRIVED'
      turnSignalHint: 0,    // -1: left, 0: none, +1: right
      preTurnLaneHint: 0,   // -1: left, 0: none, +1: right
      inIntersection: false, // true when vehicle is traversing an intersection curve
      lateralError: 0,
      headingError: 0,
      nextTurnDistance: 999,
      nextStreetName: '',
      isArrived: false
    };
  }

  setRoadNetwork(roadNetwork) {
    this.roadNetwork = roadNetwork;
  }

  hasActiveRoute() {
    return !!(this.activeRoute && this.activeRoute.length > 0 && !this._trackingRef.isArrived);
  }

  clearRoute() {
    this.activeRoute = null;
    this.currentIndex = 0;
    this.startPos = null;
    this.goalPos = null;
    this.destinationName = '';
    this.originName = '';
    this.totalDistance = 0;
    this._trackingRef.hasRoute = false;
    this._trackingRef.isArrived = false;
    this._trackingRef.turnSignalHint = 0;
    this._trackingRef.preTurnLaneHint = 0;
    this._trackingRef.inIntersection = false;
    this._trackingRef.curvature = 0;
    this._trackingRef.nextCurvature = 0;
    this._trackingRef.distToGoal = 0;
  }

  /**
   * Snaps a world coordinate (worldX, worldZ) to the nearest directed drivable road edge.
   * If heading is provided, penalizes reverse-direction edges to respect traffic flow.
   */
  snapToNearestEdge(worldX, worldZ, heading = null) {
    if (!this.roadNetwork || !this.roadNetwork.edges || this.roadNetwork.edges.length === 0) {
      return null;
    }

    let bestEdge = null;
    let bestDistSq = Infinity;
    let bestS = 0;
    let bestProjX = worldX;
    let bestProjZ = worldZ;

    const edges = this.roadNetwork.edges;
    const count = edges.length;

    for (let i = 0; i < count; i++) {
      const e = edges[i];
      const dx = worldX - e.sx;
      const dz = worldZ - e.sz;

      // Projection along edge direction
      let s = dx * e.dx + dz * e.dz;
      s = Math.max(0, Math.min(e.len, s));

      // Lane center offset (use center lane or default 0)
      const laneOffset = (e.lanes && e.lanes.length > 0)
        ? e.lanes[Math.floor(e.lanes.length / 2)]
        : 0;

      const rx = e.rx !== undefined ? e.rx : -e.dz;
      const rz = e.rz !== undefined ? e.rz : e.dx;
      const projX = e.sx + e.dx * s + rx * (typeof laneOffset === 'number' ? laneOffset : 0);
      const projZ = e.sz + e.dz * s + rz * (typeof laneOffset === 'number' ? laneOffset : 0);

      const distSq = (worldX - projX) * (worldX - projX) + (worldZ - projZ) * (worldZ - projZ);
      let penalty = 0;

      if (heading !== null) {
        // Vehicle forward vector
        const vx = Math.sin(heading);
        const vz = Math.cos(heading);
        // Edge direction
        const dot = vx * e.dx + vz * e.dz;
        if (dot < -0.2) {
          // Opposite direction: heavy penalty so we don't snap wrong-way into one-way streets
          penalty = 4000.0;
        }
      }

      const totalScore = distSq + penalty;
      if (totalScore < bestDistSq) {
        bestDistSq = totalScore;
        bestEdge = e;
        bestS = s;
        bestProjX = projX;
        bestProjZ = projZ;
      }
    }

    if (!bestEdge) return null;

    const edgeHeading = Math.atan2(bestEdge.dx, bestEdge.dz);
    return {
      edge: bestEdge,
      s: bestS,
      projX: bestProjX,
      projZ: bestProjZ,
      heading: edgeHeading,
      dist: Math.sqrt(Math.max(0, bestDistSq))
    };
  }

  /**
   * Plans an optimal road-graph route from startPos to goalPos using A* search.
   * Returns an array of densely sampled, smoothed waypoints.
   */
  planRoute(startPos, startHeading, goalPos) {
    if (!this.roadNetwork) {
      console.warn('[RoutePlanner] No road network available.');
      return null;
    }

    const startSnap = this.snapToNearestEdge(startPos.x, startPos.z, startHeading);
    const goalSnap = this.snapToNearestEdge(goalPos.x, goalPos.z, null);

    if (!startSnap || !goalSnap) {
      console.warn('[RoutePlanner] Could not snap origin or destination to road network.');
      return null;
    }

    const startEdge = startSnap.edge;
    const goalEdge = goalSnap.edge;

    // A* Graph Search over road edges
    // Distance heuristic: Euclidean distance from edge endpoint to goal
    const heuristic = (edge) => {
      const endX = edge.sx + edge.dx * edge.len;
      const endZ = edge.sz + edge.dz * edge.len;
      return Math.hypot(goalSnap.projX - endX, goalSnap.projZ - endZ);
    };

    const gScore = new Map();
    const fScore = new Map();
    const cameFrom = new Map(); // edge.id -> { edge, move, curve }

    // Min-heap / priority list
    const openSet = [startEdge];
    gScore.set(startEdge.id, 0);
    fScore.set(startEdge.id, heuristic(startEdge));

    let foundGoal = false;
    let iterations = 0;
    const maxIterations = 6000;

    while (openSet.length > 0 && iterations++ < maxIterations) {
      // Find edge with lowest fScore
      let bestIdx = 0;
      let lowestF = fScore.get(openSet[0].id) ?? Infinity;
      for (let i = 1; i < openSet.length; i++) {
        const score = fScore.get(openSet[i].id) ?? Infinity;
        if (score < lowestF) {
          lowestF = score;
          bestIdx = i;
        }
      }

      const current = openSet.splice(bestIdx, 1)[0];

      if (current.id === goalEdge.id) {
        foundGoal = true;
        break;
      }

      const currentG = gScore.get(current.id) ?? Infinity;

      // Expand outgoing valid moves at the intersection
      const moves = (current.moves && current.moves.length > 0)
        ? current.moves
        : (current.to && current.to.out ? current.to.out.map((toEdge) => ({ to: toEdge, turn: 'S' })) : []);
      for (const m of moves) {
        const neighbor = m.to;
        if (!neighbor) continue;

        // Approximate turn curve distance
        const turnLen = (m.turn === 'S') ? 6.0 : (m.turn === 'L' || m.turn === 'R') ? 14.0 : 18.0;
        const tentativeG = currentG + current.len + turnLen;

        const neighborG = gScore.get(neighbor.id) ?? Infinity;
        if (tentativeG < neighborG) {
          cameFrom.set(neighbor.id, { fromEdge: current, move: m });
          gScore.set(neighbor.id, tentativeG);
          fScore.set(neighbor.id, tentativeG + heuristic(neighbor));

          if (!openSet.includes(neighbor)) {
            openSet.push(neighbor);
          }
        }
      }
    }

    if (!foundGoal && startEdge.id !== goalEdge.id) {
      console.warn('[RoutePlanner] No road graph path found to destination.');
      return null;
    }

    // Reconstruct edge sequence from goal to start
    const edgePath = [];
    let curr = goalEdge;
    while (curr && curr.id !== startEdge.id) {
      const prev = cameFrom.get(curr.id);
      if (!prev) break;
      edgePath.unshift({ edge: curr, move: prev.move });
      curr = prev.fromEdge;
    }
    edgePath.unshift({ edge: startEdge, move: null });

    // Build dense sampled waypoints
    const waypoints = this._buildWaypointsFromEdgePath(edgePath, startSnap, goalSnap);

    if (waypoints.length > 0) {
      this.activeRoute = waypoints;
      this.currentIndex = 0;
      this.startPos = { x: startPos.x, z: startPos.z };
      this.goalPos = { x: goalPos.x, z: goalPos.z };
      this.originName = getLocationLabel(startPos.x, startPos.z);
      this.destinationName = getLocationLabel(goalPos.x, goalPos.z);
      this.totalDistance = waypoints[waypoints.length - 1].distFromStart;
      this._trackingRef.isArrived = false;
      this._trackingRef.hasRoute = true;
      this._trackingRef.totalDistance = this.totalDistance;

      console.log(`[RoutePlanner] Planned A* Route: ${edgePath.length} street segments, ${waypoints.length} waypoints, ${(this.totalDistance).toFixed(0)}m (${this.originName} → ${this.destinationName})`);
    }

    return waypoints;
  }

  _buildWaypointsFromEdgePath(edgePath, startSnap, goalSnap) {
    const waypoints = [];
    let cumDist = 0;
    const connector = this.roadNetwork.connector;
    const curvePoint = this.roadNetwork.curvePoint;

    for (let k = 0; k < edgePath.length; k++) {
      const { edge, move } = edgePath[k];
      const nextMove = (k < edgePath.length - 1) ? edgePath[k + 1].move : null;
      const isStartEdge = (k === 0);
      const isEndEdge = (k === edgePath.length - 1);

      const sStart = isStartEdge ? startSnap.s : 0.0;
      const sEnd = isEndEdge ? goalSnap.s : edge.len;

      // Real Driver's License Rule: Pre-turn lane discipline:
      // Left turns from leftmost lane (index 0), Right turns from rightmost lane (index length-1)
      let laneIdx = (edge.lanes && edge.lanes.length > 0)
        ? Math.floor(edge.lanes.length / 2)
        : 0;

      if (nextMove) {
        if (nextMove.turn === 'L') {
          laneIdx = 0;
        } else if (nextMove.turn === 'R') {
          laneIdx = (edge.lanes && edge.lanes.length > 0) ? edge.lanes.length - 1 : 0;
        }
      } else if (move) {
        if (move.turn === 'L') {
          laneIdx = 0;
        } else if (move.turn === 'R') {
          laneIdx = (edge.lanes && edge.lanes.length > 0) ? edge.lanes.length - 1 : 0;
        }
      }

      const laneOffset = (edge.lanes && edge.lanes.length > 0)
        ? edge.lanes[laneIdx]
        : 0;

      // 1. Sample straight/curved street segment
      const sampleStep = 2.5; // sample every 2.5 meters
      const streetLen = Math.max(0, sEnd - sStart);
      const steps = Math.max(1, Math.ceil(streetLen / sampleStep));

      for (let i = 0; i <= steps; i++) {
        const s = sStart + (i / steps) * streetLen;
        const rx = edge.rx !== undefined ? edge.rx : -edge.dz;
        const rz = edge.rz !== undefined ? edge.rz : edge.dx;
        const x = edge.sx + edge.dx * s + rx * (typeof laneOffset === 'number' ? laneOffset : 0);
        const z = edge.sz + edge.dz * s + rz * (typeof laneOffset === 'number' ? laneOffset : 0);
        const heading = Math.atan2(edge.dx, edge.dz);

        if (waypoints.length > 0) {
          const prev = waypoints[waypoints.length - 1];
          const d = Math.hypot(x - prev.x, z - prev.z);
          cumDist += d;
        }

        waypoints.push({
          x,
          z,
          heading,
          curvature: 0.0,
          speedLimit: edge.speedLimit || 14.0,
          distFromStart: cumDist,
          isIntersection: false,
          edgeId: edge.id,
          streetName: edge.name || ''
        });
      }

      // 2. Sample intersection turn curve to next edge
      if (k < edgePath.length - 1 && connector && curvePoint) {
        const nextEdge = edgePath[k + 1].edge;
        let fromLaneIdx = laneIdx;
        let toLaneIdx = (nextEdge.lanes && nextEdge.lanes.length > 0)
          ? Math.floor(nextEdge.lanes.length / 2)
          : 0;

        if (nextMove) {
          if (nextMove.turn === 'L') {
            fromLaneIdx = 0;
            toLaneIdx = 0;
          } else if (nextMove.turn === 'R') {
            fromLaneIdx = (edge.lanes && edge.lanes.length > 0) ? edge.lanes.length - 1 : 0;
            toLaneIdx = (nextEdge.lanes && nextEdge.lanes.length > 0) ? nextEdge.lanes.length - 1 : 0;
          } else if (nextMove.out && nextMove.out[fromLaneIdx] >= 0) {
            toLaneIdx = nextMove.out[fromLaneIdx];
          }
        }

        let curve = null;
        try {
          curve = connector(edge, fromLaneIdx, nextEdge, toLaneIdx);
        } catch (_) {}

        if (curve && curve.len > 0.5) {
          const curveSteps = Math.max(4, Math.ceil(curve.len / 1.5));
          const pt = { x: 0, z: 0, dx: 0, dz: 0 };

          for (let ci = 1; ci <= curveSteps; ci++) {
            const cs = (ci / curveSteps) * curve.len;
            curvePoint(curve, cs, pt);
            const cheading = Math.atan2(pt.dx, pt.dz);

            if (waypoints.length > 0) {
              const prev = waypoints[waypoints.length - 1];
              const d = Math.hypot(pt.x - prev.x, pt.z - prev.z);
              cumDist += d;
            }

            waypoints.push({
              x: pt.x,
              z: pt.z,
              heading: cheading,
              curvature: curve.curv || 0.07,
              speedLimit: 6.5, // 23 km/h safe cornering speed
              distFromStart: cumDist,
              isIntersection: true,
              edgeId: -1,
              streetName: 'Intersection Turn'
            });
          }
        }
      }
    }

    return waypoints;
  }

  /**
   * Evaluates the active route relative to the vehicle's current state.
   * Computes lookahead target, upcoming turn curvature, remaining distance,
   * turn-by-turn guidance, and checks for automatic off-route rerouting.
   */
  getTrackingReference(egoPos, egoHeading, egoSpeed = 0) {
    if (!this.hasActiveRoute()) {
      this._trackingRef.hasRoute = false;
      return this._trackingRef;
    }

    const route = this.activeRoute;
    const len = route.length;
    const ex = egoPos.x;
    const ez = egoPos.z;

    // 1. Find closest waypoint ahead of current search index
    let closestIdx = this.currentIndex;
    let closestDistSq = Infinity;
    const searchRange = Math.min(len, this.currentIndex + 40);

    for (let i = Math.max(0, this.currentIndex - 5); i < searchRange; i++) {
      const wp = route[i];
      const dx = ex - wp.x;
      const dz = ez - wp.z;
      const dSq = dx * dx + dz * dz;
      if (dSq < closestDistSq) {
        closestDistSq = dSq;
        closestIdx = i;
      }
    }

    // If local window didn't find a close waypoint (< 20m), search full route (handles teleports/skips)
    if (closestDistSq > 400) {
      for (let i = 0; i < len; i++) {
        const wp = route[i];
        const dx = ex - wp.x;
        const dz = ez - wp.z;
        const dSq = dx * dx + dz * dz;
        if (dSq < closestDistSq) {
          closestDistSq = dSq;
          closestIdx = i;
        }
      }
    }

    this.currentIndex = closestIdx;
    const closestWp = route[closestIdx];
    const closestDist = Math.sqrt(closestDistSq);

    // 2. Off-route detection and automatic rerouting
    const now = (typeof performance !== 'undefined') ? performance.now() : Date.now();
    if (closestDist > this.offRouteThreshold && (now - this.lastRerouteTime > 3000) && this.goalPos) {
      console.warn(`[RoutePlanner] Off-route detected (${closestDist.toFixed(1)}m > ${this.offRouteThreshold}m). Rerouting to destination...`);
      this.lastRerouteTime = now;
      this.planRoute(egoPos, egoHeading, this.goalPos);
      return this.getTrackingReference(egoPos, egoHeading, egoSpeed);
    }

    // 3. Dynamic speed-adaptive lookahead distance (L = clamp(3.0 + 0.45 * speed, 3.5, 18.0) meters)
    const lookaheadDist = Math.max(3.5, Math.min(18.0, 3.0 + 0.45 * Math.abs(egoSpeed)));
    const targetDist = closestWp.distFromStart + lookaheadDist;

    let lookaheadIdx = closestIdx;
    while (lookaheadIdx < len - 1 && route[lookaheadIdx].distFromStart < targetDist) {
      lookaheadIdx++;
    }
    const targetWp = route[lookaheadIdx];

    // 4. Calculate maximum curvature over upcoming horizon and next lookahead
    let maxCurvature = targetWp.curvature || 0.0;
