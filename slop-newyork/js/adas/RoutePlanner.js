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

