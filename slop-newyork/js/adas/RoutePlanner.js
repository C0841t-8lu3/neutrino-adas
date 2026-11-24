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
