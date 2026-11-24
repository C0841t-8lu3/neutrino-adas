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
