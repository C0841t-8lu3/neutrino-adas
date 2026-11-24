// test_p2p_navigation_e2e.mjs
// End-to-End Point-to-Point Navigation Integration Test:
// Three.js Road-Graph A* Planner -> Rust Bridge -> QNX 8.0 L3 Safety Kernel

import { buildRoadNetwork } from './slop-newyork/js/road-network.js';
import { RoutePlanner } from './slop-newyork/js/adas/RoutePlanner.js';

const WsClient = typeof WebSocket !== 'undefined' ? WebSocket : (await import('file:///usr/share/nodejs/ws/index.js')).default;

async function runEndToEndTest() {
  console.log('==================================================================');
  console.log('  QNX 8.0 ADAS POINT-TO-POINT NAVIGATION END-TO-END VERIFICATION  ');
  console.log('==================================================================');

  // 1. Build Real Manhattan Road Network
  console.log('[1/4] Building full Manhattan Road Network graph...');
  const network = buildRoadNetwork();
  console.log(`[Road Network] Loaded ${network.edges.length} edges, ${network.nodes.length} nodes.`);

  const planner = new RoutePlanner(network);

  // 2. Select Point A (Origin on 5th Ave) and Point B (Destination on 42nd St / Grand Central)
  // Point A: 5th Ave & 34th St area (x ~ 0, z ~ 100)
  // Point B: Lexington / 42nd St area (x ~ 300, z ~ 600)
  const pointA = { x: 0.1, z: 120.0 };
  const headingA = 0.0; // Heading North along 5th Ave
  const pointB = { x: 260.0, z: 650.0 };

  console.log(`\n[2/4] Planning A* Route:`);
  console.log(`  Point A (Origin):      (${pointA.x.toFixed(1)}, ${pointA.z.toFixed(1)})`);
  console.log(`  Point B (Destination): (${pointB.x.toFixed(1)}, ${pointB.z.toFixed(1)})`);

  const waypoints = planner.planRoute(pointA, headingA, pointB);
  if (!waypoints || waypoints.length === 0) {
    throw new Error('A* Route Planner failed to find a valid road path between Point A and Point B!');
  }

  console.log(`[Route Planner] SUCCESS: Generated ${waypoints.length} waypoints.`);
  console.log(`  Total Path Distance:   ${planner.totalDistance.toFixed(1)} meters`);
  console.log(`  Origin:                "${planner.originName}"`);
  console.log(`  Destination:           "${planner.destinationName}"`);

  // Verify waypoints continuity
  let turnDetected = false;
  let maxCurv = 0;
  for (let i = 1; i < waypoints.length; i++) {
    const wp = waypoints[i];
    const prev = waypoints[i - 1];
    const stepDist = Math.hypot(wp.x - prev.x, wp.z - prev.z);
    if (stepDist > 15.0) {
      throw new Error(`Discontinuous waypoint jump: ${stepDist.toFixed(1)}m at index ${i}`);
    }
    if (wp.curvature > 0.03) {
      turnDetected = true;
      if (wp.curvature > maxCurv) maxCurv = wp.curvature;
    }
  }
  console.log(`  Turn Curvature Check:  Max Curvature = ${maxCurv.toFixed(4)} rad/m (Turns detected: ${turnDetected})`);

  // 3. Connect to Rust WebSocket Bridge (relaying to QNX 8.0 RTOS on VM)
  console.log('\n[3/4] Connecting to QNX Bridge at ws://127.0.0.1:8089/ ...');
  const ws = new WsClient('ws://127.0.0.1:8089/');
  await new Promise((resolve, reject) => {
    ws.onopen = resolve;
    ws.onerror = reject;
  });
  console.log('[QNX Bridge] Connected via WebSocket.');

  // 4. Simulate Closed-Loop Autonomous Driving along Route Waypoints
  console.log('\n[4/4] Simulating Autonomous Driving along Planned A* Route...');

  let ego = {
    x: waypoints[0].x,
    y: 0.0,
    z: waypoints[0].z,
    speed: 10.0, // m/s
    heading: waypoints[0].heading,
    steer_angle: 0.0
  };

  let arrived = false;
  let rttSamples = [];
  let steps = 0;
  let currStepIdx = 0;
  const maxSteps = 200;

  for (let step = 0; step < maxSteps; step++) {
    const ref = planner.getTrackingReference(ego, ego.heading, ego.speed);

    // Build navigation telemetry packet
    const telemetry = {
      x: Math.round(ego.x * 100) / 100,
      y: 0.0,
      z: Math.round(ego.z * 100) / 100,
      speed: Math.round(ego.speed * 100) / 100,
      heading: Math.round(ego.heading * 1000) / 1000,
      steer_angle: Math.round(ego.steer_angle * 1000) / 1000,
      current_lane: 1,
      target_lane: 1,
      dt: 0.016,
      obs: [],
      has_route: 1,
      ref_x: Math.round(ref.refX * 100) / 100,
      ref_z: Math.round(ref.refZ * 100) / 100,
      ref_heading: Math.round(ref.refHeading * 1000) / 1000,
      path_curvature: Math.round(ref.curvature * 1000) / 1000,
      dist_to_goal: Math.round(ref.distToGoal * 10) / 10,
      turn_signal_hint: ref.turnSignalHint
    };

    const response = await new Promise((resolve) => {
      ws.onmessage = (event) => {
        resolve(JSON.parse(typeof event.data === 'string' ? event.data : event.data.toString()));
      };
      ws.send(JSON.stringify(telemetry));
