// test_chrysler_bldg_e2e.mjs
// ============================================================================
// E2E Verification to Chrysler Building (Lexington Ave & E 42nd St):
// Verifies Real Driver's License Rules, Phantom Stop Elimination,
// Direction-Matched Traffic Lights & Stop-Line Positioning
// ============================================================================

import assert from 'node:assert/strict';
import { buildRoadNetwork } from './slop-newyork/js/road-network.js';
import { RoutePlanner } from './slop-newyork/js/adas/RoutePlanner.js';
import { PerceptionScanner } from './slop-newyork/js/adas/PerceptionScanner.js';
import { signalPhase } from './slop-newyork/js/city-config.js';

const WsClient = typeof WebSocket !== 'undefined' ? WebSocket : (await import('file:///usr/share/nodejs/ws/index.js')).default;

async function runChryslerBuildingE2ETest() {
  console.log('====================================================================');
  console.log('  CHRYSLER BUILDING E2E REALISM & DRIVER RULES VERIFICATION SUITE   ');
  console.log('====================================================================');

  // 1. Build Real Manhattan Road Network
  console.log('[1/5] Building full Manhattan Road Network graph...');
  const network = buildRoadNetwork();
  console.log(`[Road Network] Loaded ${network.edges.length} edges, ${network.nodes.length} nodes.`);

  const planner = new RoutePlanner(network);
  const perception = new PerceptionScanner({ network, cars: [], buses: [], peds: [] });

  // 2. Plan A* Route to Chrysler Building
  // Origin: 5th Ave (0.1, 120.0)
  // Destination: Chrysler Building at Lexington Ave & E 42nd St (467.2, 0.3)
  const pointA = { x: 0.1, z: 120.0 };
  const headingA = 0.0;
  const chryslerBuilding = { x: 467.2, z: 0.3 };

  console.log(`\n[2/5] Planning A* Route to Chrysler Building:`);
  console.log(`  Origin:        (${pointA.x.toFixed(1)}, ${pointA.z.toFixed(1)})`);
  console.log(`  Destination:   (${chryslerBuilding.x.toFixed(1)}, ${chryslerBuilding.z.toFixed(1)})`);

  const waypoints = planner.planRoute(pointA, headingA, chryslerBuilding);
  assert.ok(waypoints && waypoints.length > 20, 'Route Planner must generate valid waypoints to Chrysler Building');
  console.log(`[Route Planner] SUCCESS: Generated ${waypoints.length} waypoints (${planner.totalDistance.toFixed(1)}m).`);
  console.log(`  Route Summary: ${planner.originName} -> ${planner.destinationName}`);

  // 3. Connect to Rust WebSocket Bridge (QNX 8.0 RTOS Backend)
  console.log('\n[3/5] Connecting to QNX RTOS Bridge at ws://127.0.0.1:8089/ ...');
  const ws = new WsClient('ws://127.0.0.1:8089/');
  await new Promise((resolve, reject) => {
    ws.onopen = resolve;
    ws.onerror = reject;
  });
  console.log('[QNX Bridge] Connected.');

  async function sendTelemetry(telemetry) {
    return new Promise((resolve) => {
      ws.onmessage = (event) => {
        resolve(JSON.parse(typeof event.data === 'string' ? event.data : event.data.toString()));
      };
      ws.send(JSON.stringify(telemetry));
    });
  }

  // 4. VERIFY PRIORITY 1: Real Driver's License Rules
  console.log('\n[4/5] Executing Real Driver\'s License Rules & Blinker Timing Verification...');

  // 4a. Straight Cruising: Blinker MUST be OFF even with tracking lateral error
  {
    const straightWp = waypoints[5];
    const refStraight = planner.getTrackingReference({ x: straightWp.x + 0.45, z: straightWp.z }, straightWp.heading, 10.0);
    // Distance to next turn on 5th Ave is > 45m
    if (refStraight.nextTurnDistance > 45.0) {
      assert.equal(refStraight.turnSignalHint, 0, 'Turn signal MUST be 0 during straight cruising (>45m before turn)');
    }

    const res = await sendTelemetry({
      x: straightWp.x, y: 0.0, z: straightWp.z,
      speed: 10.0, heading: straightWp.heading, steer_angle: 0.0,
      current_lane: 1, target_lane: 1, dt: 0.016, obs: [],
      has_route: 1, ref_x: straightWp.x, ref_z: straightWp.z, ref_heading: straightWp.heading,
      path_curvature: 0.0, dist_to_goal: 500.0, turn_signal_hint: 0,
      in_intersection: 0, dist_to_next_turn: 80.0, pre_turn_lane_hint: 0
    });
    assert.equal(res.left_turn_signal, false, 'Left blinker must remain OFF during straight cruising');
    assert.equal(res.right_turn_signal, false, 'Right blinker must remain OFF during straight cruising');
    console.log('  [PASS] Blinker strictly OFF during straight cruising (eliminated false blinking).');
  }

  // 4b. Turn Blinker Rule (30m - 45m before turn)
  {
    // Find an actual turn (heading change > 0.3 rad) with approach distance >= 60m
    let targetTurnIdx = -1;
    for (let i = 1; i < waypoints.length; i++) {
      if (waypoints[i].isIntersection && !waypoints[i - 1].isIntersection && waypoints[i].distFromStart >= 60.0) {
        let maxDYaw = 0;
        for (let j = i; j < Math.min(waypoints.length, i + 10); j++) {
          let dy = waypoints[j].heading - waypoints[i - 1].heading;
