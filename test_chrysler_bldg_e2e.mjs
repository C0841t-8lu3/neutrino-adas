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
          while (dy > Math.PI) dy -= 2 * Math.PI;
          while (dy < -Math.PI) dy += 2 * Math.PI;
          if (Math.abs(dy) > Math.abs(maxDYaw)) maxDYaw = dy;
        }
        if (Math.abs(maxDYaw) > 0.3) {
          targetTurnIdx = i;
          break;
        }
      }
    }
    assert.ok(targetTurnIdx > 0, 'Route must contain an actual turn with approach distance >= 60m');
    const turnWp = waypoints[targetTurnIdx];

    // Position car 34m before turn (within 30m-45m zone)
    let sampleIdx34m = targetTurnIdx;
    while (sampleIdx34m > 0 && (turnWp.distFromStart - waypoints[sampleIdx34m].distFromStart) < 34.0) {
      sampleIdx34m--;
    }
    const ego34m = waypoints[sampleIdx34m];
    const ref34m = planner.getTrackingReference(ego34m, ego34m.heading, 8.0);
    assert.ok(ref34m.nextTurnDistance <= 45.0 && ref34m.nextTurnDistance >= 20.0, `Next turn distance ${ref34m.nextTurnDistance}m within 20-45m`);
    assert.notEqual(ref34m.turnSignalHint, 0, 'Turn signal MUST activate 30-45m before turn');

    // Position car 65m before turn: signal should NOT be active yet (>45m)
    let sampleIdx65m = sampleIdx34m;
    while (sampleIdx65m > 0 && (turnWp.distFromStart - waypoints[sampleIdx65m].distFromStart) < 65.0) {
      sampleIdx65m--;
    }
    const ego65m = waypoints[sampleIdx65m];
    const ref65m = planner.getTrackingReference(ego65m, ego65m.heading, 8.0);
    assert.equal(ref65m.turnSignalHint, 0, 'Turn signal MUST NOT activate 65m too early');
    console.log(`  [PASS] Blinker timing verified: OFF at ${ref65m.nextTurnDistance}m ahead, active at ${ref34m.nextTurnDistance}m ahead.`);
  }

  // 4c. Auto-Cancel Blinker Post-Turn
  {
    // Waypoint 92 is immediately after the turn at 82-88, aligned on the new street
    const postTurnWp = waypoints[92];
    const refPost = planner.getTrackingReference(postTurnWp, postTurnWp.heading, 8.0);
    assert.equal(refPost.inIntersection, false);
    assert.equal(refPost.turnSignalHint, 0, 'Turn signal MUST auto-cancel once exited intersection curve and aligned');
    console.log('  [PASS] Blinker auto-cancels immediately upon completing intersection curve.');
  }

  // 4d. Intersection Box-Clearing Rule ("Never Stop Inside an Intersection")
  {
    const turnWp = waypoints.find(wp => wp.isIntersection);
    // Simulate being inside intersection with red light signal ahead
    const res = await sendTelemetry({
      x: turnWp.x, y: 0.0, z: turnWp.z,
      speed: 6.5, heading: turnWp.heading, steer_angle: 0.05,
      current_lane: 1, target_lane: 1, dt: 0.016,
      obs: [{ x: turnWp.x + 3.0, z: turnWp.z + 3.0, speed: 0.0, type: 3 }], // red stop line obstacle
      has_route: 1, ref_x: turnWp.x, ref_z: turnWp.z, ref_heading: turnWp.heading,
      path_curvature: turnWp.curvature, dist_to_goal: 300.0, turn_signal_hint: 0,
      in_intersection: 1, dist_to_next_turn: 0.0, pre_turn_lane_hint: 0
    });
    assert.ok(res.brake < 0.2, 'Must NOT emergency/stop inside intersection box for red light');
    assert.notEqual(res.drive_mode, 6, 'Must NOT enter STOPPED mode inside intersection box');
    console.log('  [PASS] Intersection Box-Clearing rule obeyed: car smoothly clears intersection without stopping.');
  }

  // 5. VERIFY PRIORITY 2: Eliminate Phantom Sudden Stops
  console.log('\n[5/5] Executing Phantom Brake Elimination & Perception Filter Verifications...');
