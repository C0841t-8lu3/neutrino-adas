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
