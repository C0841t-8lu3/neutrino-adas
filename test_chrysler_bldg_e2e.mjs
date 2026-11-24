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
