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
