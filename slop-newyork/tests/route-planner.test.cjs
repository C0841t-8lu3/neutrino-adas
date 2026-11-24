const assert = require('node:assert/strict');
const { readFileSync } = require('node:fs');
const { join } = require('node:path');
const test = require('node:test');
const { loadModule } = require('./helpers/module.cjs');

const plannerSource = readFileSync(join(__dirname, '../js/adas/RoutePlanner.js'), 'utf8');
const governorSource = readFileSync(join(__dirname, '../js/adas/PerformanceGovernor.js'), 'utf8');

function createMockRoadNetwork() {
  // Simple Manhattan-style 2-edge network with a right-angle intersection curve
  // Edge 0: Along 5th Ave (Z: 0 to 100, X: 0, heading 0 rad, dx=0, dz=1)
  // Edge 1: Along 42nd St (X: 0 to 100, Z: 100, heading PI/2 rad, dx=1, dz=0)
  const nodeA = { id: 0, x: 0, z: 0, in: [], out: [] };
  const nodeB = { id: 1, x: 0, z: 100, in: [], out: [] };
  const nodeC = { id: 2, x: 100, z: 100, in: [], out: [] };

  const edge0 = {
    id: 0,
    from: nodeA,
    to: nodeB,
    sx: 0,
    sz: 0,
    dx: 0,
    dz: 1,
    rx: -1,
    rz: 0,
    len: 100,
    speedLimit: 14.0,
    road: { name: '5th Ave' },
    laneIdx: 0,
    lanes: [0],
    moves: []
  };
  nodeA.out.push(edge0);
  nodeB.in.push(edge0);

  const edge1 = {
    id: 1,
    from: nodeB,
    to: nodeC,
    sx: 0,
    sz: 100,
    dx: 1,
    dz: 0,
    rx: 0,
    rz: 1,
    len: 100,
    speedLimit: 11.0,
    road: { name: 'E 42nd St' },
    laneIdx: 0,
    lanes: [0],
    moves: []
  };
  nodeB.out.push(edge1);
  nodeC.in.push(edge1);

  edge0.moves.push({ to: edge1, turn: 'R', out: [0] });

  // Turn connection from edge0 to edge1 at nodeB
  const turnCurve = {
    from: edge0,
    to: edge1,
    fromEdge: edge0,
    toEdge: edge1,
    p0x: 0, p0z: 95,
    p1x: 0, p1z: 100,
    p2x: 5, p2z: 100,
    len: 8.0,
    curve: {
      p0: { x: 0, z: 95 },
      p1: { x: 0, z: 100 },
      p2: { x: 5, z: 100 },
      len: 8.0
    }
  };

  const network = {
    edges: [edge0, edge1],
    nodes: [nodeA, nodeB, nodeC],
    turns: [turnCurve],
    turnsByFrom: new Map([[edge0, [turnCurve]]]),
    turnsByTo: new Map([[edge1, [turnCurve]]])
  };

  return { network, edge0, edge1, turnCurve };
}

test('RoutePlanner snaps coordinates to nearest valid edge', () => {
  const { RoutePlanner } = loadModule(plannerSource, {
    imports: { '../city-config.js': { getLocationLabel: (x, z) => `${x}, ${z}` } }
  });
  const { network, edge0 } = createMockRoadNetwork();
  const planner = new RoutePlanner(network);

  const snap = planner.snapToNearestEdge(2.5, 45.0, 0.0);
  assert.ok(snap);
  assert.equal(snap.edge.id, edge0.id);
  assert.equal(snap.projX, 0.0);
  assert.equal(snap.projZ, 45.0);
  assert.ok(snap.dist < 3.0);
});

test('RoutePlanner plans A* route across connected Manhattan edges', () => {
  const { RoutePlanner } = loadModule(plannerSource, {
    imports: { '../city-config.js': { getLocationLabel: (x, z) => `${x}, ${z}` } }
  });
  const { network } = createMockRoadNetwork();
  const planner = new RoutePlanner(network);

  const start = { x: 0, z: 10 };
  const goal = { x: 80, z: 100 };
  const waypoints = planner.planRoute(start, 0.0, goal);

  assert.ok(waypoints.length > 5, 'Route should contain sampled waypoints');
  assert.ok(planner.hasActiveRoute());
  assert.ok(planner.totalDistance > 50, 'Total distance should reflect multi-edge path');

  // Verify first and last waypoint approximate start and goal
  const first = waypoints[0];
  const last = waypoints[waypoints.length - 1];
  assert.ok(Math.hypot(first.x - start.x, first.z - start.z) < 15.0);
  assert.ok(Math.hypot(last.x - goal.x, last.z - goal.z) < 15.0);
});

test('RoutePlanner calculates speed-adaptive tracking reference and turn signals', () => {
  const { RoutePlanner } = loadModule(plannerSource, {
    imports: { '../city-config.js': { getLocationLabel: (x, z) => `${x}, ${z}` } }
  });
  const { network } = createMockRoadNetwork();
  const planner = new RoutePlanner(network);

  planner.planRoute({ x: 0, z: 10 }, 0.0, { x: 80, z: 100 });

  // Vehicle approaching turn at Z=88
  const egoPos = { x: 0, y: 0, z: 88 };
  const ref = planner.getTrackingReference(egoPos, 0.0, 10.0);

  assert.ok(ref);
  assert.ok(ref.refZ > egoPos.z, 'Lookahead reference should be ahead of vehicle');
  assert.ok(ref.distToGoal > 0);
  assert.ok(ref.turnSignalHint !== undefined);
});

test('RoutePlanner detects off-route deviation and clears route', () => {
  const { RoutePlanner } = loadModule(plannerSource, {
    imports: { '../city-config.js': { getLocationLabel: (x, z) => `${x}, ${z}` } }
  });
  const { network } = createMockRoadNetwork();
  const planner = new RoutePlanner(network);

  planner.planRoute({ x: 0, z: 10 }, 0.0, { x: 80, z: 100 });
  assert.ok(planner.hasActiveRoute());

  // Deviate by 40 meters off route (X = 40, route is at X = 0)
