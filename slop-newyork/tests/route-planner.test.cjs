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
