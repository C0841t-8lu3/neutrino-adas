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
