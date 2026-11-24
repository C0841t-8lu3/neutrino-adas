const assert = require('node:assert/strict');
const { readFileSync } = require('node:fs');
const { join } = require('node:path');
const test = require('node:test');
const { loadModule } = require('./helpers/module.cjs');

const sensorSource = readFileSync(join(__dirname, '../js/adas/SensorNoiseModel.js'), 'utf8');
const actuatorSource = readFileSync(join(__dirname, '../js/adas/ActuatorDynamics.js'), 'utf8');
const recorderSource = readFileSync(join(__dirname, '../js/adas/TelemetryRecorder.js'), 'utf8');
const scenarioSource = readFileSync(join(__dirname, '../js/adas/ScenarioInjector.js'), 'utf8');
const governorSource = readFileSync(join(__dirname, '../js/adas/PerformanceGovernor.js'), 'utf8');

test('SensorNoiseModel produces deterministic seeded Gaussian noise and handles dropout', () => {
  const { SensorNoiseModel } = loadModule(sensorSource);

  const modelA = new SensorNoiseModel({ enabled: true, seed: 12345, dropoutRate: 0.0 });
  const modelB = new SensorNoiseModel({ enabled: true, seed: 12345, dropoutRate: 0.0 });

  const rawObstacles = [
    { x: 0.0, z: 25.0, v: 10.0, half: 2.0, kind: 'Car', isBus: false }
  ];

  const filteredA = modelA.process(rawObstacles, { position: { x: 0, z: 0 }, heading: 0, speed: 10 }, 0.016);
  const filteredB = modelB.process(rawObstacles, { position: { x: 0, z: 0 }, heading: 0, speed: 10 }, 0.016);

  assert.equal(filteredA.length, 1);
  assert.equal(filteredB.length, 1);
  assert.equal(filteredA[0].x, filteredB[0].x);
  assert.equal(filteredA[0].z, filteredB[0].z);

  // Test 100% Dropout
  const modelDropout = new SensorNoiseModel({ enabled: true, dropoutRate: 1.0 });
  const dropped = modelDropout.process(rawObstacles, { position: { x: 0, z: 0 }, heading: 0, speed: 10 }, 0.016);
  assert.equal(dropped.length, 0);

  // Test Ground Truth bypass when enabled=false
  const modelDisabled = new SensorNoiseModel({ enabled: false });
  const groundTruth = modelDisabled.process(rawObstacles, { position: { x: 0, z: 0 }, heading: 0, speed: 10 }, 0.016);
