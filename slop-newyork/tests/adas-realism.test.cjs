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
  assert.equal(groundTruth.length, 1);
  assert.equal(groundTruth[0].x, 0.0);
  assert.equal(groundTruth[0].z, 25.0);
});

test('SensorNoiseModel performs line-of-sight radar occlusion', () => {
  const { SensorNoiseModel } = loadModule(sensorSource);
  const model = new SensorNoiseModel({ enabled: true, dropoutRate: 0.0 });

  // Two obstacles in the exact same corridor directly ahead:
  // Car 1 at 12m, Car 2 at 35m behind Car 1
  const obstacles = [
    { x: 0.0, z: 12.0, v: 8.0, half: 2.2, kind: 'Car', isBus: false },
    { x: 0.1, z: 35.0, v: 8.0, half: 2.2, kind: 'Car', isBus: false }
  ];

  const visible = model.process(obstacles, { position: { x: 0, z: 0 }, heading: 0, speed: 8 }, 0.016);
  // The lead vehicle at 12m occludes the vehicle at 35m
  assert.equal(visible.length, 1);
  assert.ok(visible[0].z < 20.0);
});

test('ActuatorDynamics models first-order lag, rate limits, and surface friction', () => {
  const { ActuatorDynamics } = loadModule(actuatorSource);
  const dynamics = new ActuatorDynamics({
    steerLagTau: 0.090,
    throttleLagTau: 0.100,
    brakeLagTau: 0.085,
    steerRateLimit: 0.85,
    surfaceType: 'dry'
  });

  // Step 1: Throttle step input from 0.0 to 1.0
  const out1 = dynamics.step(1.0, 0.0, 0.0, 0.016);
  assert.ok(out1.throttle > 0.0 && out1.throttle < 0.25, 'Throttle smoothly builds up with 100ms lag');
  assert.equal(out1.brake, 0.0);
  assert.equal(out1.steer, 0.0);

  // Step 2: Rate limited steering
  const out2 = dynamics.step(0.0, 0.0, 1.0, 0.016);
  const maxSteerStep = 0.85 * 0.016; // 0.0136 rad max
  assert.ok(out2.steer <= maxSteerStep + 1e-5, 'Steering rate clamp holds');

  // Step 3: Wet surface friction braking authority
  dynamics.setSurface('wet');
  assert.equal(dynamics.frictionMu, 0.55);
  dynamics.actualBrake = 1.0;
  const outWet = dynamics.step(0.0, 1.0, 0.0, 0.016);
  const expectedWetBrake = 1.0 * (0.55 / 0.95);
  assert.ok(Math.abs(outWet.brake - expectedWetBrake) < 0.05, 'Braking on wet surface is scaled by friction coefficient');

  // Step 4: Bypass mode
  dynamics.setEnabled(false);
  const outBypass = dynamics.step(0.77, 0.33, -0.45, 0.016);
  assert.equal(outBypass.throttle, 0.77);
  assert.equal(outBypass.brake, 0.33);
  assert.equal(outBypass.steer, -0.45);
});

test('TelemetryRecorder circular ring buffer records, wraps and computes scorecard', () => {
  const { TelemetryRecorder } = loadModule(recorderSource);
  const recorder = new TelemetryRecorder(10); // Small 10-slot buffer for test

  for (let i = 0; i < 15; i++) {
    recorder.record({
      time: 1000 + i * 16,
      seq: i,
      x: i * 2,
      y: 0,
      z: 50 + i * 5,
      speed: 12.0,
      steerAngle: 0.01,
      throttle: 0.6,
      brake: i === 7 ? 0.95 : 0.0,
      steerCmd: 0.0,
      driveMode: 1,
      leadDist: 30.0 - i,
