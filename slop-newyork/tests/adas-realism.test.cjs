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
      ttc: 30.0 - i > 0 ? (30.0 - i) / 2.0 : 999,
      rttUs: 120,
      bsmAlert: false,
      aebActive: i === 7,
      watchdogOk: true,
      laneOffset: 0.0,
      status: 'Cruising'
    });
  }

  assert.equal(recorder.capacity, 10);
  assert.equal(recorder.count, 10, 'Count capped at buffer capacity');
  assert.equal(recorder.head, 5, 'Head wrapped around (15 % 10 = 5)');

  // Scorecard statistics
  const scorecard = recorder.getScorecard();
  assert.ok(scorecard);
  assert.equal(scorecard.totalFrames, 10);
  assert.equal(scorecard.aebEvents, 1);
  assert.ok(scorecard.maxBrakeCmd >= 0.95);
  assert.ok(scorecard.minTtcS < 15.0);

  // CSV generation
  const csv = recorder.exportCsv();
  assert.ok(csv.startsWith('time_s,seq,x_m,y_m,z_m,speed_mps,steer_rad,throttle,brake'));
  const lines = csv.trim().split('\n');
  assert.equal(lines.length, 11, 'Header + 10 recorded rows');

  // JSON generation
  const json = recorder.exportJson();
  const parsed = JSON.parse(json);
  assert.equal(parsed.samples.length, 10);
  assert.ok(parsed.scorecard);
});

test('ScenarioInjector triggers Euro NCAP scenarios and toggles chaos mode', () => {
  const { ScenarioInjector } = loadModule(scenarioSource);

  const mockEgo = {
    position: { x: 10, y: 0, z: 20 },
    heading: 0,
    speed: 12.0
  };

  const mockCar = { x: 0, z: 0, speed: 0, v: 0, yaw: 0, dl: 0, dlv: 0 };
  const mockPed = { placed: false, x: 0, z: 0, speed: 0, v: 0, ux: 0, uz: 0 };

  const mockTraffic = {
    cars: [mockCar],
    peds: [mockPed]
  };

  const mockDriver = {
    requestLaneChange: (dir) => { mockDriver.lastLaneChange = dir; },
    actuatorDynamics: { setSurface: (s) => { mockDriver.surface = s; } },
    perceptionScanner: { sensorNoiseModel: { setRealistic: (r) => {}, setDropoutRate: (d) => {} } }
  };

  const injector = new ScenarioInjector({
    trafficSystem: mockTraffic,
    autonomousDriver: mockDriver,
    controllerManager: { entities: { car: mockEgo } }
  });

  // Scenario 1: Cut-In
  const res1 = injector.triggerScenario(1);
  assert.equal(res1, true);
  assert.equal(injector.activeScenario.id, 1);
  assert.equal(mockCar.dl, 3.6);
  assert.equal(mockCar.dlv, -2.6);

  // Scenario 2: Jaywalk
  const res2 = injector.triggerScenario(2);
  assert.equal(res2, true);
  assert.equal(injector.activeScenario.id, 2);
  assert.equal(mockPed.placed, true);
  assert.equal(mockPed.speed, 2.8);

  // Scenario 3: CCRb Brake
  const res3 = injector.triggerScenario(3);
  assert.equal(res3, true);
  assert.equal(injector.activeScenario.id, 3);
  assert.equal(mockCar.panicAcc, -6.0);

  // Scenario 4: Blind-Spot Ambush
  const res4 = injector.triggerScenario(4);
  assert.equal(res4, true);
  assert.equal(injector.activeScenario.id, 4);
  assert.equal(mockDriver.lastLaneChange, -1);

  // Scenario 5: Wet road stress test
  const res5 = injector.triggerScenario(5);
  assert.equal(res5, true);
  assert.equal(injector.activeScenario.id, 5);
  assert.equal(mockDriver.surface, 'wet');

  // Cancel restores
  injector.cancelActiveScenario();
  assert.equal(injector.activeScenario, null);

  // Stochastic chaos toggle
  const chaosOn = injector.toggleUnpredictableTraffic(true);
  assert.equal(chaosOn, true);
  assert.equal(injector.unpredictableEnabled, true);
});

test('PerformanceGovernor manages quality profiles (AUTO, LOW, HIGH)', () => {
  const { PerformanceGovernor } = loadModule(governorSource);
  const governor = new PerformanceGovernor(null, { profile: 'AUTO' });

  assert.equal(governor.profile, 'AUTO');

  governor.setProfile('LOW');
  assert.equal(governor.profile, 'LOW');
  assert.equal(governor.currentDpr, 0.75);
