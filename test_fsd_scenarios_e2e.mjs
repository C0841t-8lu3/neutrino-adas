// test_fsd_scenarios_e2e.mjs
// ============================================================================
// Comprehensive Behavioral & Safety Verification of Tesla FSD-Style ADAS
// Tests run against live QNX 8.0 Neutrino RTOS on QEMU Target via Rust Bridge
// ============================================================================

import assert from 'node:assert/strict';

const WsClient = typeof WebSocket !== 'undefined' ? WebSocket : (await import('file:///usr/share/nodejs/ws/index.js')).default;

async function sendPacket(ws, telemetry) {
  return new Promise((resolve) => {
    ws.onmessage = (event) => {
      resolve(JSON.parse(typeof event.data === 'string' ? event.data : event.data.toString()));
    };
    ws.send(JSON.stringify(telemetry));
  });
}

async function runFsdBehaviorVerification() {
  console.log('====================================================================');
  console.log('  QNX 8.0 ADAS FSD BEHAVIOR & SAFETY VERIFICATION SUITE             ');
  console.log('====================================================================');

  console.log('[1/5] Connecting to QNX Bridge at ws://127.0.0.1:8089/ ...');
  const ws = new WsClient('ws://127.0.0.1:8089/');
  await new Promise((resolve, reject) => {
    ws.onopen = resolve;
    ws.onerror = reject;
  });
  console.log('[QNX Bridge] Connected.');

  // Base telemetry template
  const baseTelemetry = {
    x: 0.0,
    y: 0.0,
    z: 100.0,
    speed: 10.0, // 36 km/h
    heading: 0.0,
    steer_angle: 0.0,
    current_lane: 1,
    target_lane: 1,
    dt: 0.016,
    obs: [],
    has_route: 1,
    ref_x: 0.0,
    ref_z: 120.0,
    ref_heading: 0.0,
    path_curvature: 0.0,
    next_curvature: 0.0,
    dist_to_goal: 500.0,
    turn_signal_hint: 0,
    drive_profile: 1 // STANDARD
  };

  // --------------------------------------------------------------------------
  // TEST 1: Service Braking Smoothness (JerkLimiter in effect)
  // --------------------------------------------------------------------------
  console.log('\n[TEST 1] Verifying Jerk-Limited Smooth Service Braking...');
  {
    // Stabilize car first with empty road
    for (let i = 0; i < 15; i++) {
      await sendPacket(ws, { ...baseTelemetry, z: 100.0 + i * 0.16 });
    }

    // Now present a stop line at 45m ahead (smooth braking required)
    const brakeOutputs = [];
    for (let i = 0; i < 20; i++) {
      const resp = await sendPacket(ws, {
        ...baseTelemetry,
        z: 100.0 + i * 0.16,
        obs: [{ x: 0.0, z: 145.0, speed: 0.0, type: 3 }] // Stop line obstacle
      });
      brakeOutputs.push(resp.brake);
    }

    console.log(`  Brake trajectory over first 8 frames: [${brakeOutputs.slice(0, 8).map(b => b.toFixed(3)).join(', ')}]`);
    
    // Jerk limiter property: brake must NOT jump to 1.0 on frame 0
    assert.ok(brakeOutputs[0] < 0.6, `Frame 0 brake (${brakeOutputs[0]}) should be smooth, not an instant slam`);
    
    // Consecutive differences must be bounded by comfort jerk limit
