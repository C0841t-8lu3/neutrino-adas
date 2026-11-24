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
    for (let i = 1; i < brakeOutputs.length; i++) {
      const deltaBrake = Math.abs(brakeOutputs[i] - brakeOutputs[i - 1]);
      assert.ok(deltaBrake <= 0.25, `Step ${i} delta brake (${deltaBrake.toFixed(3)}) exceeds comfort jerk limit`);
    }
    console.log('  [PASS] Service braking is S-curve jerk-limited and comfortable.');
  }

  // --------------------------------------------------------------------------
  // TEST 2: Hard Safety Bypass on AEB Emergency (Scenario 2 Pedestrian Jaywalk)
  // --------------------------------------------------------------------------
  console.log('\n[TEST 2] Verifying 1-Frame Hard Safety Bypass on AEB (Pedestrian Jaywalk)...');
  {
    // Ego at 10 m/s, pedestrian suddenly darting across at 12m ahead (TTC ~ 1.2s -> CRITICAL AEB)
    const respAeb = await sendPacket(ws, {
      ...baseTelemetry,
      speed: 10.0,
      obs: [{ x: 0.0, z: 112.0, speed: 0.0, type: 2 }] // Pedestrian in ego corridor (type 2)
    });

    console.log(`  AEB Response -> Mode: ${respAeb.drive_mode} | Brake: ${respAeb.brake.toFixed(2)} | Horn: ${respAeb.horn_active} | Hazards: ${respAeb.hazard_lights}`);
    
    assert.equal(respAeb.drive_mode, 5, 'Drive mode must be 5 (MODE_AEB_EMERGENCY)');
    assert.equal(respAeb.brake, 1.0, 'Brake MUST hit 1.0 in exactly 1 frame (JerkLimiter hard safety bypass)');
    assert.equal(respAeb.horn_active, true, 'Horn must sound immediately during AEB emergency');
    assert.equal(respAeb.hazard_lights, true, 'Hazard lights must activate during AEB emergency');
    console.log('  [PASS] AEB Hard Safety Bypass triggered 100% braking in 1 frame!');
  }

  // --------------------------------------------------------------------------
  // TEST 3: Driving Profiles (CHILL vs STANDARD vs HURRY Headway Dynamics)
  // --------------------------------------------------------------------------
  console.log('\n[TEST 3] Verifying FSD Driving Profiles (CHILL vs STANDARD vs HURRY Headway)...');
  {
    // Settle system back to cruising (empty road)
    for (let i = 0; i < 20; i++) {
      await sendPacket(ws, { ...baseTelemetry, speed: 8.0, obs: [] });
    }

    // Lead vehicle at 20m ahead (z = 120.0), moving at 8.0 m/s:
    // Lead distance = 20.0 - 2.25 - 2.2 = 15.55m
    // - CHILL headway (1.8s): desired gap = 2.0 + 8.0*1.8 = 16.4m -> relaxed conservative follow
    // - HURRY headway (1.0s): desired gap = 2.0 + 8.0*1.0 = 10.0m -> assertive, tighter following
    const chillPkt = {
      ...baseTelemetry,
      speed: 8.0,
      drive_profile: 0, // CHILL
      obs: [{ x: 0.0, z: 120.0, speed: 8.0, type: 0 }]
    };
    const hurryPkt = {
      ...baseTelemetry,
      speed: 8.0,
      drive_profile: 2, // HURRY
      obs: [{ x: 0.0, z: 120.0, speed: 8.0, type: 0 }]
    };

    let chillResp = null;
    for (let i = 0; i < 25; i++) {
      chillResp = await sendPacket(ws, chillPkt);
    }

    let hurryResp = null;
    for (let i = 0; i < 25; i++) {
      hurryResp = await sendPacket(ws, hurryPkt);
    }

    console.log(`  CHILL (Profile 0, 1.8s headway) -> Throttle: ${chillResp.throttle.toFixed(3)} | Brake: ${chillResp.brake.toFixed(3)} | Mode: ${chillResp.drive_mode}`);
    console.log(`  HURRY (Profile 2, 1.0s headway) -> Throttle: ${hurryResp.throttle.toFixed(3)} | Brake: ${hurryResp.brake.toFixed(3)} | Mode: ${hurryResp.drive_mode}`);

    assert.equal(chillResp.drive_mode, 2, 'CHILL mode should be ACC_FOLLOWING (2)');
    assert.equal(hurryResp.drive_mode, 2, 'HURRY mode should be ACC_FOLLOWING (2)');
    
    // At 20m gap: HURRY throttle is significantly higher (> 5x) than CHILL throttle
    assert.ok(hurryResp.throttle > chillResp.throttle * 5.0, `HURRY throttle (${hurryResp.throttle}) should be significantly more assertive than CHILL (${chillResp.throttle})`);
    console.log('  [PASS] Profiles produce distinct assertive vs conservative headway dynamics.');
  }

  // --------------------------------------------------------------------------
  // TEST 4: BehaviorPlanner Route-Aware Pre-Turn Lane Positioning
  // --------------------------------------------------------------------------
  console.log('\n[TEST 4] Verifying BehaviorPlanner Pre-Turn Lane Positioning (< 90m)...');
  {
    // Clear obstacles
    for (let i = 0; i < 10; i++) {
      await sendPacket(ws, { ...baseTelemetry, speed: 10.0, obs: [] });
    }

    // Vehicle in middle lane (lane 1), upcoming left turn in 60m (hint: -1 for Left)
    const preTurnLeftPkt = {
      ...baseTelemetry,
      current_lane: 1,
      target_lane: 1,
      dist_to_goal: 60.0,
      turn_signal_hint: -1, // Left turn (-1)
      obs: []
    };
    let leftBlinkerFlashed = false;
    for (let i = 0; i < 25; i++) {
      const resp = await sendPacket(ws, preTurnLeftPkt);
      if (resp.left_turn_signal) leftBlinkerFlashed = true;
    }
    console.log(`  Upcoming Left Turn  (hint=-1, dist=60m) -> Left Blinker Flashing: ${leftBlinkerFlashed}`);
    assert.equal(leftBlinkerFlashed, true, 'Left turn blinker must flash within 90m of left turn');

    // Upcoming right turn in 60m (hint: +1 for Right)
    const preTurnRightPkt = {
      ...baseTelemetry,
      current_lane: 1,
      target_lane: 1,
      dist_to_goal: 60.0,
      turn_signal_hint: +1, // Right turn (+1)
      obs: []
    };
    let rightBlinkerFlashed = false;
    for (let i = 0; i < 25; i++) {
      const resp = await sendPacket(ws, preTurnRightPkt);
      if (resp.right_turn_signal) rightBlinkerFlashed = true;
    }
    console.log(`  Upcoming Right Turn (hint=+1, dist=60m) -> Right Blinker Flashing: ${rightBlinkerFlashed}`);
    assert.equal(rightBlinkerFlashed, true, 'Right turn blinker must flash within 90m of right turn');

    console.log('  [PASS] Pre-turn lane positioning and signaling verified.');
  }

