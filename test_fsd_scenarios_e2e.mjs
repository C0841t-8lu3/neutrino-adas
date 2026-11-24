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
