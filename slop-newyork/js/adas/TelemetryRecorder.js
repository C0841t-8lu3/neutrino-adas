// js/adas/TelemetryRecorder.js
// ============================================================================
// ADAS Black-Box Flight Recorder & Empirical Safety Scorecard
// Zero-allocation 60-second circular ring buffer (3,600 frames @ 60 Hz)
// Exports empirical CSV and JSON flight logs and calculates safety scorecards.
// ============================================================================

export class TelemetryRecorder {
  constructor(capacity = 3600) {
    this.capacity = capacity;
    this.buffer = new Array(capacity);
    this.head = 0;
    this.count = 0;
    this.isRecording = true;

    // Pre-allocate zero-GC ring buffer slots
    for (let i = 0; i < capacity; i++) {
      this.buffer[i] = {
        time: 0,
        seq: 0,
        x: 0,
        y: 0,
        z: 0,
        speed: 0,
        steerAngle: 0,
        throttle: 0,
        brake: 0,
        steerCmd: 0,
        driveMode: 0,
        leadDist: 999,
        ttc: 999,
        rttUs: 0,
        bsmAlert: false,
        aebActive: false,
        watchdogOk: true,
