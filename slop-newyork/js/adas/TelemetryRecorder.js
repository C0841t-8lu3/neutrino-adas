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
        laneOffset: 0,
        status: ''
      };
    }

    // Safety statistics counters
    this.totalAebEvents = 0;
    this.totalBsmAborts = 0;
    this.totalCollisions = 0;
    this.minTtcObserved = 999;
    this.maxBrakeObserved = 0;
    this._lastAebState = false;
  }

  record(frame) {
    if (!this.isRecording) return;

    const slot = this.buffer[this.head];
    slot.time = frame.time || 0;
    slot.seq = frame.seq || 0;
    slot.x = frame.x || 0;
    slot.y = frame.y || 0;
    slot.z = frame.z || 0;
    slot.speed = frame.speed || 0;
    slot.steerAngle = frame.steerAngle || 0;
    slot.throttle = frame.throttle || 0;
    slot.brake = frame.brake || 0;
    slot.steerCmd = frame.steerCmd || 0;
    slot.driveMode = frame.driveMode || 0;
    slot.leadDist = frame.leadDist !== undefined ? frame.leadDist : 999;
    slot.ttc = frame.ttc !== undefined ? frame.ttc : 999;
    slot.rttUs = frame.rttUs || 0;
    slot.bsmAlert = Boolean(frame.bsmAlert);
    slot.aebActive = Boolean(frame.aebActive);
    slot.watchdogOk = frame.watchdogOk !== undefined ? frame.watchdogOk : true;
    slot.laneOffset = frame.laneOffset || 0;
    slot.status = frame.status || '';

    // Track incident statistics
    if (slot.ttc > 0 && slot.ttc < this.minTtcObserved) {
      this.minTtcObserved = slot.ttc;
    }
    if (slot.brake > this.maxBrakeObserved) {
      this.maxBrakeObserved = slot.brake;
    }
    if (slot.aebActive && !this._lastAebState) {
      this.totalAebEvents++;
    }
    this._lastAebState = slot.aebActive;

    if (frame.collisionOccurred) {
      this.totalCollisions++;
    }
    if (frame.bsmAbortOccurred) {
      this.totalBsmAborts++;
    }

    this.head = (this.head + 1) % this.capacity;
    if (this.count < this.capacity) {
      this.count++;
    }
  }

  getOrderedFrames() {
    const frames = [];
    if (this.count === 0) return frames;

    const startIdx = this.count < this.capacity ? 0 : this.head;
    for (let i = 0; i < this.count; i++) {
      const idx = (startIdx + i) % this.capacity;
      frames.push(this.buffer[idx]);
    }
    return frames;
  }

  getScorecard() {
    const frames = this.getOrderedFrames();
    const n = frames.length;
    if (n === 0) {
      return {
        totalFrames: 0,
        durationSeconds: 0,
        rmsLaneOffsetM: 0,
        minTtcS: 999,
        maxBrakeCmd: 0,
        aebEvents: 0,
        bsmAborts: 0,
        collisions: 0,
        qnxRttP50Us: 0,
        qnxRttP95Us: 0
      };
    }

    // RMS Lane Offset
    let sumSqOffset = 0;
    const rtts = [];
    for (let i = 0; i < n; i++) {
      const f = frames[i];
      sumSqOffset += f.laneOffset * f.laneOffset;
      if (f.rttUs > 0) rtts.push(f.rttUs);
    }
    const rmsLaneOffset = Math.sqrt(sumSqOffset / n);

    rtts.sort((a, b) => a - b);
