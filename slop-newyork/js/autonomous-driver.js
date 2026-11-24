// js/autonomous-driver.js
// ============================================================================
// QNX 8.0 NEUTRINO RTOS - ADAS LEVEL 3 AUTONOMOUS DRIVING & SAFETY SYSTEM
// Thin OOP Orchestrator coordinating Perception, Telemetry, HUD, Audio,
// Mirrors, Actuator Dynamics, Safety Watchdog, Scenarios & Flight Recorder
// ============================================================================

import { QnxTelemetryClient } from './adas/QnxTelemetryClient.js';
import { PerceptionScanner } from './adas/PerceptionScanner.js';
import { SideMirrorRenderer } from './adas/SideMirrorRenderer.js';
import { AdasHudView } from './adas/AdasHudView.js';
import { HornAudioEngine } from './adas/HornAudioEngine.js';
import { RoutePlanner } from './adas/RoutePlanner.js';
import { ActuatorDynamics } from './adas/ActuatorDynamics.js';
import { TelemetryRecorder } from './adas/TelemetryRecorder.js';
import { ScenarioInjector } from './adas/ScenarioInjector.js';
import { FsdVisualizer } from './adas/FsdVisualizer.js';

export class AutonomousDriver {
  constructor(controllerManager, trafficSystem, scene = null) {
    this.controllerManager = controllerManager;
    this.trafficSystem = trafficSystem;
    this.scene = scene;

    this.enabled = true;
    this.state = 'CRUISING';
    this.statusMessage = 'Connecting to QNX Neutrino RTOS (SDP 8.0)...';
    this.qnxConnected = false;
    this.lastPacketTime = 0;
    this.lastQnxPacketTime = 0;
    this.watchdogFault = false;
    this.seq = 0;
    this.rttMs = 0;
    this.rttUs = 0;
    this.lastObstacleCount = 0;
    this.mirrorTick = 0;
    this.driveModeCode = 1;

    // Navigation and Kinematics
    this.driveProfile = 1; // 0: CHILL, 1: STANDARD, 2: HURRY
    this.lateralNudge = 0.0;
    this.isYielding = false;
    this.targetSpeedMph = 32;
    this.targetSpeed = this.targetSpeedMph * 0.44704;
    this.minSafeGap = 6.0;
    this.timeHeadway = 1.4;
    this.currentLaneIndex = 1; // 0: Left, 1: Center, 2: Right
    this.targetLaneIndex = 1;

    // Radar Perception Readings
    this.leadDist = 999;
    this.leadSpeed = 0;
    this.leadRelSpeed = 0;
    this.ttc = 999;
    this.closestObstacleName = null;
    this.leftBsmAlert = false;
    this.rightBsmAlert = false;
    this.leftBlinker = false;
    this.rightBlinker = false;
    this.hazardLights = false;
    this.hornActive = false;
    this.activeRedStopObstacle = null;

    // Actuator Outputs from QNX RTOS
    this.throttle = 0;
    this.brake = 0;
    this.steerInput = 0;

    // Composed OOP Modules
    this.audioEngine = new HornAudioEngine();
    this.perceptionScanner = new PerceptionScanner(trafficSystem);
    this.mirrorRenderer = new SideMirrorRenderer();
    this.routePlanner = new RoutePlanner(trafficSystem?.network);
    this.actuatorDynamics = new ActuatorDynamics();
    this.telemetryRecorder = new TelemetryRecorder(3600);
    this.scenarioInjector = new ScenarioInjector({
      controllerManager: this.controllerManager,
      trafficSystem: this.trafficSystem,
      autonomousDriver: this,
      routePlanner: this.routePlanner
    });

    this.fsdVisualizer = (this.scene) ? new FsdVisualizer(this.scene, { perfGovernor: this.perfGovernor }) : null;

    this.hudView = new AdasHudView({
      onToggle: () => this.toggle(),
      onProfileChange: (p) => this.setDriveProfile(p),
      onToggleFsdVis: () => {
        if (this.fsdVisualizer) {
          this.fsdVisualizer.toggle();
          this.updateHud();
        }
      },
      onLaneChangeLeft: () => this.requestLaneChange(-1),
      onLaneChangeRight: () => this.requestLaneChange(+1),
      onToggleSensors: () => {
        const snm = this.perceptionScanner?.sensorNoiseModel;
        if (snm) {
          snm.setRealistic(!snm.enabled);
          this.updateHud();
        }
      },
      onToggleSurface: () => {
        if (this.actuatorDynamics) {
          const next = this.actuatorDynamics.surfaceType === 'dry' ? 'wet' : 'dry';
          this.actuatorDynamics.setSurface(next);
          this.updateHud();
        }
      },
      onExportJson: () => this.telemetryRecorder.exportJson(),
      onExportCsv: () => this.telemetryRecorder.exportCsv()
    });

    // Zero-GC pre-allocated telemetry packet object
    this._telemetryPacket = {
      seq: 0,
      x: 0,
      y: 0,
      z: 0,
      speed: 0,
      heading: 0,
      steer_angle: 0,
      current_lane: 1,
      target_lane: 1,
      dt: 0.016,
      obs: []
    };

    // Expose DOM & Canvas handles for wire-compatibility
    this.hudElement = this.hudView.hudElement;
    this.leftMirrorCanvas = this.hudView.leftMirrorCanvas;
    this.rightMirrorCanvas = this.hudView.rightMirrorCanvas;
    this.leftMirrorCtx = this.hudView.leftMirrorCtx;
    this.rightMirrorCtx = this.hudView.rightMirrorCtx;

    this.telemetryClient = new QnxTelemetryClient({
      url: 'ws://127.0.0.1:8089/',
      onResponse: (res) => this.applyQnxResponse(res),
      onConnectionChange: (connected, message) => {
        this.qnxConnected = connected;
        this.statusMessage = message;
        this.updateHud();
      }
    });

    this.bindKeyboard();
    this.telemetryClient.connect();
  }

  get ws() {
    return this.telemetryClient.ws;
  }

  initAudio() {
    this.audioEngine.initAudio();
  }

  playHornSound(duration = 0.35) {
    this.audioEngine.playHornSound(duration);
  }

  initWebSocket() {
    this.telemetryClient.connect();
  }

  applyQnxResponse(res) {
    if (!res) return;

    this.lastPacketTime = (typeof performance !== 'undefined' ? performance.now() : Date.now());
    this.lastQnxPacketTime = this.lastPacketTime;
    this.watchdogFault = false;
    this._watchdogTicks = 0;

    this.throttle = res.throttle || 0;
    this.brake = res.brake || 0;
    this.steerInput = res.steer_cmd || 0;
    this.leftBlinker = !!res.left_turn_signal;
    this.rightBlinker = !!res.right_turn_signal;
    this.hazardLights = !!res.hazard_lights;
    this.leftBsmAlert = !!res.left_mirror_blind_spot;
    this.rightBsmAlert = !!res.right_mirror_blind_spot;
    this.leadDist = (res.lead_distance !== undefined) ? res.lead_distance : 999;
    this.leadRelSpeed = res.lead_rel_speed || 0;
    this.ttc = (res.time_to_collision !== undefined) ? res.time_to_collision : 999;

    if (res.status_message) {
      this.statusMessage = res.status_message;
    }
    if (res.bridge_rtt_us !== undefined) {
      this.rttUs = res.bridge_rtt_us;
    }
    if (res.drive_profile !== undefined) {
      this.driveProfile = res.drive_profile;
    }
    if (res.lateral_nudge !== undefined) {
      this.lateralNudge = res.lateral_nudge;
    }

    // Drive mode string mapping
    const MODES = ['MANUAL', 'CRUISING', 'ACC_FOLLOWING', 'LANE_CHANGING', 'INTERSECTION_TURN', 'AEB_BRAKING', 'STOPPED'];
    this.driveModeCode = res.drive_mode !== undefined ? res.drive_mode : 1;
    this.state = MODES[this.driveModeCode] || 'CRUISING';

    // Acoustic horn trigger from QNX RTOS
    if (res.horn_active) {
      this.hornActive = true;
      this.playHornSound(0.32);
    } else {
      this.hornActive = false;
    }
  }

  requestLaneChange(dir) {
    this.initAudio();
    const newLane = Math.max(0, Math.min(2, this.currentLaneIndex + dir));
    this.targetLaneIndex = newLane;
    console.log(`[AutonomousDriver] Requested Lane: ${newLane} (Current: ${this.currentLaneIndex})`);
  }

  enable() {
    this.initAudio();
    this.enabled = true;
    this.updateHud();
    console.log('[AutonomousDriver] Autopilot: ENGAGED');
  }

  toggle(forceState = null) {
    this.initAudio();
    this.enabled = forceState !== null ? Boolean(forceState) : !this.enabled;
    this.updateHud();
    if (!this.enabled) {
      this.controllerManager.keys.forward = false;
      this.controllerManager.keys.backward = false;
      this.controllerManager.keys.left = false;
      this.controllerManager.keys.right = false;
    }
    console.log(`[AutonomousDriver] Autopilot: ${this.enabled ? 'ENGAGED' : 'DISENGAGED'}`);
  }

  setDriveProfile(profile) {
    this.driveProfile = (profile >= 0 && profile <= 2) ? profile : 1;
    const names = ['CHILL', 'STANDARD', 'HURRY'];
    console.log(`[AutonomousDriver] FSD Driving Profile: ${names[this.driveProfile]}`);
    this.updateHud();
  }

  cycleDriveProfile() {
    this.setDriveProfile((this.driveProfile + 1) % 3);
  }

  bindKeyboard() {
    if (typeof window === 'undefined') return;
    window.addEventListener('keydown', (e) => {
      this.initAudio();
      if (e.code === 'KeyP') {
        this.cycleDriveProfile();
      } else if (e.code === 'KeyV') {
        if (this.fsdVisualizer) {
          this.fsdVisualizer.toggle();
          this.updateHud();
        }
      } else if (e.code === 'KeyQ' || e.code === 'ArrowLeft') {
        this.requestLaneChange(-1);
      } else if (e.code === 'KeyE' || e.code === 'ArrowRight') {
        this.requestLaneChange(+1);
      } else if (e.code === 'KeyH') {
        this.playHornSound(0.4);
      }
    });
  }

  createHudAndMirrors() {
    // Handled in AdasHudView constructor
  }

  renderSideMirror(ctx, canvas, car, isLeft, bsmActive) {
    this.mirrorRenderer.renderSideMirror(ctx, canvas, car, isLeft, bsmActive, this.trafficSystem);
  }

  scanPerception(car, elapsed = 0) {
    const scan = this.perceptionScanner.scanPerception(car, elapsed, this.routePlanner);
    this.leadDist = scan.leadDist;
    this.leadRelSpeed = scan.leadRelSpeed;
    this.closestObstacleName = scan.closestObstacleName;
    this.ttc = scan.ttc;
    this.activeRedStopObstacle = scan.activeRedStopObstacle;
    return scan.trafficList;
  }

  update(dt, elapsed = null) {
    if (this.controllerManager.activeMode !== 'car') {
      this.hudView.setVisibility(false);
      return;
    }

    this.hudView.setVisibility(true);

    const car = this.controllerManager.entities?.car;
    if (!car) return;

    const now = (typeof performance !== 'undefined' ? performance.now() : Date.now());
    this.elapsed = elapsed !== null ? elapsed : (this.elapsed || 0) + dt;

    // Track intersection state from route planner
    if (this.routePlanner && this.routePlanner.hasActiveRoute()) {
      car.in_intersection = !!this.routePlanner._trackingRef.inIntersection;
    } else {
      car.in_intersection = false;
    }

    // 1. Perception Radar Scan
    const trafficList = this.scanPerception(car, this.elapsed);

    // 2. Transmit Real-Time Telemetry to QNX RTOS Bridge (Zero-GC reuse packet)
    if (this.telemetryClient.isConnected()) {
      const obsPayload = this.perceptionScanner.buildObstaclePayload(car, trafficList, 24, dt);
      this.lastObstacleCount = obsPayload.length;

      this.seq = (this.seq || 0) + 1;
      const p = this._telemetryPacket;
      p.seq = this.seq;
      p.x = Math.round(car.position.x * 100) / 100;
      p.y = Math.round(car.position.y * 100) / 100;
      p.z = Math.round(car.position.z * 100) / 100;
      p.speed = Math.round(car.speed * 100) / 100;
      p.heading = Math.round(car.heading * 1000) / 1000;
      p.steer_angle = Math.round((car.steerAngle || 0) * 1000) / 1000;
      p.current_lane = this.currentLaneIndex;
      p.target_lane = this.targetLaneIndex;
      p.dt = Math.round(dt * 1000) / 1000;
      p.drive_profile = this.driveProfile;
      p.obs = obsPayload;

      // Integrate Point-to-Point Navigation Route if active
      if (this.routePlanner && this.routePlanner.hasActiveRoute()) {
        const ref = this.routePlanner.getTrackingReference(car.position, car.heading, car.speed);
        p.has_route = 1;
        p.ref_x = Math.round(ref.refX * 100) / 100;
        p.ref_z = Math.round(ref.refZ * 100) / 100;
        p.ref_heading = Math.round(ref.refHeading * 1000) / 1000;
        p.path_curvature = Math.round(ref.curvature * 1000) / 1000;
        p.next_curvature = Math.round((ref.nextCurvature || 0) * 1000) / 1000;
        p.dist_to_goal = Math.round(ref.distToGoal * 10) / 10;
        p.turn_signal_hint = ref.turnSignalHint;
        p.in_intersection = ref.inIntersection ? 1 : 0;
        p.dist_to_next_turn = Math.round(ref.nextTurnDistance * 10) / 10;
        p.pre_turn_lane_hint = ref.preTurnLaneHint || 0;

        if (ref.turnSignalHint < 0) this.leftBlinker = true;
        else if (ref.turnSignalHint > 0) this.rightBlinker = true;

        if (ref.isArrived && car.speed < 0.5) {
          this.state = 'STOPPED';
          this.statusMessage = 'QNX NAV: Arrived at Destination';
        }
      } else {
        p.has_route = 0;
        p.in_intersection = 0;
        p.dist_to_next_turn = 999.0;
        p.pre_turn_lane_hint = 0;
        delete p.ref_x;
        delete p.ref_z;
        delete p.ref_heading;
        delete p.path_curvature;
        delete p.next_curvature;
        delete p.dist_to_goal;
        delete p.turn_signal_hint;
      }

      this.telemetryClient.send(p);
    }

    // 3. Fallback Autonomous Controls or Watchdog Safe Stop
    if (this.enabled && this.qnxConnected) {
      if (this.lastQnxPacketTime > 0 && (now - this.lastQnxPacketTime > 450)) {
        this._watchdogTicks = (this._watchdogTicks || 0) + 1;
        if (this._watchdogTicks >= 2) {
          // Watchdog timeout exceeded (>450ms with 2 ticks): Engage Safe Fail-Stop
          this.watchdogFault = true;
          this.throttle = 0.0;
          this.brake = 1.0;
          this.hazardLights = true;
          this.state = 'STOPPED';
          this.statusMessage = 'QNX WATCHDOG FAULT — SAFE STOP';
        }
      } else {
        this._watchdogTicks = 0;
      }
    } else if (!this.qnxConnected) {
      if ((this.ttc < 1.7 && this.leadDist < 30.0) || (this.leadDist < 6.5 && car.speed > 1.0)) {
        this.throttle = -1.0;
        this.brake = 1.0;
        this.state = 'AEB_BRAKING';
        this.statusMessage = 'Fallback AEB Braking Active';
      } else if (this.leadDist < 45.0) {
        const desiredGap = this.minSafeGap + Math.max(0, car.speed) * this.timeHeadway;
        const gapError = this.leadDist - desiredGap;
        const targetAccel = 0.5 * gapError + 0.85 * this.leadRelSpeed;
        this.throttle = Math.max(-0.85, Math.min(1.0, targetAccel / 2.2));
        this.brake = this.throttle < 0 ? -this.throttle : 0;
        this.state = 'ACC_FOLLOWING';
        this.statusMessage = `Fallback ACC Following (${this.leadDist.toFixed(1)}m)`;
      } else {
        const speedError = this.targetSpeed - car.speed;
        this.throttle = Math.max(-0.35, Math.min(1.0, speedError * 0.45));
        this.brake = 0;
        this.state = 'CRUISING';
        this.statusMessage = 'Autopilot Cruising (Searching for QNX Link)';
      }
    }

    // 4. Actuate 3D Vehicle Controls via Physical Actuator Dynamics
    const act = this.actuatorDynamics.step(
      this.throttle,
      this.brake,
      this.steerInput,
      dt
    );
    const effThrottle = act.throttle;
    const effBrake = act.brake;
    const effSteer = act.steer;

    if (this.enabled) {
      if (effBrake > 0.05 && car.speed > 0.2) {
        this.controllerManager.keys.forward = false;
        this.controllerManager.keys.backward = Math.min(1.0, effBrake);
      } else if (car.speed <= 0.2 && (effBrake > 0.05 || this.state === 'STOPPED')) {
        this.controllerManager.keys.forward = false;
        this.controllerManager.keys.backward = false;
        car.speed = 0;
        car.vx = 0;
        car.vz = 0;
      } else {
        this.controllerManager.keys.forward = effThrottle > 0 ? effThrottle : false;
        this.controllerManager.keys.backward = effThrottle < 0 ? -effThrottle : false;
      }
      this.controllerManager.keys.left = effSteer > 0 ? effSteer : false;
      this.controllerManager.keys.right = effSteer < 0 ? -effSteer : false;
    }

    // 5. Update 3D Turn Signals and Brake Lights on Car Fascias
    if (car.carState) {
      car.carState.value.x = (effBrake > 0.1 || (car.speed * effThrottle < 0)) ? 1 : 0;
      car.carState.value.y = (this.leftBlinker || this.hazardLights) ? 1 : 0;
      car.carState.value.z = (this.rightBlinker || this.hazardLights) ? 1 : 0;
    }

    // 6. Render Real-Time Side-View Mirrors
    this.renderSideMirror(this.leftMirrorCtx, this.leftMirrorCanvas, car, true, this.leftBsmAlert);
    this.renderSideMirror(this.rightMirrorCtx, this.rightMirrorCanvas, car, false, this.rightBsmAlert);

    // 7. Update HUD & Mirror Warning Indicators
    this.updateHud();
  }

  updateHud() {
    const car = this.controllerManager.entities?.car;
    const speedMph = car ? (Math.abs(car.speed) * 2.23694).toFixed(1) : '0.0';

    this.hudView.updateHud({
      qnxConnected: this.qnxConnected,
      enabled: this.enabled,
      state: this.state,
      speedMph: speedMph,
      targetSpeedMph: this.targetSpeedMph,
      leadDist: this.leadDist,
      closestObstacleName: this.closestObstacleName,
      leftBsmAlert: this.leftBsmAlert,
      rightBsmAlert: this.rightBsmAlert,
      leftBlinker: this.leftBlinker,
      rightBlinker: this.rightBlinker,
      hazardLights: this.hazardLights,
      statusMessage: this.statusMessage
    });
  }
}
