#include "adas/QnxAdasController.hpp"

namespace adas {

QnxAdasController::QnxAdasController(const AdasConfig& config)
    : config_(config),
      current_profile_(config.profile),
      forward_scanner_(std::make_unique<ForwardRadar>()),
      blind_spot_scanner_(std::make_unique<BlindSpotMonitor>()),
      longitudinal_ctrl_(config),
      lateral_ctrl_(config),
      signal_mgr_(),
      behavior_planner_(config),
      jerk_limiter_(config) {}

void QnxAdasController::reset() {
    longitudinal_ctrl_.reset();
    lateral_ctrl_.reset();
    signal_mgr_.reset();
    behavior_planner_.reset();
    jerk_limiter_.reset();
}

void QnxAdasController::setConfig(const AdasConfig& config) {
    config_ = config;
    current_profile_ = config.profile;
    longitudinal_ctrl_.setConfig(config);
    lateral_ctrl_.setConfig(config);
}

AdasOutput QnxAdasController::step(const VehicleState& ego, const std::vector<Obstacle>& obstacles, float dt) {
    AdasOutput out;

    // 1. Update signal clock and timers
    signal_mgr_.updateTimers(dt);

    // 2. Perception layer: Forward Radar & Side Blind Spot Monitor
    ForwardScanResult fwd;
    if (forward_scanner_) {
        fwd = forward_scanner_->scan(ego, obstacles);
    }

    BlindSpotResult bsm;
    if (blind_spot_scanner_) {
        bsm = blind_spot_scanner_->scan(ego, obstacles);
    }

    out.left_mirror_blind_spot = bsm.left_bsm;
    out.right_mirror_blind_spot = bsm.right_bsm;
    out.lead_distance = fwd.lead_distance;
    out.lead_rel_speed = fwd.lead_rel_speed;
    out.time_to_collision = fwd.time_to_collision;

    // 3. Lateral control & Lane-change decision state machine
    LateralOutput lat = lateral_ctrl_.step(ego, fwd, bsm, dt);

    // 4. Longitudinal control (AEB, ACC, Cruising, Stop-Line Deceleration)
    LongitudinalOutput lon = longitudinal_ctrl_.step(ego, fwd, dt);

    // 5. Arbitrate hazard and horn actuators
    bool hazard = lat.hazard || lon.hazard;
    bool horn = lat.horn || lon.horn;
    if (lat.horn_timer > 0.0f) {
        signal_mgr_.triggerHorn(lat.horn_timer);
    }
    if (lon.horn_timer > 0.0f) {
        signal_mgr_.triggerHorn(lon.horn_timer);
    }

    // 6. Actuator and light signal synthesis
    SignalOutput sig = signal_mgr_.computeSignals(
        hazard,
        horn,
        lat.lane_change_active,
        lat.lateral_error,
        lon.brake
    );

    // 7. Assemble final output
    out.throttle = lon.throttle;
    out.brake = lon.brake;
    out.steer_cmd = lat.steer_cmd;
    out.left_turn_signal = sig.left_turn_signal;
    out.right_turn_signal = sig.right_turn_signal;
    out.hazard_lights = sig.hazard_lights;
    out.horn_active = sig.horn_active;
    out.brake_lights = sig.brake_lights;
    out.drive_mode = lon.mode;

    // Preserve BSM alert status unless emergency/stop overrides it
    if (!lat.bsm_status_message.empty() &&
        lon.mode != DriveMode::AEB_EMERGENCY &&
        lon.mode != DriveMode::STOPPED) {
        out.status_message = lat.bsm_status_message;
    } else {
        out.status_message = lon.status_message;
    }

    return out;
}

} // namespace adas
