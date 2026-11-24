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
    behavior_planner_.setConfig(config);
    jerk_limiter_.setConfig(config);
}

AdasOutput QnxAdasController::step(const VehicleState& ego, const std::vector<Obstacle>& obstacles, float dt) {
    AdasOutput out;

    // 0. Dynamic Drive Profile Tuning (CHILL / STANDARD / HURRY)
    if (ego.drive_profile != current_profile_) {
        current_profile_ = ego.drive_profile;
        config_.applyProfile(current_profile_);
        longitudinal_ctrl_.setConfig(config_);
        lateral_ctrl_.setConfig(config_);
        behavior_planner_.setConfig(config_);
        jerk_limiter_.setConfig(config_);
    }

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

    // 3. Human-Like Behavior Planner
    BehaviorDecision plan = behavior_planner_.plan(ego, fwd, bsm, obstacles, dt);

    // 4. Lateral control with in-lane lateral nudge and pre-turn positioning
    LateralOutput lat = lateral_ctrl_.step(ego, fwd, bsm, dt, plan.target_lane, plan.lateral_nudge);

    // 5. Longitudinal control with predictive cornering limit & intersection creep
    LongitudinalOutput lon = longitudinal_ctrl_.step(ego, fwd, dt, plan.speed_limit_override);

    // 6. S-Curve Jerk Limiting on service accel/brake and steering rate (Instant 100% bypass on AEB)
    float final_throttle = 0.0f;
    float final_brake = 0.0f;
    float final_steer = 0.0f;
    jerk_limiter_.process(
        lon.throttle,
        lon.brake,
        lat.steer_cmd,
        lon.mode,
        dt,
        final_throttle,
        final_brake,
        final_steer
    );

    // 7. Arbitrate hazard and horn actuators
    bool hazard = lat.hazard || lon.hazard;
    bool horn = lat.horn || lon.horn;
    if (lat.horn_timer > 0.0f) {
        signal_mgr_.triggerHorn(lat.horn_timer);
    }
    if (lon.horn_timer > 0.0f) {
        signal_mgr_.triggerHorn(lon.horn_timer);
    }

    // 8. Actuator and light signal synthesis
    bool lane_change_sig_active = lat.lane_change_active || plan.lane_change_blinker;
    int lane_change_dir = plan.lane_change_direction;
    if (lane_change_dir == 0 && lat.lane_change_active) {
        lane_change_dir = (lat.desired_lane < ego.current_lane) ? -1 : ((lat.desired_lane > ego.current_lane) ? 1 : 0);
    }
    // Cancel lane change signal if within 0.35m of target lane center
    if (lat.lane_change_active && std::abs(lat.lateral_error) <= 0.35f && ego.current_lane == lat.desired_lane) {
        lane_change_sig_active = false;
        lane_change_dir = 0;
    }

    SignalOutput sig = signal_mgr_.computeSignals(
        hazard,
        horn,
        lane_change_sig_active,
        lat.lateral_error,
        final_brake,
        ego.turn_signal_hint,
        lane_change_dir
    );

    // 9. Assemble final output
    out.throttle = final_throttle;
    out.brake = final_brake;
    out.steer_cmd = final_steer;
    out.left_turn_signal = sig.left_turn_signal;
    out.right_turn_signal = sig.right_turn_signal;
    out.hazard_lights = sig.hazard_lights;
    out.horn_active = sig.horn_active;
    out.brake_lights = sig.brake_lights;
    out.drive_mode = lon.mode;
    out.drive_profile = ego.drive_profile;
    out.lateral_nudge = plan.lateral_nudge;
    out.ack_seq = ego.seq;
    out.watchdog_ok = true;

    // Status message priority arbitration
    if (!lat.bsm_status_message.empty() &&
        lon.mode != DriveMode::AEB_EMERGENCY &&
        lon.mode != DriveMode::STOPPED) {
        out.status_message = lat.bsm_status_message;
    } else if (!plan.behavior_status.empty() &&
               lon.mode != DriveMode::AEB_EMERGENCY &&
               lon.mode != DriveMode::STOPPED) {
        out.status_message = plan.behavior_status;
    } else {
        out.status_message = lon.status_message;
    }

    return out;
}

} // namespace adas
