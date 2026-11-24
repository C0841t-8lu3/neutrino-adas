#include "adas/LateralController.hpp"
#include <cmath>
#include <algorithm>

namespace adas {

LateralController::LateralController(const AdasConfig& config)
    : config_(config), desired_lane_(1), lane_change_active_(false), lane_change_timer_(0.0f) {}

void LateralController::reset() {
    desired_lane_ = 1;
    lane_change_active_ = false;
    lane_change_timer_ = 0.0f;
}

LateralOutput LateralController::step(
    const VehicleState& ego,
    const ForwardScanResult& fwd_scan,
    const BlindSpotResult& bsm_scan,
    float dt,
    int planned_target_lane,
    float lateral_nudge
) {
    LateralOutput out;

    // 1. Behavior Planner Target Lane Integration
    if (planned_target_lane >= 0 && planned_target_lane != desired_lane_) {
        bool bsm_blocked = (planned_target_lane < desired_lane_ && bsm_scan.left_bsm) ||
                           (planned_target_lane > desired_lane_ && bsm_scan.right_bsm);
        if (!bsm_blocked) {
            desired_lane_ = planned_target_lane;
            lane_change_active_ = true;
            lane_change_timer_ = 3.2f;
        }
    }

    // 2. Direct Driver Target Lane Request & Blind-Spot Accident Prevention
    if (ego.target_lane != ego.current_lane) {
        if (ego.target_lane < ego.current_lane) {
            // Requested left lane change
            if (bsm_scan.left_bsm) {
                // ACCIDENT PREVENTION: Left mirror blind spot occupied!
                desired_lane_ = ego.current_lane;
                lane_change_active_ = false;
                out.hazard = true;
                out.horn = true;
                out.horn_timer = 0.4f;
                out.bsm_status_message = "QNX BSM: ACCIDENT PREVENTED! Left Blind Spot Occupied - Lane Change Aborted";
            } else {
                desired_lane_ = ego.target_lane;
                lane_change_active_ = true;
                lane_change_timer_ = 3.2f;
            }
        } else if (ego.target_lane > ego.current_lane) {
            // Requested right lane change
            if (bsm_scan.right_bsm) {
                // ACCIDENT PREVENTION: Right mirror blind spot occupied!
                desired_lane_ = ego.current_lane;
                lane_change_active_ = false;
                out.hazard = true;
                out.horn = true;
                out.horn_timer = 0.4f;
                out.bsm_status_message = "QNX BSM: ACCIDENT PREVENTED! Right Blind Spot Occupied - Lane Change Aborted";
            } else {
                desired_lane_ = ego.target_lane;
                lane_change_active_ = true;
                lane_change_timer_ = 3.2f;
            }
        }
    } else if (!lane_change_active_ && fwd_scan.lead_distance < 28.0f && (ego.speed + fwd_scan.lead_rel_speed) < 8.5f) {
        if (desired_lane_ > 0) {
            if (!bsm_scan.left_bsm) {
                // Safe to pass: change to left lane
                desired_lane_ = desired_lane_ - 1;
                lane_change_active_ = true;
                lane_change_timer_ = 3.5f;
            } else {
                // ACCIDENT PREVENTION: Left side mirror detects vehicle in blind spot!
                // QNX safety kernel aborts lane change and maintains safe following!
                desired_lane_ = ego.current_lane;
                out.hazard = true;
                out.bsm_status_message = "QNX BSM: Left Lane Blocked! Aborting Lane Change (Accident Prevented)";
            }
        }
    }

    if (lane_change_active_) {
        lane_change_timer_ -= dt;
        if (lane_change_timer_ <= 0.0f) {
            lane_change_active_ = false;
        }
    }

    // Lateral Steering Control using Stanley Lateral Tracking Controller
    float target_x = 3.6f;
    if (desired_lane_ == 0) {
        target_x = 0.1f;
    } else if (desired_lane_ == 2) {
        target_x = 7.1f;
    }
    float lateral_error = ego.x - target_x;

    // Normalize heading error to [-pi, pi] (road heading along 5th Ave Southbound is 0.0)
    float heading_error = ego.heading;
    while (heading_error > 3.14159265f) heading_error -= 6.2831853f;
    while (heading_error < -3.14159265f) heading_error += 6.2831853f;

    // Stanley controller formulation with velocity scheduling:
    // delta = -(k_psi * heading_error) - atan2(k_e * lateral_error, max(2.5, speed))
    float k_psi = config_.stanley_k_psi;
    float k_e = config_.stanley_k_e;
    float steer_cmd = -(k_psi * heading_error) - std::atan2(k_e * lateral_error, std::max(2.5f, ego.speed));
    steer_cmd = std::max(-config_.stanley_steer_clamp, std::min(config_.stanley_steer_clamp, steer_cmd));

    out.steer_cmd = steer_cmd;
    out.lateral_error = lateral_error;
    out.desired_lane = desired_lane_;
    out.lane_change_active = lane_change_active_;

    return out;
}

} // namespace adas
