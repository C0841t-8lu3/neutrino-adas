#include "adas/LateralController.hpp"
#include <cmath>
#include <algorithm>

namespace adas {

LateralController::LateralController(const AdasConfig& config)
    : config_(config) {}

void LateralController::reset() {
    desired_lane_ = 1;
    lane_change_active_ = false;
    lane_change_timer_ = 0.0f;
    lc_start_lane_f_ = 1.0f;
    current_lane_f_ = 1.0f;
    lc_elapsed_s_ = 0.0f;
    prev_lateral_error_ = 0.0f;
    filtered_e_lat_dot_ = 0.0f;
    first_step_ = true;
}

LateralOutput LateralController::step(
    const VehicleState& ego,
    const ForwardScanResult& fwd_scan,
    const BlindSpotResult& bsm_scan,
    float dt,
    int planned_target_lane,
    float lateral_nudge
) {
    (void)fwd_scan;
    LateralOutput out;

    auto startSmoothLaneChange = [&](int target_lane) {
        lc_start_lane_f_ = current_lane_f_;
        desired_lane_ = std::max(0, std::min(2, target_lane));
        lane_change_active_ = true;
        lc_elapsed_s_ = 0.0f;
        lc_duration_s_ = 3.2f;
        lane_change_timer_ = lc_duration_s_;
    };

    // 1. Behavior Planner Target Lane Integration (already pre-vetted for 1.5s blinker + BSM + 28m gap)
    if (planned_target_lane >= 0 && planned_target_lane != desired_lane_) {
        bool bsm_blocked = (planned_target_lane < desired_lane_ && bsm_scan.left_bsm) ||
                           (planned_target_lane > desired_lane_ && bsm_scan.right_bsm);
        if (!bsm_blocked) {
            startSmoothLaneChange(planned_target_lane);
        }
    }

    // 2. Direct External Target Lane Request & Blind-Spot Accident Prevention
    if (ego.target_lane != ego.current_lane && planned_target_lane < 0) {
        if (ego.target_lane < ego.current_lane) {
            if (bsm_scan.left_bsm) {
                desired_lane_ = ego.current_lane;
                current_lane_f_ = static_cast<float>(desired_lane_);
                lane_change_active_ = false;
                out.hazard = true;
                out.horn = true;
                out.horn_timer = 0.4f;
                out.bsm_status_message = "QNX BSM: ACCIDENT PREVENTED! Left Blind Spot Occupied - Lane Change Aborted";
            } else {
                startSmoothLaneChange(ego.target_lane);
            }
        } else if (ego.target_lane > ego.current_lane) {
            if (bsm_scan.right_bsm) {
                desired_lane_ = ego.current_lane;
                current_lane_f_ = static_cast<float>(desired_lane_);
                lane_change_active_ = false;
                out.hazard = true;
                out.horn = true;
                out.horn_timer = 0.4f;
                out.bsm_status_message = "QNX BSM: ACCIDENT PREVENTED! Right Blind Spot Occupied - Lane Change Aborted";
            } else {
                startSmoothLaneChange(ego.target_lane);
            }
        }
    }

    // Also preserve legacy test path when target_lane != current_lane even with BehaviorPlanner active
    if (ego.target_lane != ego.current_lane) {
        if ((ego.target_lane < ego.current_lane && bsm_scan.left_bsm) ||
            (ego.target_lane > ego.current_lane && bsm_scan.right_bsm)) {
            desired_lane_ = ego.current_lane;
            current_lane_f_ = static_cast<float>(desired_lane_);
            lane_change_active_ = false;
            out.hazard = true;
            out.horn = true;
            out.horn_timer = 0.4f;
            out.bsm_status_message = (ego.target_lane < ego.current_lane)
                ? "QNX BSM: ACCIDENT PREVENTED! Left Blind Spot Occupied - Lane Change Aborted"
                : "QNX BSM: ACCIDENT PREVENTED! Right Blind Spot Occupied - Lane Change Aborted";
        }
    }

    // 3. Quintic Minimum-Jerk S-Curve Lane Change Trajectory over 3.2s
    float lc_heading_ff = 0.0f;
    if (lane_change_active_) {
        lc_elapsed_s_ += std::max(0.001f, dt);
        lane_change_timer_ = std::max(0.0f, lc_duration_s_ - lc_elapsed_s_);
        float tau = std::min(1.0f, lc_elapsed_s_ / lc_duration_s_);
        float tau2 = tau * tau;
        float tau3 = tau2 * tau;
        float s = tau3 * (10.0f + tau * (-15.0f + 6.0f * tau));
        float one_m_tau = 1.0f - tau;
        float s_dot = (30.0f * tau2 * one_m_tau * one_m_tau) / lc_duration_s_;

        current_lane_f_ = lc_start_lane_f_ + (static_cast<float>(desired_lane_) - lc_start_lane_f_) * s;
        float v_lat_traj = (static_cast<float>(desired_lane_) - lc_start_lane_f_) * 3.5f * s_dot;
        if (ego.speed > 1.5f) {
            lc_heading_ff = std::atan2(v_lat_traj, ego.speed);
        }

        if (tau >= 1.0f) {
            current_lane_f_ = static_cast<float>(desired_lane_);
            lane_change_active_ = false;
            lane_change_timer_ = 0.0f;
            lc_heading_ff = 0.0f;
        }
    } else {
        current_lane_f_ = static_cast<float>(desired_lane_);
    }

    // 4. Lateral Error & Heading Error Computation
    float lateral_error = 0.0f;
    float heading_error = 0.0f;

    if (ego.has_route) {
        float dx = ego.x - ego.ref_x;
        float dz = ego.z - ego.ref_z;
        float cos_ref = std::cos(ego.ref_heading);
        float sin_ref = std::sin(ego.ref_heading);
        float cross_track = (dx * cos_ref) - (dz * sin_ref);
        float lane_offset = (current_lane_f_ - 1.0f) * 3.5f + lateral_nudge;
        lateral_error = cross_track - lane_offset;

        heading_error = ego.heading - (ego.ref_heading + lc_heading_ff);
        while (heading_error > 3.14159265f) heading_error -= 6.2831853f;
        while (heading_error < -3.14159265f) heading_error += 6.2831853f;
    } else {
        float target_x = 3.6f + (current_lane_f_ - 1.0f) * 3.5f + lateral_nudge;
        lateral_error = ego.x - target_x;

        heading_error = ego.heading - lc_heading_ff;
        while (heading_error > 3.14159265f) heading_error -= 6.2831853f;
        while (heading_error < -3.14159265f) heading_error += 6.2831853f;
    }

    // 5. Lateral Error Velocity Estimation with Light Low-Pass Filtering
    float eff_dt = std::max(0.005f, dt);
    if (first_step_) {
        first_step_ = false;
        filtered_e_lat_dot_ = 0.0f;
    } else {
        float raw_dot = (lateral_error - prev_lateral_error_) / eff_dt;
        raw_dot = std::max(-3.5f, std::min(3.5f, raw_dot));
        filtered_e_lat_dot_ = 0.65f * filtered_e_lat_dot_ + 0.35f * raw_dot;
    }
    prev_lateral_error_ = lateral_error;

    // 6. Pro-Driver Ackermann Feedforward + Stanley + Lateral Damping:
    // delta = atan(L * kappa_preview) - k_psi * d_psi - atan2(k_e * e_lat, v + 3.5) - k_d * e_lat_dot
    float kappa_preview = ego.has_route ? ego.path_curvature : 0.0f;
    float delta_ff = std::atan(config_.wheelbase_m * kappa_preview);
    float delta_psi = -config_.stanley_k_psi * heading_error;
    float delta_lat = -std::atan2(config_.stanley_k_e * lateral_error, std::max(0.0f, ego.speed) + 3.5f);
    float delta_damp = -config_.stanley_k_d * filtered_e_lat_dot_;

    float steer_cmd = delta_ff + delta_psi + delta_lat + delta_damp;
    steer_cmd = std::max(-config_.stanley_steer_clamp, std::min(config_.stanley_steer_clamp, steer_cmd));

    out.steer_cmd = steer_cmd;
    out.lateral_error = lateral_error;
    out.desired_lane = desired_lane_;
    out.lane_change_active = lane_change_active_;

    return out;
}

} // namespace adas
