#include "adas/LongitudinalController.hpp"
#include <cmath>
#include <algorithm>
#include <cstdio>

namespace adas {

LongitudinalController::LongitudinalController(const AdasConfig& config)
    : config_(config), current_mode_(DriveMode::CRUISING) {}

void LongitudinalController::reset() {
    current_mode_ = DriveMode::CRUISING;
    aeb_confirm_count_ = 0;
    aeb_hold_timer_ = 0.0f;
}

LongitudinalOutput LongitudinalController::step(
    const VehicleState& ego,
    const ForwardScanResult& scan,
    float dt,
    float speed_limit_override
) {
    float eff_dt = std::max(0.001f, dt);
    aeb_hold_timer_ = std::max(0.0f, aeb_hold_timer_ - eff_dt);
    LongitudinalOutput out;
    float v = std::max(0.0f, ego.speed);

    // 1. Point-to-Point Destination Arrival Hold
    if (ego.has_route && ego.dist_to_goal <= 3.5f && v < 0.6f) {
        current_mode_ = DriveMode::STOPPED;
        out.throttle = 0.0f;
        out.brake = 0.8f;
        out.status_message = "QNX NAV: Arrived at Destination";
        out.mode = current_mode_;
        return out;
    }

    // 2. Determine Effective Cruise Target Speed (Profile, Curvature & Point B Decel)
    float effective_target_speed = config_.target_speed_mps;
    if (speed_limit_override < effective_target_speed) {
        effective_target_speed = speed_limit_override;
    }

    bool approaching_point_b = false;
    if (ego.has_route) {
        float a_lat = 2.2f;
        if (config_.profile == DriveProfile::CHILL) a_lat = 1.75f;
        else if (config_.profile == DriveProfile::HURRY) a_lat = 2.65f;

        if (std::abs(ego.path_curvature) > 0.003f) {
            float v_turn = std::sqrt(a_lat / std::abs(ego.path_curvature));
            effective_target_speed = std::min(effective_target_speed, std::max(4.5f, v_turn));
        }
        if (std::abs(ego.next_curvature) > 0.003f) {
            float v_next = std::sqrt(a_lat / std::abs(ego.next_curvature));
            effective_target_speed = std::min(effective_target_speed, std::max(4.5f, v_next));
        }
        if (ego.dist_to_goal < 55.0f) {
            float v_goal = std::sqrt(2.0f * 1.65f * std::max(0.05f, ego.dist_to_goal - 1.5f));
            if (v_goal < effective_target_speed) {
                effective_target_speed = v_goal;
                approaching_point_b = true;
            }
        }
    }

    // Intersection Box-Clearing Rule ("Never Stop Inside an Intersection")
    bool ignore_stop_line = (ego.in_intersection && scan.is_lead_stop_line);

    // 3. Tightened Pro-Driver AEB Trigger (genuine collision hazards only; zero yo-yo false triggers)
    float closing_speed = -scan.lead_rel_speed; // Positive when approaching obstacle
    bool imminence_ttc = (scan.time_to_collision < 1.45f && closing_speed > 3.5f && std::abs(scan.lead_lat) < 1.30f);
    bool imminence_gap = (scan.lead_distance < 4.5f && closing_speed > 0.8f);
    bool imminence_ped = (scan.pedestrian_ahead && scan.lead_distance < 18.0f);
    bool aeb_candidate = (!scan.is_lead_stop_line) && (imminence_ttc || imminence_gap || imminence_ped);

    bool aeb_trigger = false;
    if (aeb_candidate) {
        if ((scan.pedestrian_ahead && scan.lead_distance < 12.0f) ||
            (scan.time_to_collision < 1.15f && closing_speed > 3.5f) ||
            imminence_gap) {
            aeb_confirm_count_ = 2;
            aeb_trigger = true;
        } else {
            aeb_confirm_count_++;
            if (aeb_confirm_count_ >= 2) {
                aeb_trigger = true;
            }
        }
    } else {
        aeb_confirm_count_ = 0;
    }

    if (aeb_trigger) {
        aeb_hold_timer_ = 0.35f; // Hysteresis latch prevents rapid AEB chatter across threshold
    } else if (aeb_hold_timer_ > 0.0f && scan.lead_distance < 22.0f && !scan.is_lead_stop_line) {
        aeb_trigger = true;
    }

    if (aeb_trigger) {
        current_mode_ = DriveMode::AEB_EMERGENCY;
        out.throttle = 0.0f;
        out.brake = 1.0f;
        out.hazard = true;
        out.horn = true;
        out.horn_timer = 0.5f;

        char diag_buf[160];
        if (scan.pedestrian_ahead) {
            std::snprintf(diag_buf, sizeof(diag_buf),
                          "QNX AEB: Pedestrian in path at %.1fm (lat: %.2fm, rel_v: %.1fm/s)!",
                          scan.lead_distance, scan.lead_lat, scan.lead_rel_speed);
        } else {
            std::snprintf(diag_buf, sizeof(diag_buf),
                          "QNX AEB: Hazard Type %d at %.1fm (lat: %.2fm, TTC: %.1fs, rel_v: %.1fm/s)!",
                          scan.lead_type, scan.lead_distance, scan.lead_lat, scan.time_to_collision, scan.lead_rel_speed);
        }
        out.status_message = diag_buf;
        out.mode = current_mode_;
        return out;
    }

    // 4. Monotonic Parabolic Stop-Line Approach (Target 1.5 m bumper gap behind stop bar)
    float target_accel = 0.0f;
    float v0 = std::max(0.5f, effective_target_speed);
    float v_ratio = v / v0;
    float free_accel = config_.max_accel * (1.0f - (v_ratio * v_ratio * v_ratio * v_ratio));

    if (scan.is_lead_stop_line && !ignore_stop_line && scan.lead_distance < 65.0f) {
        if (v < 0.35f && scan.lead_distance <= 2.4f) {
            current_mode_ = DriveMode::STOPPED;
            out.throttle = 0.0f;
            out.brake = 0.78f;
            out.status_message = "QNX FSD: Holding at Red Light Stop Line";
            out.mode = current_mode_;
            return out;
        }
        float d_rem = std::max(0.0f, scan.lead_distance - 1.5f);
        float a_approach = 1.75f; // Comfortable kinematic stopping profile (m/s^2)
        float v_stop = std::sqrt(2.0f * a_approach * d_rem);
        float v_des = std::min(effective_target_speed, v_stop);

        if (v_des < effective_target_speed - 0.15f || scan.lead_distance < 48.0f) {
            current_mode_ = DriveMode::ACC_FOLLOWING;
            float speed_err = v_des - v;
            if (speed_err < 0.0f) {
                target_accel = std::max(-config_.max_service_brake, speed_err * 2.1f - 0.55f * a_approach);
            } else {
                // Smooth low-speed creep forward up to the stop bar envelope if needed
                target_accel = std::min(1.1f, speed_err * 0.95f);
            }

            char status_buf[128];
            std::snprintf(status_buf, sizeof(status_buf),
                          "QNX FSD: Stopping at Signal Bar (%.1fm) | Speed: %.1f km/h",
                          scan.lead_distance, v * 3.6f);
            out.status_message = status_buf;
        } else {
            current_mode_ = DriveMode::CRUISING;
            target_accel = free_accel;
            char status_buf[128];
            std::snprintf(status_buf, sizeof(status_buf),
                          "QNX FSD: Approaching Signal Ahead (%.1fm) | Speed: %.1f km/h",
                          scan.lead_distance, v * 3.6f);
            out.status_message = status_buf;
        }
    }
    // 5. IDM Smooth Predictive ACC Following (up to 60 m range -> zero yo-yo brake checking)
    else if (!scan.is_lead_stop_line && scan.lead_distance < 60.0f) {
        current_mode_ = DriveMode::ACC_FOLLOWING;
        float lead_speed = std::max(0.0f, v + scan.lead_rel_speed);
        float s0 = config_.min_safe_dist;
        float T = config_.time_headway;
        float a_max = config_.max_accel;
        float b_comf = config_.idm_b_comfort;

        if (v < 0.45f && lead_speed < 0.6f && scan.lead_distance <= (s0 + 1.5f)) {
            current_mode_ = DriveMode::STOPPED;
            out.throttle = 0.0f;
            out.brake = 0.78f;
            out.status_message = "QNX Safety Stop: Holding Behind Stopped Vehicle";
            out.mode = current_mode_;
            return out;
        }

        float s_star = s0 + std::max(
            0.0f,
            v * T + (v * closing_speed) / (2.0f * std::sqrt(a_max * b_comf))
        );
        float s_ratio = s_star / std::max(0.5f, scan.lead_distance);
        target_accel = a_max * (1.0f - (v_ratio * v_ratio * v_ratio * v_ratio) - (s_ratio * s_ratio));
        target_accel = std::max(-config_.max_service_brake, std::min(config_.max_accel, target_accel));

        char status_buf[128];
        std::snprintf(status_buf, sizeof(status_buf),
                      "QNX IDM ACC: Gap %.1fm (Des %.1fm) | Speed: %.1f km/h",
                      scan.lead_distance, s_star, v * 3.6f);
        out.status_message = status_buf;
    }
    // 6. Free Road Cruising & Point B Smooth Glide Approach
    else {
        current_mode_ = DriveMode::CRUISING;
        float speed_error = effective_target_speed - v;
        if (speed_error >= 0.0f) {
            target_accel = std::min(config_.max_accel, speed_error * 0.9f);
        } else if (approaching_point_b) {
            target_accel = std::max(-config_.max_service_brake, speed_error * 2.1f - 0.8f);
        } else {
            target_accel = std::max(-config_.idm_b_comfort, speed_error * 0.8f);
        }

        if (ego.has_route) {
            char status_buf[128];
            std::snprintf(status_buf, sizeof(status_buf),
                          "QNX NAV: Route Active | Goal: %.0fm | Speed: %.1f km/h",
                          ego.dist_to_goal, v * 3.6f);
            out.status_message = status_buf;
        } else {
            out.status_message = "QNX Cruising 50 km/h | Lane Centering Active | Watchdog OK";
        }
    }

    if (target_accel >= 0.0f) {
        out.throttle = (config_.max_accel > 0.0f) ? std::min(1.0f, target_accel / config_.max_accel) : 0.0f;
        out.brake = 0.0f;
    } else {
        out.throttle = 0.0f;
        float brake_scale = std::max(config_.max_service_brake, config_.emergency_brake_accel);
        out.brake = (brake_scale > 0.0f)
            ? std::min(1.0f, -target_accel / brake_scale)
            : 0.0f;
    }

    out.mode = current_mode_;
    return out;
}

} // namespace adas
