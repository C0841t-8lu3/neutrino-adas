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
}

LongitudinalOutput LongitudinalController::step(
    const VehicleState& ego,
    const ForwardScanResult& scan,
    float dt,
    float speed_limit_override
) {
    (void)dt;
    LongitudinalOutput out;

    // Check Point-to-Point Destination Arrival
    if (ego.has_route && ego.dist_to_goal <= 3.0f && ego.speed < 0.6f) {
        current_mode_ = DriveMode::STOPPED;
        out.throttle = 0.0f;
        out.brake = 0.8f;
        out.status_message = "QNX NAV: Arrived at Destination";
        out.mode = current_mode_;
        return out;
    }

    // Determine target speed with behavior planner limit, path curvature, next lookahead curvature, and destination approach
    float effective_target_speed = config_.target_speed_mps;
    if (speed_limit_override < effective_target_speed) {
        effective_target_speed = speed_limit_override;
    }

    if (ego.has_route) {
        float a_lat = (config_.profile == DriveProfile::HURRY) ? 2.8f : ((config_.profile == DriveProfile::CHILL) ? 1.8f : 2.4f);

        // Current waypoint curvature
        if (std::abs(ego.path_curvature) > 0.005f) {
            float v_turn = std::sqrt(a_lat / std::abs(ego.path_curvature));
            effective_target_speed = std::min(effective_target_speed, v_turn);
        }
        // Upcoming lookahead curvature (predictive cornering before entry)
        if (std::abs(ego.next_curvature) > 0.005f) {
            float v_next = std::sqrt(a_lat / std::abs(ego.next_curvature));
            effective_target_speed = std::min(effective_target_speed, v_next);
        }
        // Smooth kinematic deceleration towards destination
        if (ego.dist_to_goal < 40.0f) {
            float v_goal = std::sqrt(2.0f * config_.max_service_brake * std::max(0.1f, ego.dist_to_goal - 2.5f));
            effective_target_speed = std::min(effective_target_speed, v_goal);
        }
    }

    // Intersection Box-Clearing Rule ("Never Stop Inside an Intersection"):
    // Once inside an intersection, never stop for newly-red light or virtual stop line.
    bool ignore_stop_line = (ego.in_intersection && scan.is_lead_stop_line);

    // Automatic Emergency Braking (AEB) with Multi-Frame Confirmation & Diagnostic Cause
    bool aeb_candidate = (!scan.is_lead_stop_line) && (
        (scan.time_to_collision < 1.7f && scan.lead_distance < 32.0f) ||
        (scan.lead_distance < 6.5f && ego.speed > 1.0f) ||
        (scan.pedestrian_ahead && scan.lead_distance < 18.0f)
    );

    bool aeb_trigger = false;
    if (aeb_candidate) {
        // Critical imminent hazard (<4.0m, low TTC < 1.3s, or close pedestrian < 12m): trigger in 1 frame
        if (scan.lead_distance < 4.0f || scan.time_to_collision < 1.3f || (scan.pedestrian_ahead && scan.lead_distance < 12.0f)) {
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
        current_mode_ = DriveMode::AEB_EMERGENCY;
        out.throttle = 0.0f;
        out.brake = 1.0f; // 100% emergency brake pressure
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
    }
    // Adaptive Cruise Control (ACC) & Intersection Stop-Line Deceleration
    else if (scan.lead_distance < 85.0f && !ignore_stop_line) {
        current_mode_ = DriveMode::ACC_FOLLOWING;
        float desired_gap = (scan.is_lead_stop_line)
            ? 1.2f // Front bumper stops 1.2m behind painted stop line (0.8m-1.8m safe zone)
            : (config_.min_safe_dist + (ego.speed * config_.time_headway));

        bool lead_is_stationary = scan.is_lead_stop_line || (std::abs(scan.lead_rel_speed + ego.speed) < 0.5f);
        float target_accel = 0.0f;

        if (lead_is_stationary && ego.speed > 0.5f) {
            float remaining_stopping_dist = std::max(0.5f, scan.lead_distance - desired_gap);
            float req_decel = (ego.speed * ego.speed) / (2.0f * remaining_stopping_dist);
            target_accel = -std::min(config_.max_service_brake, std::max(0.8f, req_decel));
        } else {
            float gap_error = scan.lead_distance - desired_gap;
            target_accel = (0.55f * gap_error) + (0.85f * scan.lead_rel_speed);
        }

        if (target_accel >= 0.0f) {
            out.throttle = std::min(1.0f, target_accel / config_.max_accel);
            out.brake = 0.0f;
        } else {
            out.throttle = 0.0f;
            out.brake = std::min(1.0f, -target_accel / config_.max_service_brake);
        }

        if (ego.speed < 0.6f && lead_is_stationary && scan.lead_distance <= 3.0f) {
            current_mode_ = DriveMode::STOPPED;
            out.throttle = 0.0f;
            out.brake = 0.8f;
            out.status_message = scan.is_lead_stop_line
                ? "QNX FSD: Holding at Red Light Stop Line"
                : "QNX Safety Stop: Holding Behind Stopped Vehicle";
        } else {
            char status_buf[128];
            if (scan.is_lead_stop_line) {
                std::snprintf(status_buf, sizeof(status_buf),
                              "QNX FSD: Decelerating to Red Light Stop Line (%.1fm) | Speed: %.1f km/h",
                              scan.lead_distance, ego.speed * 3.6f);
            } else {
                std::snprintf(status_buf, sizeof(status_buf),
                              "QNX ACC: Following Lead (%.1fm) | Speed: %.1f km/h",
                              scan.lead_distance, ego.speed * 3.6f);
            }
            out.status_message = status_buf;
        }
    }
    // Free Cruising
    else {
        current_mode_ = DriveMode::CRUISING;
        float speed_error = effective_target_speed - ego.speed;
        if (speed_error >= 0.0f) {
            out.throttle = std::min(1.0f, speed_error * 0.45f);
            out.brake = 0.0f;
        } else {
            out.throttle = 0.0f;
            out.brake = std::min(0.6f, -speed_error * 0.30f);
        }

        if (ego.has_route) {
            char status_buf[128];
            std::snprintf(status_buf, sizeof(status_buf),
                          "QNX NAV: Route Active | Goal: %.0fm | Speed: %.1f km/h",
                          ego.dist_to_goal, ego.speed * 3.6f);
            out.status_message = status_buf;
        } else {
            out.status_message = "QNX Cruising 50 km/h | Lane Centering Active | Watchdog OK";
        }
    }

    if (ego.speed < 0.15f && out.brake > 0.0f && current_mode_ == DriveMode::AEB_EMERGENCY) {
        current_mode_ = DriveMode::STOPPED;
        out.status_message = "QNX Safety Stop: Vehicle Stopped Before Obstacle";
    }

    out.mode = current_mode_;
    return out;
}

} // namespace adas
