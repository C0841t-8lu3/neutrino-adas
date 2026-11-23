#include "adas/LongitudinalController.hpp"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace adas {

LongitudinalController::LongitudinalController(const AdasConfig& config)
    : config_(config), current_mode_(DriveMode::CRUISING) {}

void LongitudinalController::reset() {
    current_mode_ = DriveMode::CRUISING;
}

LongitudinalOutput LongitudinalController::step(const VehicleState& ego, const ForwardScanResult& scan, float dt) {
    (void)dt; // dt unused in instantaneous kinematic state evaluation
    LongitudinalOutput out;

    // Automatic Emergency Braking (AEB)
    if ((scan.time_to_collision < 1.7f && scan.lead_distance < 32.0f) ||
        (scan.lead_distance < 6.5f && ego.speed > 1.0f) ||
        (scan.pedestrian_ahead && scan.lead_distance < 18.0f)) {

        current_mode_ = DriveMode::AEB_EMERGENCY;
        out.throttle = 0.0f;
        out.brake = 1.0f; // 100% emergency brake pressure
        out.hazard = true;
        out.horn = true;
        out.horn_timer = 0.5f;

        if (scan.pedestrian_ahead) {
            out.status_message = "QNX AEB: Pedestrian in path! Emergency Braking & Horn Triggered";
        } else {
            out.status_message = "QNX AEB: Collision hazard! Full Emergency Deceleration (-8.5 m/s2)";
        }
    }
    // Adaptive Cruise Control (ACC) & Intersection Stop-Line Deceleration
    else if (scan.lead_distance < 85.0f) {
        current_mode_ = DriveMode::ACC_FOLLOWING;
        float desired_gap = config_.min_safe_dist + (ego.speed * config_.time_headway);

        // If lead obstacle is stationary (e.g. red light stop line), apply kinematic stopping curve
        bool lead_is_stationary = (std::abs(scan.lead_rel_speed + ego.speed) < 0.5f);
        float target_accel = 0.0f;

        if (lead_is_stationary && ego.speed > 0.5f) {
            float remaining_stopping_dist = std::max(0.5f, scan.lead_distance - config_.min_safe_dist);
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

        if (ego.speed < 0.6f && lead_is_stationary && scan.lead_distance <= 15.0f) {
            current_mode_ = DriveMode::STOPPED;
            out.throttle = 0.0f;
            out.brake = 0.8f;
            out.status_message = "QNX Safety Stop: Holding at Red Light / Stop Line";
        } else {
            std::ostringstream ss;
            ss << "QNX ACC: Following Lead (" << std::fixed << std::setprecision(1) << scan.lead_distance
               << "m) | Speed: " << (ego.speed * 3.6f) << " km/h";
            out.status_message = ss.str();
        }
    }
    // Free Cruising
    else {
        current_mode_ = DriveMode::CRUISING;
        float speed_error = config_.target_speed_mps - ego.speed;
        if (speed_error >= 0.0f) {
            out.throttle = std::min(1.0f, speed_error * 0.45f);
            out.brake = 0.0f;
        } else {
            out.throttle = 0.0f;
            out.brake = std::min(0.6f, -speed_error * 0.30f);
        }

        out.status_message = "QNX Cruising 50 km/h | Lane Centering Active | Watchdog OK";
    }

    if (ego.speed < 0.15f && out.brake > 0.0f && current_mode_ == DriveMode::AEB_EMERGENCY) {
        current_mode_ = DriveMode::STOPPED;
        out.status_message = "QNX Safety Stop: Vehicle Stopped Before Obstacle";
    }

    out.mode = current_mode_;
    return out;
}

} // namespace adas
