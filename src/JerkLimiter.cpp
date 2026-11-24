#include "adas/JerkLimiter.hpp"
#include <algorithm>
#include <cmath>

namespace adas {

JerkLimiter::JerkLimiter(const AdasConfig& config)
    : config_(config), current_accel_(0.0f), current_steer_(0.0f) {}

void JerkLimiter::reset() {
    current_accel_ = 0.0f;
    current_steer_ = 0.0f;
}

void JerkLimiter::setConfig(const AdasConfig& config) {
    config_ = config;
}

void JerkLimiter::process(
    float raw_throttle,
    float raw_brake,
    float raw_steer,
    DriveMode mode,
    float dt,
    float& out_throttle,
    float& out_brake,
    float& out_steer
) {
    // 1. HARD SAFETY BYPASS:
    // If AEB emergency braking is commanded, bypass jerk limiting immediately
    // to provide instantaneous full emergency stopping power in 1 single frame.
    if (mode == DriveMode::AEB_EMERGENCY) {
        out_throttle = 0.0f;
        out_brake = (raw_brake > 0.0f) ? raw_brake : 1.0f;
        current_accel_ = -config_.emergency_brake_accel;
        current_steer_ = raw_steer;
        out_steer = raw_steer;
        return;
    }

    // If transitioning out of AEB emergency braking:
    if (current_accel_ < -config_.max_service_brake) {
        if (raw_brake <= 0.01f) {
            current_accel_ = 0.0f;
        } else {
            current_accel_ = -config_.max_service_brake;
        }
    }

    // 2. Longitudinal S-Curve Jerk Limiting
    float target_accel = 0.0f;
    if (raw_throttle > 0.0f) {
        target_accel = raw_throttle * config_.max_accel;
    } else if (raw_brake > 0.0f) {
        target_accel = -raw_brake * config_.max_service_brake;
    }

    float max_jerk = (config_.max_jerk > 0.0f) ? config_.max_jerk : 2.8f;
    float effective_dt = std::max(0.001f, dt);
    float da_max = max_jerk * effective_dt;

    float da = target_accel - current_accel_;
    da = std::max(-da_max, std::min(da_max, da));
    current_accel_ += da;

    // Small deadband to prevent microscopic floating creep near zero
    if (std::abs(current_accel_) < 0.015f && target_accel == 0.0f) {
        current_accel_ = 0.0f;
    }

    // Translate smoothed acceleration back into throttle / service brake commands
    if (current_accel_ >= 0.0f) {
        out_throttle = (config_.max_accel > 0.0f) ? std::min(1.0f, current_accel_ / config_.max_accel) : 0.0f;
        out_brake = 0.0f;
    } else {
        out_throttle = 0.0f;
        out_brake = (config_.max_service_brake > 0.0f) ? std::min(1.0f, -current_accel_ / config_.max_service_brake) : 0.0f;
    }

    // 3. Lateral Steering Rate Smoothing
    float max_steer_rate = (config_.max_steer_rate > 0.0f) ? config_.max_steer_rate : 1.6f;
    float ds_max = max_steer_rate * effective_dt;

    float ds = raw_steer - current_steer_;
    ds = std::max(-ds_max, std::min(ds_max, ds));
    current_steer_ += ds;
    out_steer = current_steer_;
}

} // namespace adas
