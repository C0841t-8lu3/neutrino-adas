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
