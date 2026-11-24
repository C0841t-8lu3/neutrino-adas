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
