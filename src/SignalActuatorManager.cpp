#include "adas/SignalActuatorManager.hpp"
#include <algorithm>

namespace adas {

SignalActuatorManager::SignalActuatorManager()
    : blinker_timer_(0.0f), blinker_phase_(false), horn_timer_(0.0f) {}

void SignalActuatorManager::reset() {
    blinker_timer_ = 0.0f;
    blinker_phase_ = false;
    horn_timer_ = 0.0f;
}

void SignalActuatorManager::updateTimers(float dt) {
    // Blinker flash clock (3 Hz)
    blinker_timer_ += dt;
    if (blinker_timer_ >= 0.33f) {
        blinker_timer_ = 0.0f;
        blinker_phase_ = !blinker_phase_;
    }

    // Horn timer
    if (horn_timer_ > 0.0f) {
        horn_timer_ -= dt;
    }
}

void SignalActuatorManager::triggerHorn(float duration_sec) {
    horn_timer_ = std::max(horn_timer_, duration_sec);
}

SignalOutput SignalActuatorManager::computeSignals(
    bool hazard_req,
    bool horn_req,
    bool lane_change_active,
    float lateral_error,
    float brake
) {
    SignalOutput out;

    bool left_blinker = false;
    bool right_blinker = false;

    if (hazard_req) {
        left_blinker = blinker_phase_;
        right_blinker = blinker_phase_;
    } else if (lane_change_active || lateral_error > 0.4f) {
        left_blinker = blinker_phase_;
    } else if (lateral_error < -0.4f) {
        right_blinker = blinker_phase_;
    }

    out.left_turn_signal = left_blinker;
    out.right_turn_signal = right_blinker;
    out.hazard_lights = hazard_req;
    out.horn_active = horn_req || (horn_timer_ > 0.0f);
    out.brake_lights = (brake > 0.15f);

    return out;
}

} // namespace adas
