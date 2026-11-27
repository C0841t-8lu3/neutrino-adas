#pragma once

#include "adas/Types.hpp"

namespace adas {

class JerkLimiter {
public:
    explicit JerkLimiter(const AdasConfig& config = AdasConfig());
    ~JerkLimiter() = default;

    void reset();
    void setConfig(const AdasConfig& config);
    const AdasConfig& getConfig() const { return config_; }

    /**
     * Process raw throttle, brake, and steer commands through S-curve jerk limiting.
     * Zero heap allocations in 60 Hz hot path.
     * Hard Safety Bypass: Instantly delivers 100% emergency brake during AEB_EMERGENCY.
     */
    void process(
        float raw_throttle,
        float raw_brake,
        float raw_steer,
        DriveMode mode,
        float dt,
        float& out_throttle,
        float& out_brake,
        float& out_steer
    );

    float getCurrentAccel() const { return current_accel_; }
    float getCurrentSteer() const { return current_steer_; }

private:
    AdasConfig config_;
    float current_accel_ = 0.0f;
    float current_steer_ = 0.0f;
};

} // namespace adas
