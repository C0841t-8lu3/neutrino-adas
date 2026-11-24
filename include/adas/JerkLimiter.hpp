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
