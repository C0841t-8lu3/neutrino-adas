#pragma once

#include "adas/Types.hpp"
#include <vector>
#include <string>

namespace adas {

struct BehaviorDecision {
    int target_lane = 1;
    bool request_lane_change = false;
    int lane_change_direction = 0;       // -1: left, +1: right, 0: none
    bool lane_change_blinker = false;    // true if lane-change blinker should be active
    float lateral_nudge = 0.0f;          // in-lane bias in meters ([-0.45, +0.45])
    float speed_limit_override = 999.0f; // m/s
    bool is_yielding = false;
    bool creep_active = false;
    std::string behavior_status;
};

class BehaviorPlanner {
public:
    explicit BehaviorPlanner(const AdasConfig& config = AdasConfig());
    ~BehaviorPlanner() = default;

    void reset();
    void setConfig(const AdasConfig& config);
    const AdasConfig& getConfig() const { return config_; }

    /**
     * Executes human-like high-level urban driving behavior decisions:
     * - Route-Aware Pre-Turn Lane Selection (within 90m of turn)
     * - Proactive Overtake & Stopped-Blocker Escape (>2.5s blocked)
     * - Lateral "Nudge" Clearance (±0.45m in-lane bias away from hazards)
     * - Predictive Cornering & Intersection Creep
     *
     * Zero heap allocations in 60 Hz hot path.
     */
    BehaviorDecision plan(
        const VehicleState& ego,
        const ForwardScanResult& fwd,
        const BlindSpotResult& bsm,
        const std::vector<Obstacle>& obstacles,
