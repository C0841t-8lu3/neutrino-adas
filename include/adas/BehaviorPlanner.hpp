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
     * Pro-Driver urban/highway decision planner (zero heap allocations in hot loop):
     * - Route-aware pre-turn lane selection (20m - 95m ahead)
     * - Proactive overtaking only when BSM (-12..+4m) AND adjacent forward gap (>28m) are clear
     * - Smooth 1.5s advance blinker + auto-cancel on lane centering
     * - Predictive cornering speed cap (a_lat_max ~ 2.2 m/s^2)
     */
    BehaviorDecision plan(
        const VehicleState& ego,
        const ForwardScanResult& fwd,
        const BlindSpotResult& bsm,
        const std::vector<Obstacle>& obstacles,
        float dt
    );

    int getDesiredLane() const { return desired_lane_; }
    void setDesiredLane(int lane) { desired_lane_ = lane; }
    float getCurrentNudge() const { return current_nudge_; }

private:
    bool isAdjacentLaneForwardClear(
        const VehicleState& ego,
        const std::vector<Obstacle>& obstacles,
        int direction,
        float min_forward_gap_m = 28.0f
    ) const;

    AdasConfig config_;
    int desired_lane_ = 1;
    int pending_lane_ = 1;
    int lane_change_dir_ = 0;
    float lane_change_prep_timer_ = 0.0f;
    float lane_change_exec_timer_ = 0.0f;
    bool lane_change_in_progress_ = false;
    float blocked_timer_ = 0.0f;
    float current_nudge_ = 0.0f;
    float creep_timer_ = 0.0f;
    float lane_change_cooldown_ = 0.0f;
};

} // namespace adas
