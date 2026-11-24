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
