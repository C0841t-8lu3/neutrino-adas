#include "adas/BehaviorPlanner.hpp"
#include <algorithm>
#include <cmath>

namespace adas {

BehaviorPlanner::BehaviorPlanner(const AdasConfig& config)
    : config_(config),
      desired_lane_(1),
      pending_lane_(1),
      lane_change_dir_(0),
      lane_change_prep_timer_(0.0f),
      lane_change_exec_timer_(0.0f),
      lane_change_in_progress_(false),
      blocked_timer_(0.0f),
      current_nudge_(0.0f),
      creep_timer_(0.0f),
      lane_change_cooldown_(0.0f) {}

void BehaviorPlanner::reset() {
    desired_lane_ = 1;
    pending_lane_ = 1;
    lane_change_dir_ = 0;
    lane_change_prep_timer_ = 0.0f;
    lane_change_exec_timer_ = 0.0f;
    lane_change_in_progress_ = false;
    blocked_timer_ = 0.0f;
    current_nudge_ = 0.0f;
    creep_timer_ = 0.0f;
    lane_change_cooldown_ = 0.0f;
}

void BehaviorPlanner::setConfig(const AdasConfig& config) {
    config_ = config;
}

bool BehaviorPlanner::isAdjacentLaneForwardClear(
    const VehicleState& ego,
    const std::vector<Obstacle>& obstacles,
    int direction,
    float min_forward_gap_m
) const {
    float sinH = std::sin(ego.heading);
    float cosH = std::cos(ego.heading);
    float target_lat_center = (direction < 0) ? -3.5f : 3.5f;

    for (const auto& obs : obstacles) {
        if (obs.type == 3) continue;
        float dx = obs.x - ego.x;
        float dz = obs.z - ego.z;
        float fwd = dx * sinH + dz * cosH;
        if (fwd < 0.0f || fwd > min_forward_gap_m) continue;

        float lat = dx * cosH - dz * sinH;
        float lat_curve = ego.has_route ? (0.5f * ego.path_curvature * fwd * fwd) : 0.0f;
        float lat_rel = lat - lat_curve;

        if (std::abs(lat_rel - target_lat_center) < 1.65f) {
            return false;
        }
    }
    return true;
}

BehaviorDecision BehaviorPlanner::plan(
    const VehicleState& ego,
    const ForwardScanResult& fwd,
    const BlindSpotResult& bsm,
    const std::vector<Obstacle>& obstacles,
    float dt
) {
    BehaviorDecision decision;
    decision.target_lane = desired_lane_;
    decision.speed_limit_override = config_.target_speed_mps;

    if (lane_change_cooldown_ > 0.0f) {
        lane_change_cooldown_ = std::max(0.0f, lane_change_cooldown_ - dt);
    }

    // Lane change execution: 1.5s advance blinker prep -> 3.2s S-curve transition -> auto-cancel when centered
    if (lane_change_in_progress_) {
        decision.lane_change_direction = lane_change_dir_;
        decision.lane_change_blinker = true;

        if (lane_change_prep_timer_ > 0.0f) {
            // Abort pre-lane-change if BSM or adjacent forward gap became occupied during 1.5s prep
            bool bsm_blocked = (lane_change_dir_ < 0) ? bsm.left_bsm : bsm.right_bsm;
            if (bsm_blocked || !isAdjacentLaneForwardClear(ego, obstacles, lane_change_dir_, 28.0f)) {
                lane_change_in_progress_ = false;
                decision.lane_change_blinker = false;
                decision.lane_change_direction = 0;
                lane_change_cooldown_ = 1.5f;
            } else {
                lane_change_prep_timer_ -= dt;
                decision.target_lane = desired_lane_;
            }
        } else {
            desired_lane_ = pending_lane_;
            decision.target_lane = desired_lane_;
            decision.request_lane_change = true;
            lane_change_exec_timer_ += dt;

            if ((lane_change_exec_timer_ >= 2.4f && ego.current_lane == desired_lane_) ||
                lane_change_exec_timer_ >= 3.3f) {
                lane_change_in_progress_ = false;
                decision.lane_change_blinker = false;
                decision.lane_change_direction = 0;
                lane_change_exec_timer_ = 0.0f;
                lane_change_cooldown_ = 3.0f;
            }
        }
    }

    bool near_or_in_intersection = ego.in_intersection ||
        (ego.dist_to_next_turn > 0.0f && ego.dist_to_next_turn < 22.0f);

    // 1. Route-Aware Pre-Turn Lane Selection (22m - 95m ahead)
    bool is_preturn_active = false;
    int turn_hint = (ego.pre_turn_lane_hint != 0) ? ego.pre_turn_lane_hint : ego.turn_signal_hint;
    if (ego.has_route && turn_hint != 0 && ego.dist_to_next_turn >= 22.0f &&
        ego.dist_to_next_turn <= 95.0f && !near_or_in_intersection) {
        is_preturn_active = true;
        if (turn_hint < 0 && desired_lane_ > 0 && !lane_change_in_progress_ && lane_change_cooldown_ <= 0.0f) {
            if (!bsm.left_bsm && isAdjacentLaneForwardClear(ego, obstacles, -1, 28.0f)) {
                lane_change_in_progress_ = true;
                pending_lane_ = desired_lane_ - 1;
                lane_change_dir_ = -1;
                lane_change_prep_timer_ = 1.5f;
                lane_change_exec_timer_ = 0.0f;
                decision.behavior_status = "QNX FSD: Pre-Turn Positioning -> Leftmost Turn Lane";
            }
        } else if (turn_hint > 0 && desired_lane_ < 2 && !lane_change_in_progress_ && lane_change_cooldown_ <= 0.0f) {
            if (!bsm.right_bsm && isAdjacentLaneForwardClear(ego, obstacles, +1, 28.0f)) {
                lane_change_in_progress_ = true;
                pending_lane_ = desired_lane_ + 1;
                lane_change_dir_ = +1;
                lane_change_prep_timer_ = 1.5f;
                lane_change_exec_timer_ = 0.0f;
                decision.behavior_status = "QNX FSD: Pre-Turn Positioning -> Rightmost Turn Lane";
            }
        }
    }

    // 2. Proactive Clean Overtake (requires BOTH BSM -12..+4m AND >28m forward gap clear)
    if (!is_preturn_active && !near_or_in_intersection) {
        bool is_blocked = false;
        if (fwd.lead_distance < 28.0f && !fwd.is_lead_stop_line && !fwd.pedestrian_ahead) {
            float lead_speed = ego.speed + fwd.lead_rel_speed;
            if (config_.profile == DriveProfile::CHILL) {
                if (lead_speed < 1.2f) is_blocked = true;
            } else if (config_.profile == DriveProfile::STANDARD) {
                if (lead_speed < (0.72f * config_.target_speed_mps)) is_blocked = true;
            } else {
                if (lead_speed < (0.85f * config_.target_speed_mps)) is_blocked = true;
            }
        }

        if (is_blocked) {
            blocked_timer_ += dt;
        } else {
            blocked_timer_ = std::max(0.0f, blocked_timer_ - dt * 2.0f);
        }

        float escape_threshold = (config_.profile == DriveProfile::HURRY) ? 1.5f : 2.5f;
        if (blocked_timer_ >= escape_threshold && !lane_change_in_progress_ && lane_change_cooldown_ <= 0.0f) {
            bool can_left = (desired_lane_ > 0) && !bsm.left_bsm &&
                            isAdjacentLaneForwardClear(ego, obstacles, -1, 28.0f);
            bool can_right = (desired_lane_ < 2) && !bsm.right_bsm &&
                             isAdjacentLaneForwardClear(ego, obstacles, +1, 28.0f);

            if (desired_lane_ == 1) {
                if (can_left) {
                    lane_change_in_progress_ = true;
                    pending_lane_ = 0;
                    lane_change_dir_ = -1;
                    lane_change_prep_timer_ = 1.5f;
                    lane_change_exec_timer_ = 0.0f;
                    blocked_timer_ = 0.0f;
                    decision.behavior_status = "QNX FSD: Proactive Overtake -> Passing Left";
                } else if (can_right) {
                    lane_change_in_progress_ = true;
                    pending_lane_ = 2;
                    lane_change_dir_ = +1;
                    lane_change_prep_timer_ = 1.5f;
                    lane_change_exec_timer_ = 0.0f;
                    blocked_timer_ = 0.0f;
                    decision.behavior_status = "QNX FSD: Proactive Overtake -> Passing Right";
                }
            } else if (desired_lane_ == 2 && can_left) {
                lane_change_in_progress_ = true;
                pending_lane_ = 1;
                lane_change_dir_ = -1;
                lane_change_prep_timer_ = 1.5f;
                lane_change_exec_timer_ = 0.0f;
                blocked_timer_ = 0.0f;
                decision.behavior_status = "QNX FSD: Blocker Escape -> Center Lane";
            } else if (desired_lane_ == 0 && can_right) {
                lane_change_in_progress_ = true;
                pending_lane_ = 1;
                lane_change_dir_ = +1;
                lane_change_prep_timer_ = 1.5f;
                lane_change_exec_timer_ = 0.0f;
                blocked_timer_ = 0.0f;
                decision.behavior_status = "QNX FSD: Blocker Escape -> Center Lane";
            }
        }
    }

    // 3. Subtle In-Lane Nudge Clearance around close pedestrians/parked edge obstacles
    float target_nudge = 0.0f;
    float sinH = std::sin(ego.heading);
    float cosH = std::cos(ego.heading);

    for (const auto& obs : obstacles) {
        if (obs.type == 3) continue;
        float dx = obs.x - ego.x;
        float dz = obs.z - ego.z;
        float fwd_dist = dx * sinH + dz * cosH;
        float lat_dist = dx * cosH - dz * sinH;

        if (fwd_dist > -1.0f && fwd_dist < 22.0f) {
            float abs_lat = std::abs(lat_dist);
            if (abs_lat >= 1.1f && abs_lat <= 2.4f) {
                bool is_hazard = (obs.type == 2) || (std::abs(obs.speed) < 1.2f);
                if (is_hazard) {
                    if (lat_dist > 0.0f && !bsm.left_bsm) {
                        target_nudge = -0.35f;
                    } else if (lat_dist < 0.0f && !bsm.right_bsm) {
                        target_nudge = +0.35f;
                    }
                    break;
                }
            }
        }
    }

    float max_dnudge = 0.8f * std::max(0.001f, dt);
    float dnudge = std::max(-max_dnudge, std::min(max_dnudge, target_nudge - current_nudge_));
    current_nudge_ += dnudge;
    decision.lateral_nudge = current_nudge_;

    // 4. Predictive Cornering Speed Cap (a_lat,max = 2.2 m/s^2 in STANDARD)
    float max_curv = std::max(std::abs(ego.path_curvature), std::abs(ego.next_curvature));
    if (max_curv > 0.003f) {
        float a_lat = 2.2f;
        if (config_.profile == DriveProfile::CHILL) a_lat = 1.75f;
        else if (config_.profile == DriveProfile::HURRY) a_lat = 2.65f;

        float v_corner = std::sqrt(a_lat / max_curv);
        decision.speed_limit_override = std::min(decision.speed_limit_override, std::max(4.5f, v_corner));
    }

    // 5. Intersection Box-Clearing & Pedestrian Yielding
    if (ego.in_intersection) {
        creep_timer_ = 0.0f;
        decision.is_yielding = false;
    } else if (fwd.is_lead_stop_line) {
        creep_timer_ = 0.0f;
    } else if (fwd.pedestrian_ahead && fwd.lead_distance < 22.0f) {
        decision.is_yielding = true;
        decision.speed_limit_override = std::min(decision.speed_limit_override, 1.5f);
        decision.behavior_status = "QNX FSD: Yielding to Pedestrian";
    } else if (creep_timer_ < 1.2f && ego.speed < 3.5f && !fwd.is_lead_stop_line &&
               fwd.lead_distance > 18.0f && ego.speed > 0.1f) {
        creep_timer_ += dt;
        decision.creep_active = true;
    }

    decision.target_lane = desired_lane_;
    return decision;
}

} // namespace adas
