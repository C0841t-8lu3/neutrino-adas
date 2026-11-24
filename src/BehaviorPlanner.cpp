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
    lane_change_in_progress_ = false;
    blocked_timer_ = 0.0f;
    current_nudge_ = 0.0f;
    creep_timer_ = 0.0f;
    lane_change_cooldown_ = 0.0f;
}

void BehaviorPlanner::setConfig(const AdasConfig& config) {
    config_ = config;
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
        lane_change_cooldown_ -= dt;
    }

    // Lane change execution with 1.5s blinker prep rule
    if (lane_change_in_progress_) {
        decision.lane_change_direction = lane_change_dir_;
        decision.lane_change_blinker = true;

        if (lane_change_prep_timer_ > 0.0f) {
            // Signal blinker 1.5s before steering laterally across lane line
            lane_change_prep_timer_ -= dt;
            decision.target_lane = desired_lane_;
        } else {
            // 1.5s prep elapsed: steer laterally to pending lane
            desired_lane_ = pending_lane_;
            decision.target_lane = desired_lane_;
            decision.request_lane_change = true;

            // Auto-cancel blinker once within 0.35m of target lane center
            if (ego.current_lane == desired_lane_) {
                lane_change_in_progress_ = false;
                decision.lane_change_blinker = false;
                decision.lane_change_direction = 0;
                lane_change_cooldown_ = 2.5f;
            }
        }
    }

    // Never initiate a lane change within the final 20m before an intersection or inside an intersection box.
    bool near_or_in_intersection = ego.in_intersection || (ego.dist_to_next_turn > 0.0f && ego.dist_to_next_turn < 20.0f);

    // 1. Route-Aware Pre-Turn Lane Selection (50m - 95m ahead)
    bool is_preturn_active = false;
    int turn_hint = (ego.pre_turn_lane_hint != 0) ? ego.pre_turn_lane_hint : ego.turn_signal_hint;
    if (ego.has_route && turn_hint != 0 && ego.dist_to_next_turn >= 20.0f && ego.dist_to_next_turn <= 95.0f && !near_or_in_intersection) {
        is_preturn_active = true;
        if (turn_hint < 0) {
            // Left turn upcoming -> Target leftmost lane (lane 0)
            if (desired_lane_ > 0 && !lane_change_in_progress_ && lane_change_cooldown_ <= 0.0f) {
                if (!bsm.left_bsm) {
                    lane_change_in_progress_ = true;
                    pending_lane_ = desired_lane_ - 1;
                    lane_change_dir_ = -1;
                    lane_change_prep_timer_ = 1.5f;
                    decision.behavior_status = "QNX FSD: Pre-Turn Positioning -> Leftmost Turn Lane";
                }
            }
        } else if (turn_hint > 0) {
            // Right turn upcoming -> Target rightmost lane (lane 2)
            if (desired_lane_ < 2 && !lane_change_in_progress_ && lane_change_cooldown_ <= 0.0f) {
                if (!bsm.right_bsm) {
                    lane_change_in_progress_ = true;
                    pending_lane_ = desired_lane_ + 1;
                    lane_change_dir_ = +1;
                    lane_change_prep_timer_ = 1.5f;
                    decision.behavior_status = "QNX FSD: Pre-Turn Positioning -> Rightmost Turn Lane";
                }
            }
        }
    }

    // 2. Proactive Overtake & Stopped-Blocker Escape
    // If blocked by a slow or stopped vehicle (not a red-light stop line) for > 2.5s,
    // check adjacent lane clearance + BSM and smoothly pass around it.
    if (!is_preturn_active && !near_or_in_intersection) {
        bool is_blocked = false;
        if (fwd.lead_distance < 28.0f && !fwd.is_lead_stop_line) {
            float lead_speed = ego.speed + fwd.lead_rel_speed;
            if (config_.profile == DriveProfile::CHILL) {
                // CHILL: Only for stopped blockers (< 1.0 m/s)
                if (lead_speed < 1.0f) is_blocked = true;
            } else if (config_.profile == DriveProfile::STANDARD) {
                // STANDARD: Proactive passing of slow vehicles (< 70% target speed)
                if (lead_speed < (0.70f * config_.target_speed_mps)) is_blocked = true;
            } else {
                // HURRY: Higher assertiveness for faster lanes (< 85% target speed)
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
            if (desired_lane_ == 1) {
                if (!bsm.left_bsm) {
                    lane_change_in_progress_ = true;
                    pending_lane_ = 0;
                    lane_change_dir_ = -1;
                    lane_change_prep_timer_ = 1.5f;
                    blocked_timer_ = 0.0f;
                    decision.behavior_status = "QNX FSD: Proactive Overtake -> Passing on Left";
                } else if (!bsm.right_bsm) {
                    lane_change_in_progress_ = true;
                    pending_lane_ = 2;
                    lane_change_dir_ = +1;
                    lane_change_prep_timer_ = 1.5f;
                    blocked_timer_ = 0.0f;
                    decision.behavior_status = "QNX FSD: Proactive Overtake -> Passing on Right";
                }
            } else if (desired_lane_ == 2) {
                if (!bsm.left_bsm) {
                    lane_change_in_progress_ = true;
                    pending_lane_ = 1;
                    lane_change_dir_ = -1;
                    lane_change_prep_timer_ = 1.5f;
                    blocked_timer_ = 0.0f;
                    decision.behavior_status = "QNX FSD: Blocker Escape -> Moving to Center Lane";
                }
            } else if (desired_lane_ == 0) {
                if (!bsm.right_bsm) {
                    lane_change_in_progress_ = true;
                    pending_lane_ = 1;
                    lane_change_dir_ = +1;
                    lane_change_prep_timer_ = 1.5f;
                    blocked_timer_ = 0.0f;
                    decision.behavior_status = "QNX FSD: Blocker Escape -> Moving to Center Lane";
                }
            }
        }
    }

    // 3. Lateral "Nudge" Clearance
    // When a pedestrian, cyclist, or closely parked vehicle is near the lane edge (|lat| < 2.6m),
    // apply a subtle, smooth in-lane lateral bias (±0.45m away from the hazard) just like FSD
    // giving extra room—without crossing into an occupied adjacent lane.
    float target_nudge = 0.0f;
    float sinH = std::sin(ego.heading);
    float cosH = std::cos(ego.heading);

    for (const auto& obs : obstacles) {
        float dx = obs.x - ego.x;
        float dz = obs.z - ego.z;
        float fwd_dist = dx * sinH + dz * cosH;
        float lat_dist = dx * cosH - dz * sinH;

        if (fwd_dist > -1.5f && fwd_dist < 25.0f) {
            float abs_lat = std::abs(lat_dist);
            if (abs_lat >= 0.8f && abs_lat <= 2.6f) {
                bool is_hazard = (obs.type == 2) || (std::abs(obs.speed) < 1.5f);
                if (is_hazard) {
                    if (lat_dist > 0.0f) {
                        // Hazard to the right -> nudge left (-0.45m)
                        if (!bsm.left_bsm) {
                            target_nudge = -0.45f;
                            if (decision.behavior_status.empty()) {
                                decision.behavior_status = "QNX FSD: Lateral Nudge -> Giving Space to Right Hazard";
                            }
                        }
                    } else {
                        // Hazard to the left -> nudge right (+0.45m)
                        if (!bsm.right_bsm) {
                            target_nudge = +0.45f;
                            if (decision.behavior_status.empty()) {
                                decision.behavior_status = "QNX FSD: Lateral Nudge -> Giving Space to Left Hazard";
                            }
                        }
                    }
                    break;
                }
            }
        }
    }

    // Rate-limit lateral nudge smoothly (~1.0 m/s lateral rate)
    float max_dnudge = 1.0f * std::max(0.001f, dt);
    float dnudge = target_nudge - current_nudge_;
    dnudge = std::max(-max_dnudge, std::min(max_dnudge, dnudge));
    current_nudge_ += dnudge;
    decision.lateral_nudge = current_nudge_;

    // 4. Predictive Cornering & Intersection Creep
    // Use lookahead curvature (path_curvature + upcoming next_curvature)
    // to brake smoothly before corner entry and accelerate progressively out of the apex.
    float max_curv = std::max(std::abs(ego.path_curvature), std::abs(ego.next_curvature));
    if (max_curv > 0.005f) {
        float a_lat = 2.4f;
        if (config_.profile == DriveProfile::CHILL) a_lat = 1.8f;
        else if (config_.profile == DriveProfile::HURRY) a_lat = 2.8f;

        float v_corner = std::sqrt(a_lat / max_curv);
        decision.speed_limit_override = std::min(decision.speed_limit_override, v_corner);
