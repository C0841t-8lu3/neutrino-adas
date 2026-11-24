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
