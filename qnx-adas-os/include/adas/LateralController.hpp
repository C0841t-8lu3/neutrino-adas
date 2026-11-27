#pragma once

#include "adas/Types.hpp"
#include <string>

namespace adas {

struct LateralOutput {
    float steer_cmd = 0.0f;
    float lateral_error = 0.0f;
    int desired_lane = 1;
    bool lane_change_active = false;
    bool hazard = false;
    bool horn = false;
    float horn_timer = 0.0f;
    std::string bsm_status_message;
};

class LateralController {
public:
    explicit LateralController(const AdasConfig& config = AdasConfig());
    ~LateralController() = default;

    LateralOutput step(
        const VehicleState& ego,
        const ForwardScanResult& fwd_scan,
        const BlindSpotResult& bsm_scan,
        float dt,
        int planned_target_lane = -1,
        float lateral_nudge = 0.0f
    );

    void reset();

    int getDesiredLane() const { return desired_lane_; }
    void setDesiredLane(int lane) {
        desired_lane_ = lane;
        current_lane_f_ = static_cast<float>(lane);
    }
    bool isLaneChangeActive() const { return lane_change_active_; }

    void setConfig(const AdasConfig& config) { config_ = config; }
    const AdasConfig& getConfig() const { return config_; }

private:
    AdasConfig config_;
    int desired_lane_ = 1;
    bool lane_change_active_ = false;
    float lane_change_timer_ = 0.0f;
    float lc_start_lane_f_ = 1.0f;
    float current_lane_f_ = 1.0f;
    float lc_elapsed_s_ = 0.0f;
    float lc_duration_s_ = 3.2f;
    float prev_lateral_error_ = 0.0f;
    float filtered_e_lat_dot_ = 0.0f;
    bool first_step_ = true;
};

} // namespace adas
