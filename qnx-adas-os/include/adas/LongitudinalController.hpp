#pragma once

#include "adas/Types.hpp"
#include <string>

namespace adas {

struct LongitudinalOutput {
    float throttle = 0.0f;
    float brake = 0.0f;
    DriveMode mode = DriveMode::CRUISING;
    bool hazard = false;
    bool horn = false;
    float horn_timer = 0.0f;
    std::string status_message;
};

class LongitudinalController {
public:
    explicit LongitudinalController(const AdasConfig& config = AdasConfig());
    ~LongitudinalController() = default;

    LongitudinalOutput step(
        const VehicleState& ego,
        const ForwardScanResult& scan,
        float dt,
        float speed_limit_override = 999.0f
    );

    DriveMode getCurrentMode() const { return current_mode_; }
    void reset();

    void setConfig(const AdasConfig& config) { config_ = config; }
    const AdasConfig& getConfig() const { return config_; }

private:
    AdasConfig config_;
    DriveMode current_mode_ = DriveMode::CRUISING;
    int aeb_confirm_count_ = 0;
    float aeb_hold_timer_ = 0.0f;
};

} // namespace adas
