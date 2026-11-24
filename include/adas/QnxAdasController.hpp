#pragma once

#include "adas/Types.hpp"
#include "adas/IPerceptionSensor.hpp"
#include "adas/ForwardRadar.hpp"
#include "adas/BlindSpotMonitor.hpp"
#include "adas/LongitudinalController.hpp"
#include "adas/LateralController.hpp"
#include "adas/SignalActuatorManager.hpp"
#include "adas/BehaviorPlanner.hpp"
#include "adas/JerkLimiter.hpp"
#include <memory>
#include <vector>

namespace adas {

class QnxAdasController {
public:
    explicit QnxAdasController(const AdasConfig& config = AdasConfig());
    ~QnxAdasController() = default;

    AdasOutput step(const VehicleState& ego, const std::vector<Obstacle>& obstacles, float dt);

    void reset();

    void setConfig(const AdasConfig& config);
    const AdasConfig& getConfig() const { return config_; }

    void setForwardScanner(std::unique_ptr<IForwardScanner> scanner) {
        forward_scanner_ = std::move(scanner);
    }
    void setBlindSpotScanner(std::unique_ptr<IBlindSpotScanner> scanner) {
        blind_spot_scanner_ = std::move(scanner);
    }

    LongitudinalController& getLongitudinalController() { return longitudinal_ctrl_; }
    LateralController& getLateralController() { return lateral_ctrl_; }
    SignalActuatorManager& getSignalManager() { return signal_mgr_; }

private:
    AdasConfig config_;
    std::unique_ptr<IForwardScanner> forward_scanner_;
    std::unique_ptr<IBlindSpotScanner> blind_spot_scanner_;
    LongitudinalController longitudinal_ctrl_;
    LateralController lateral_ctrl_;
    SignalActuatorManager signal_mgr_;
};

} // namespace adas
