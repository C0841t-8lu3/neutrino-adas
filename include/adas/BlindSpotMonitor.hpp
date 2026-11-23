#pragma once

#include "adas/IPerceptionSensor.hpp"

namespace adas {

class BlindSpotMonitor : public IBlindSpotScanner {
public:
    BlindSpotMonitor() = default;
    ~BlindSpotMonitor() override = default;

    BlindSpotResult scan(const VehicleState& ego, const std::vector<Obstacle>& obstacles) override;
};

} // namespace adas
