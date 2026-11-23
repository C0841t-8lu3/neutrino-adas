#pragma once

#include "adas/IPerceptionSensor.hpp"

namespace adas {

class ForwardRadar : public IForwardScanner {
public:
    ForwardRadar() = default;
    ~ForwardRadar() override = default;

    ForwardScanResult scan(const VehicleState& ego, const std::vector<Obstacle>& obstacles) override;
};

} // namespace adas
