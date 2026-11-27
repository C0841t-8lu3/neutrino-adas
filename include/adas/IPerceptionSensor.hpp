#pragma once

#include "adas/Types.hpp"
#include <vector>

namespace adas {

class IForwardScanner {
public:
    virtual ~IForwardScanner() = default;
    virtual ForwardScanResult scan(const VehicleState& ego, const std::vector<Obstacle>& obstacles) = 0;
};

class IBlindSpotScanner {
public:
    virtual ~IBlindSpotScanner() = default;
    virtual BlindSpotResult scan(const VehicleState& ego, const std::vector<Obstacle>& obstacles) = 0;
};

} // namespace adas
