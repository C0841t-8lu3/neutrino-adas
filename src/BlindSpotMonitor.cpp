#include "adas/BlindSpotMonitor.hpp"
#include <cmath>

namespace adas {

BlindSpotResult BlindSpotMonitor::scan(const VehicleState& ego, const std::vector<Obstacle>& obstacles) {
    BlindSpotResult res;
    res.left_bsm = false;
    res.right_bsm = false;

    float sinH = std::sin(ego.heading);
    float cosH = std::cos(ego.heading);

    for (const auto& obs : obstacles) {
        float dx = obs.x - ego.x;
        float dz = obs.z - ego.z;

        float fwd = dx * sinH + dz * cosH;
        float lat = dx * cosH - dz * sinH;

        // Blind-spot zone: -8.5m behind up to +3.0m beside
        if (fwd >= -8.5f && fwd <= 3.0f) {
            // Left mirror blind spot zone (-4.8m to -1.3m)
            if (lat <= -1.3f && lat >= -4.8f) {
                res.left_bsm = true;
            }
            // Right mirror blind spot zone (+1.3m to +4.8m)
            if (lat >= 1.3f && lat <= 4.8f) {
                res.right_bsm = true;
            }
        }
    }

    return res;
}

} // namespace adas
