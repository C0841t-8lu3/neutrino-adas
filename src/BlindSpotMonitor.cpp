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
        if (obs.type == 3) continue; // Ignore virtual stop lines

        float dx = obs.x - ego.x;
        float dz = obs.z - ego.z;

        float fwd = dx * sinH + dz * cosH;
        float lat = dx * cosH - dz * sinH;

        float lat_curve = ego.has_route ? (0.5f * ego.path_curvature * fwd * fwd) : 0.0f;
        float lat_rel = lat - lat_curve;

        // Pro-Driver BSM coverage: -12.0m rear approach up to +4.0m front fender
        if (fwd >= -12.0f && fwd <= 4.0f) {
            if (lat_rel <= -1.3f && lat_rel >= -4.8f) {
                res.left_bsm = true;
            }
            if (lat_rel >= 1.3f && lat_rel <= 4.8f) {
                res.right_bsm = true;
            }
        }
    }

    return res;
}

} // namespace adas
