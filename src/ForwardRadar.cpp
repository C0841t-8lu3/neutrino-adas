#include "adas/ForwardRadar.hpp"
#include <cmath>
#include <algorithm>

namespace adas {

ForwardScanResult ForwardRadar::scan(const VehicleState& ego, const std::vector<Obstacle>& obstacles) {
    ForwardScanResult res;
    res.lead_distance = 999.0f;
    res.lead_rel_speed = 0.0f;
    res.lead_lat = 0.0f;
    res.time_to_collision = 999.0f;
    res.pedestrian_ahead = false;
    res.is_lead_stop_line = false;
    res.lead_type = 0;

    float sinH = std::sin(ego.heading);
    float cosH = std::cos(ego.heading);

    for (const auto& obs : obstacles) {
        float dx = obs.x - ego.x;
        float dz = obs.z - ego.z;

        // Project into ego vehicle coordinate frame
        float fwd = dx * sinH + dz * cosH;
        float lat = dx * cosH - dz * sinH;

        // Forward detection corridor
        if (fwd > 0.8f && fwd < 85.0f && std::abs(lat) < 2.3f) {
            float dist = fwd - (obs.length * 0.5f) - 2.2f;
            if (dist < res.lead_distance) {
                res.lead_distance = std::max(0.1f, dist);
                res.lead_rel_speed = obs.speed - ego.speed;
                if (obs.type == 2) {
                    res.pedestrian_ahead = true;
                }
            }
        }
    }

    if (res.lead_rel_speed < -0.1f && res.lead_distance < 85.0f) {
        res.time_to_collision = res.lead_distance / (-res.lead_rel_speed);
    }

    return res;
}

} // namespace adas
