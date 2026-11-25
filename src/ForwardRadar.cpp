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

        if (fwd < 0.6f || fwd > 85.0f) continue;

        // 1. Exact Curved-Route Corridor Projection
        // Compute road lateral offset at forward range `fwd` along curved path arc.
        float lat_lane_curve = 0.0f;
        if (ego.has_route && std::abs(ego.path_curvature) > 0.002f) {
            lat_lane_curve = (1.0f - std::cos(ego.path_curvature * fwd)) / ego.path_curvature;
        } else if (ego.has_route && std::abs(ego.path_curvature) > 0.0005f) {
            lat_lane_curve = 0.5f * ego.path_curvature * fwd * fwd;
        }
        float lat_rel = lat - lat_lane_curve;

        // 2. Red/Yellow Stop Line Barriers (Type 3)
        if (obs.type == 3) {
            if (std::abs(lat_rel) < 2.6f || std::abs(lat) < 2.6f) {
                float dist = fwd - 2.2f; // Front bumper to painted stop bar
                if (dist < res.lead_distance) {
                    res.lead_distance = std::max(0.1f, dist);
                    res.lead_rel_speed = -ego.speed;
                    res.lead_lat = lat_rel;
                    res.lead_type = 3;
                    res.pedestrian_ahead = false;
                    res.is_lead_stop_line = true;
                }
            }
            continue;
        }

        // 3. Pedestrian / Cyclist Corridor & Convergence Filter (Type 2)
        // Sidewalk pedestrians not moving toward ego lane are strictly ignored;
        // pedestrians actively crossing toward ego lane (|lat_rel| < 4.8m, |v_lat| > 0.5m/s)
        // trigger early predictive slowing & AEB before stepping directly onto bumper.
        if (obs.type == 2) {
            float v_lat = obs.vx * cosH - obs.vz * sinH;
            bool moving_toward_center = (lat_rel > 0.0f && v_lat < -0.50f) ||
                                        (lat_rel < 0.0f && v_lat > 0.50f);
            bool ped_in_corridor = (std::abs(lat_rel) <= 1.35f);
            bool ped_crossing = (std::abs(lat_rel) < 4.8f && moving_toward_center);

            if (ped_in_corridor || ped_crossing) {
                float dist = fwd - 0.8f - 2.2f;
                if (dist < res.lead_distance) {
                    res.lead_distance = std::max(0.1f, dist);
                    res.lead_rel_speed = obs.speed - ego.speed;
                    res.lead_lat = lat_rel;
                    res.lead_type = 2;
                    res.pedestrian_ahead = true;
                    res.is_lead_stop_line = false;
                }
            }
            continue;
        }

        // 4. Pro-Driver Tight Vehicle Corridor (Type 0, 1)
        // Strictly reject adjacent-lane vehicles even around arches using 1.30m half-width.
        if (std::abs(lat_rel) <= 1.30f) {
            // Reject oncoming vehicles across narrow road dividers
            if (obs.speed < -2.0f && ego.speed > 1.0f && lat_rel < -1.05f) {
                continue;
            }

            float dist = fwd - (obs.length * 0.5f) - 2.2f;
            if (dist < res.lead_distance) {
                res.lead_distance = std::max(0.1f, dist);
                res.lead_rel_speed = obs.speed - ego.speed;
                res.lead_lat = lat_rel;
                res.lead_type = obs.type;
                res.pedestrian_ahead = false;
                res.is_lead_stop_line = false;
            }
        }
    }

    if (res.lead_rel_speed < -0.1f && res.lead_distance < 85.0f) {
        res.time_to_collision = res.lead_distance / (-res.lead_rel_speed);
    }

    return res;
}

} // namespace adas
