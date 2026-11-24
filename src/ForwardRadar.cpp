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

        // 1. Path-Aware Obstacle Filtering
        // When route is active, calculate Frenet lateral offset relative to curved path arc:
        // offset ~ 0.5 * curvature * fwd^2
        float lat_lane_curve = 0.0f;
        if (ego.has_route && std::abs(ego.path_curvature) > 0.001f) {
            lat_lane_curve = 0.5f * ego.path_curvature * fwd * fwd;
        }
        float lat_rel = lat - lat_lane_curve;

        // 2. Handle Stop Line Barriers (Type 3)
        if (obs.type == 3) {
            if (std::abs(lat) < 3.2f) {
                float dist = fwd - 2.2f; // Bumper distance to stop line
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

        // 3. Sidewalk Pedestrian Filter (Type 2)
        // Pedestrians walking normally on the sidewalk (|lat| > 1.4m and not moving laterally toward road)
        // must NEVER trigger AEB. Only trigger when actually inside drivable corridor (|lat| <= 1.35m)
        // OR actively crossing into ego lane (|lat| < 2.6m and moving toward lane center).
        if (obs.type == 2) {
            float v_lat = obs.vx * cosH - obs.vz * sinH;
            bool moving_toward_center = (lat_rel > 0.0f && (v_lat < -0.20f || obs.speed < -0.20f)) ||
                                        (lat_rel < 0.0f && (v_lat > 0.20f || obs.speed > 0.20f));
            bool ped_in_corridor = (std::abs(lat_rel) <= 1.35f);
            bool ped_crossing = (std::abs(lat_rel) < 2.6f && moving_toward_center);

            if (ped_in_corridor || ped_crossing) {
                float dist = fwd - 1.0f - 2.2f;
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

        // 4. Vehicles and Buses (Type 0, 1)
        // Tighten active lane collision half-width to +-1.45m (+ vehicle half-width)
        // Reject parked curb cars, adjacent-lane cars, and oncoming vehicles in opposite lanes.
        float max_half_width = 1.45f + (obs.width * 0.5f);
        if (std::abs(lat_rel) <= max_half_width) {
            // Reject oncoming traffic in opposite lane on two-way street
            if (obs.speed < -2.0f && ego.speed > 2.0f && lat_rel < -1.6f) {
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
