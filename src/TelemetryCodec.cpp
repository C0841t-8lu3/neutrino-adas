#include "ipc/TelemetryCodec.hpp"
#include <cstdio>
#include <cstdlib>

namespace ipc {

std::string TelemetryCodec::generateResponseJson(const adas::AdasOutput& out) {
    char buf[1024];
    int n = std::snprintf(
        buf, sizeof(buf),
        "{\"throttle\":%.3f,\"brake\":%.3f,\"steer_cmd\":%.4f,"
        "\"left_turn_signal\":%s,\"right_turn_signal\":%s,\"hazard_lights\":%s,"
        "\"horn_active\":%s,\"brake_lights\":%s,\"left_mirror_blind_spot\":%s,"
        "\"right_mirror_blind_spot\":%s,\"drive_mode\":%d,\"drive_profile\":%d,\"lateral_nudge\":%.2f,"
        "\"lead_distance\":%.2f,\"lead_rel_speed\":%.2f,\"time_to_collision\":%.2f,"
        "\"ack_seq\":%llu,\"watchdog_ok\":%s,"
        "\"status_message\":\"%s\"}\n",
        out.throttle,
        out.brake,
        out.steer_cmd,
        out.left_turn_signal ? "true" : "false",
        out.right_turn_signal ? "true" : "false",
        out.hazard_lights ? "true" : "false",
        out.horn_active ? "true" : "false",
        out.brake_lights ? "true" : "false",
        out.left_mirror_blind_spot ? "true" : "false",
        out.right_mirror_blind_spot ? "true" : "false",
        static_cast<int>(out.drive_mode),
        static_cast<int>(out.drive_profile),
        out.lateral_nudge,
        out.lead_distance,
        out.lead_rel_speed,
        out.time_to_collision,
        static_cast<unsigned long long>(out.ack_seq),
        out.watchdog_ok ? "true" : "false",
        out.status_message.c_str()
    );
    if (n > 0 && static_cast<size_t>(n) < sizeof(buf)) {
        return std::string(buf, n);
    }
    return std::string(buf);
}

float TelemetryCodec::parseNumber(const std::string& src, const std::string& key) {
    size_t pos = src.find("\"" + key + "\"");
    if (pos == std::string::npos) return 0.0f;
    pos += key.length() + 2;
    while (pos < src.length() && (src[pos] == ' ' || src[pos] == ':')) pos++;
    if (pos >= src.length()) return 0.0f;
    return std::atof(src.c_str() + pos);
}

bool TelemetryCodec::parseTelemetryJson(
    const std::string& str,
    adas::VehicleState& ego,
    std::vector<adas::Obstacle>& obstacles,
    float& dt
) {
    dt = 0.016f; // default 60 Hz frame
    obstacles.clear();

    ego.x = parseNumber(str, "x");
    ego.y = parseNumber(str, "y");
    ego.z = parseNumber(str, "z");
    ego.speed = parseNumber(str, "speed");
    ego.heading = parseNumber(str, "heading");
    ego.steer_angle = parseNumber(str, "steer_angle");
    ego.current_lane = static_cast<int>(parseNumber(str, "current_lane"));
    ego.target_lane = static_cast<int>(parseNumber(str, "target_lane"));
    ego.seq = static_cast<uint64_t>(parseNumber(str, "seq"));

    // Optional FSD Driving Profile (defaults to STANDARD if omitted)
    if (str.find("\"drive_profile\"") != std::string::npos) {
        int prof = static_cast<int>(parseNumber(str, "drive_profile"));
        if (prof == 0) ego.drive_profile = adas::DriveProfile::CHILL;
        else if (prof == 2) ego.drive_profile = adas::DriveProfile::HURRY;
        else ego.drive_profile = adas::DriveProfile::STANDARD;
    } else {
        ego.drive_profile = adas::DriveProfile::STANDARD;
    }

    // Optional Point-to-Point Navigation Route Fields
    ego.has_route = parseNumber(str, "has_route") > 0.5f;
    if (ego.has_route) {
        ego.ref_x = parseNumber(str, "ref_x");
        ego.ref_z = parseNumber(str, "ref_z");
        ego.ref_heading = parseNumber(str, "ref_heading");
        ego.path_curvature = parseNumber(str, "path_curvature");
        ego.next_curvature = (str.find("\"next_curvature\"") != std::string::npos) ? parseNumber(str, "next_curvature") : 0.0f;
        ego.dist_to_goal = parseNumber(str, "dist_to_goal");
        ego.turn_signal_hint = static_cast<int>(parseNumber(str, "turn_signal_hint"));
        ego.dist_to_next_turn = (str.find("\"dist_to_next_turn\"") != std::string::npos) ? parseNumber(str, "dist_to_next_turn") : 999.0f;
        ego.pre_turn_lane_hint = static_cast<int>(parseNumber(str, "pre_turn_lane_hint"));
        ego.in_intersection = (str.find("\"in_intersection\"") != std::string::npos) ? (parseNumber(str, "in_intersection") > 0.5f) : false;
    } else {
        ego.ref_x = 0.0f;
        ego.ref_z = 0.0f;
        ego.ref_heading = 0.0f;
        ego.path_curvature = 0.0f;
        ego.next_curvature = 0.0f;
        ego.dist_to_goal = 99999.0f;
        ego.turn_signal_hint = 0;
        ego.dist_to_next_turn = 999.0f;
        ego.pre_turn_lane_hint = 0;
        ego.in_intersection = (str.find("\"in_intersection\"") != std::string::npos) ? (parseNumber(str, "in_intersection") > 0.5f) : false;
    }

    float parsed_dt = parseNumber(str, "dt");
    if (parsed_dt > 0.001f && parsed_dt < 0.1f) dt = parsed_dt;

    // Parse simple obstacle array: "obs":[{"x":...,"z":...,"speed":...,"type":...}]
    size_t obs_pos = str.find("\"obs\"");
    if (obs_pos != std::string::npos) {
        size_t open_bracket = str.find("[", obs_pos);
        size_t end_bracket = str.find("]", obs_pos);
        if (open_bracket != std::string::npos && end_bracket != std::string::npos && end_bracket > open_bracket) {
            std::string obs_substr = str.substr(open_bracket + 1, end_bracket - open_bracket - 1);
            size_t cur = 0;
            while ((cur = obs_substr.find("{", cur)) != std::string::npos) {
                size_t obj_end = obs_substr.find("}", cur);
                if (obj_end == std::string::npos) break;
                std::string item = obs_substr.substr(cur, obj_end - cur + 1);
                adas::Obstacle o;
                o.x = parseNumber(item, "x");
                o.z = parseNumber(item, "z");
                o.speed = parseNumber(item, "speed");
                o.vx = parseNumber(item, "vx");
                o.vz = parseNumber(item, "vz");
                o.type = static_cast<int>(parseNumber(item, "type"));
                o.length = (o.type == 1) ? 7.0f : (o.type == 3 ? 1.0f : 4.5f);
                o.width = (o.type == 1) ? 2.4f : (o.type == 3 ? 3.6f : 2.0f);
                obstacles.push_back(o);
                cur = obj_end + 1;
            }
        }
    }

    return true;
}

} // namespace ipc
