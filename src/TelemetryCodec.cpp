#include "ipc/TelemetryCodec.hpp"
#include <sstream>
#include <cstdlib>

namespace ipc {

std::string TelemetryCodec::generateResponseJson(const adas::AdasOutput& out) {
    std::ostringstream ss;
    ss << "{"
       << "\"throttle\":" << out.throttle << ","
       << "\"brake\":" << out.brake << ","
       << "\"steer_cmd\":" << out.steer_cmd << ","
       << "\"left_turn_signal\":" << (out.left_turn_signal ? "true" : "false") << ","
       << "\"right_turn_signal\":" << (out.right_turn_signal ? "true" : "false") << ","
       << "\"hazard_lights\":" << (out.hazard_lights ? "true" : "false") << ","
       << "\"horn_active\":" << (out.horn_active ? "true" : "false") << ","
       << "\"brake_lights\":" << (out.brake_lights ? "true" : "false") << ","
       << "\"left_mirror_blind_spot\":" << (out.left_mirror_blind_spot ? "true" : "false") << ","
       << "\"right_mirror_blind_spot\":" << (out.right_mirror_blind_spot ? "true" : "false") << ","
       << "\"drive_mode\":" << static_cast<int>(out.drive_mode) << ","
       << "\"lead_distance\":" << out.lead_distance << ","
       << "\"lead_rel_speed\":" << out.lead_rel_speed << ","
       << "\"time_to_collision\":" << out.time_to_collision << ","
       << "\"status_message\":\"" << out.status_message << "\""
       << "}\n";
    return ss.str();
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
                o.type = static_cast<int>(parseNumber(item, "type"));
                o.length = (o.type == 1) ? 7.0f : 4.5f;
                o.width = (o.type == 1) ? 2.4f : 2.0f;
                obstacles.push_back(o);
                cur = obj_end + 1;
            }
        }
    }

    return true;
}

} // namespace ipc
