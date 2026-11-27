#pragma once

#include "adas/Types.hpp"
#include <string>
#include <vector>

namespace ipc {

class TelemetryCodec {
public:
    static std::string generateResponseJson(const adas::AdasOutput& out);
    static bool parseTelemetryJson(
        const std::string& str,
        adas::VehicleState& ego,
        std::vector<adas::Obstacle>& obstacles,
        float& dt
    );

private:
    static float parseNumber(const std::string& src, const std::string& key);
};

} // namespace ipc
