#include "adas/Types.hpp"

namespace adas {

const char* modeToString(DriveMode mode) {
    switch (mode) {
        case DriveMode::CRUISING:              return "CRUISING";
        case DriveMode::ACC_FOLLOWING:         return "ACC FOLLOWING";
        case DriveMode::LANE_CHANGING:         return "LANE CHANGING";
        case DriveMode::TURNING_INTERSECTION:  return "INTERSECTION TURN";
        case DriveMode::AEB_EMERGENCY:         return "AEB EMERGENCY BRAKE";
        case DriveMode::STOPPED:               return "STOPPED";
        case DriveMode::MANUAL:                return "MANUAL";
        default:                               return "UNKNOWN";
    }
}

const char* modeToString(int mode) {
    return modeToString(static_cast<DriveMode>(mode));
}

} // namespace adas
