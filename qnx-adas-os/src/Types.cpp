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

const char* profileToString(DriveProfile profile) {
    switch (profile) {
        case DriveProfile::CHILL:    return "CHILL";
        case DriveProfile::STANDARD: return "STANDARD";
        case DriveProfile::HURRY:    return "HURRY";
        default:                     return "STANDARD";
    }
}

const char* profileToString(int profile) {
    return profileToString(static_cast<DriveProfile>(profile));
}

} // namespace adas
