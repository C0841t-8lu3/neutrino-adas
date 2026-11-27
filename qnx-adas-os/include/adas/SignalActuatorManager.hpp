#pragma once

namespace adas {

struct SignalOutput {
    bool left_turn_signal = false;
    bool right_turn_signal = false;
    bool hazard_lights = false;
    bool horn_active = false;
    bool brake_lights = false;
};

class SignalActuatorManager {
public:
    SignalActuatorManager();
    ~SignalActuatorManager() = default;

    void updateTimers(float dt);
    void triggerHorn(float duration_sec);

    SignalOutput computeSignals(
        bool hazard_req,
        bool horn_req,
        bool lane_change_active,
        float lateral_error,
        float brake,
        int turn_signal_hint = 0,
        int lane_change_direction = 0
    );

    void reset();

    bool getBlinkerPhase() const { return blinker_phase_; }
    float getBlinkerTimer() const { return blinker_timer_; }
    float getHornTimer() const { return horn_timer_; }

private:
    float blinker_timer_ = 0.0f;
    bool blinker_phase_ = false;
    float horn_timer_ = 0.0f;
};

} // namespace adas
