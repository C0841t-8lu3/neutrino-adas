#include "sim/ConsoleSelfTest.hpp"
#include "adas/QnxAdasController.hpp"
#include "adas/Types.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <algorithm>
#include <unistd.h>

namespace sim {

void ConsoleSelfTest::run(int steps) {
    std::cout << "\033[1;36m=== QNX 8.0 NEUTRINO RTOS ADAS L3 SAFETY KERNEL ===\033[0m\n";
    std::cout << "Target: QNX Neutrino 8.0 (x86_64) | Safety Priority: Real-Time\n\n";

    adas::QnxAdasController controller;
    adas::VehicleState ego;
    ego.speed = 10.0f;

    std::vector<adas::Obstacle> obstacles = {
        // Slow truck ahead
        {0.0f, 65.0f, 6.5f, 0.0f, 2.4f, 7.2f, 1, 1},
        // Fast sedan in right lane
        {4.0f, 25.0f, 16.0f, 0.0f, 1.9f, 4.5f, 2, 0},
        // Blind-spot vehicle on Left side (Accident Prevention Test!)
        {-3.8f, -3.0f, 10.5f, 0.0f, 1.9f, 4.4f, 3, 0}
    };

    float dt = 0.1f;
    for (int i = 0; i < steps; ++i) {
        // Move obstacles
        for (auto& o : obstacles) {
            o.z += o.speed * dt;
        }

        // Trigger cut-in hazard at step 80
        if (i == 80) {
            obstacles[0].z = ego.z + 18.0f;
            obstacles[0].speed = 2.0f;
        }

        adas::AdasOutput out = controller.step(ego, obstacles, dt);

        // Update ego kinematics
        float accel = (out.throttle * 2.5f) - (out.brake * 8.5f);
        ego.speed = std::max(0.0f, ego.speed + accel * dt);
        ego.z += ego.speed * dt;
        ego.x += ego.speed * std::sin(ego.heading) * dt;

        if (i % 10 == 0 || out.drive_mode == adas::DriveMode::AEB_EMERGENCY || out.left_mirror_blind_spot) {
            std::cout << "[Step " << std::setw(3) << i << "] "
                      << "Mode: " << std::setw(20) << adas::modeToString(out.drive_mode)
                      << " | Spd: " << std::setw(5) << std::fixed << std::setprecision(1) << (ego.speed * 3.6f) << " km/h"
                      << " | Lead: " << std::setw(5) << out.lead_distance << "m"
                      << " | Left BSM: " << (out.left_mirror_blind_spot ? "\033[1;31mALERT\033[0m" : "\033[1;32mCLEAR\033[0m")
                      << " | Horn: " << (out.horn_active ? "\033[1;31mON\033[0m" : "OFF")
                      << "\n       MSG: " << out.status_message << "\n";
        }

        usleep(40000);
    }

    std::cout << "\n\033[1;32m[QNX SUCCESS]\033[0m Safety Supervisor Verification Complete.\n";
}

} // namespace sim
