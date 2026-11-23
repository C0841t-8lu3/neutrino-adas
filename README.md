# Neutrino ADAS: Real-Time Autonomous Driving on QNX 8.0 RTOS

<div align="center">

[![QNX](https://img.shields.io/badge/QNX-Neutrino%208.0-red.svg?style=for-the-badge&logo=blackberry)](https://blackberry.qnx.com)
[![Rust](https://img.shields.io/badge/Rust-1.80+-orange.svg?style=for-the-badge&logo=rust)](https://www.rust-lang.org/)
[![Three.js](https://img.shields.io/badge/Three.js-WebGPU%20TSL-blue.svg?style=for-the-badge&logo=three.js)](https://threejs.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20QNX%20x86__64%20%7C%20aarch64-green.svg?style=for-the-badge)]()

**An empirical, mission-critical ADAS (Advanced Driver Assistance Systems) platform integrating a deterministic QNX 8.0 Neutrino RTOS L3 Safety Kernel with a procedural 1:1 scale Manhattan 3D simulation (Three.js WebGPU / TSL) and a high-performance native Rust desktop runtime.**

[Key Features](#-key-features) • [System Architecture](#-system-architecture) • [Empirical Benchmarks](#-empirical-benchmarks--optimization) • [Quickstart](#-quickstart--build-guide) • [Acknowledgements](#-acknowledgements--credits)

</div>

---

## 🌟 Overview

**Neutrino ADAS** demonstrates a real-world automotive architecture where hard real-time vehicle dynamics and safety-critical decisions run within **BlackBerry QNX Neutrino RTOS 8.0**, decoupled from non-real-time visualization. 

The vehicle operates inside a true-to-scale, 1:1 procedural Manhattan digital twin ("Slop New York" by Mr.doob), receiving simulated multi-sensor telemetry (forward radar, blind-spot detectors, traffic signal network) and transmitting microsecond-level actuation commands (steering angle, throttle, service brakes, turn blinkers, horn) over a low-latency IPC socket bridge.

Driving accuracy is **measured, not guessed**: the platform incorporates a headless Chrome DevTools Protocol (CDP) automated benchmarking engine that samples 5 core ADAS metrics at 60 FPS across fixed test routes on 5th Avenue.

---

## 🏛 System Architecture

The project employs a clean three-tier distributed architecture:

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                              QNX 8.0 NEUTRINO RTOS VM                                  │
│                                                                                        │
│   ┌────────────────────────────────────────────────────────────────────────────────┐   │
│   │                      ADAS L3 Real-Time Safety Kernel                           │   │
│   │                                                                                │   │
│   │   • Nonlinear Stanley Lateral Tracking Controller                              │   │
│   │   • Kinematic Intersection Stop-Line Deceleration Profile                      │   │
│   │   • Dual-Mirror Blind Spot Monitoring (BSM) Radar Engines (-8.5m to +3.0m)     │   │
│   │   • Automatic Emergency Braking (AEB, TTC < 1.7s @ -8.5 m/s²)                  │   │
│   │   • Adaptive Cruise Control (ACC) & Stop-and-Hold Service Brake                │   │
│   │   • High-Priority POSIX Non-Blocking Socket Server (Port 9090)                 │   │
│   └───────────────────────────────────────┬────────────────────────────────────────┘   │
└───────────────────────────────────────────┼────────────────────────────────────────────┘
                                     TCP / IP (9090)
┌───────────────────────────────────────────┼────────────────────────────────────────────┐
│                       RUST NATIVE DESKTOP HOST APPLICATION                             │
│                                           │                                            │
│   ┌───────────────────────────────────────┴────────────────────────────────────────┐   │
│   │                       Asynchronous IPC & Asset Bridge                          │   │
│   │                                                                                │   │
│   │   • Multithreaded Static Asset HTTP Server (Port 8088)                         │   │
│   │   • Full-Duplex WebSocket IPC Server (Port 8089)                               │   │
│   │   • Dedicated TCP Client Relay to QNX Virtual Machine                          │   │
│   │   • Native Borderless WebGPU Application Window Launcher                       │   │
│   │   • Ultra-Low-Latency Telemetry Forwarding (< 0.2ms socket jitter)             │   │
│   └───────────────────────────────────────┬────────────────────────────────────────┘   │
└───────────────────────────────────────────┼────────────────────────────────────────────┘
                                    WebSocket (8089)
┌───────────────────────────────────────────┼────────────────────────────────────────────┐
│                      3D SIMULATION ENGINE (SLOP NEW YORK)                              │
│                                           │                                            │
│   ┌───────────────────────────────────────┴────────────────────────────────────────┐   │
│   │                      WebGPU / Three.js TSL Runtime                             │   │
│   │                                                                                │   │
│   │   • True 1:1 Scale Manhattan Road Graph & Island Footprints (5th Ave Corridor) │   │
│   │   • Real-Time Dual Side-View Mirror Offscreen Canvas Projections               │   │
│   │   • Procedural Traffic Flow, Ambient City Buses, & Pedestrian Crowds           │   │
│   │   • Signal Phase Timing Network (Avenue vs. Street 60s Cycles)                 │   │
│   │   • 60 FPS Telemetry Sensors & Actuator Dynamic Response                       │   │
│   └────────────────────────────────────────────────────────────────────────────────┘   │
└────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## ⚡ Key Features

* **Nonlinear Stanley Lateral Tracking Controller:**
  Computes heading alignment ($\Delta\psi = \psi_{\text{ego}} - \psi_{\text{road}}$) combined with a velocity-normalized cross-track arctangent term $\arctan\left(\frac{k_e \cdot e_{\text{lat}}}{\max(2.5, v)}\right)$. Replaces unstable linear P-D control, delivering sub-centimeter lane accuracy at $50\,\text{km/h}$.
* **Intersection Stop-Line Deceleration & Traffic Light Awareness:**
  Scans upcoming intersection nodes dynamically, projects virtual stationary stopping barriers during amber/red phases, and calculates kinematic stopping curves $a_{\text{req}} = -\frac{v^2}{2(d - d_{\text{safe}})}$ with an active $80\%$ service brake hold at rest.
* **Dual Real-Time Side-View Mirrors:**
  Independent off-screen 2D canvas projections with perspective vanishing points and rearward optic deflection, rendering approaching vehicles with distance callout tags and 3D lighting.
* **Blind Spot Monitoring (BSM) & Accident Prevention Interlock:**
  QNX radar algorithms continuously scan left ($-4.8\,\text{m}$ to $-1.3\,\text{m}$) and right ($+1.3\,\text{m}$ to $+4.8\,\text{m}$) blind zones. Any unsafe manual or autonomous lane change is automatically aborted with visual HUD warnings and acoustic alerts.
* **Automatic Emergency Braking (AEB) & Adaptive Cruise Control (ACC):**
  Monitors forward Time-to-Collision ($\text{TTC}$). Full emergency deceleration ($-8.5\,\text{m/s}^2$) triggers if $\text{TTC} < 1.7\,\text{s}$ or if a pedestrian steps into the vehicle path.
* **Automated CDP Benchmarking Harness:**
  Built-in Python test runner (`run_benchmark.py`) attaching directly to WebGPU via Chrome DevTools Protocol to measure and verify metrics with zero human guesswork.

---

## 📊 Empirical Benchmarks & Optimization

Every improvement in this repository was recorded empirically via the automated CDP runner across identical test routes on 5th Avenue (Southbound from 35th St).

### Iteration 1: Lateral Stability (Stanley Controller)
* **Failure Mode:** Unreferenced coordinate tracking and velocity-independent P-D gains saturated steering to $-1.0\,\text{rad}$ from frame 0, resulting in 3 building/curb collisions within 2.5 seconds.
* **Fix:** Implemented the nonlinear Stanley steering controller centered on 5th Ave Lane 1 ($x = 3.6\,\text{m}$) with speed-normalized damping.

| Metric | Baseline (Pre-Fix 1) | Post-Fix 1 (Stanley) | Improvement |
| :--- | :---: | :---: | :---: |
| **RMS Lane-Keeping Offset** | **$12.739\,\text{m}$** | **$0.006\,\text{m}$** | **$-99.95\%$ (Sub-centimeter)** |
| **Mean Lane Offset** | $11.424\,\text{m}$ | $0.005\,\text{m}$ | **$-99.96\%$** |
| **Peak Lane Offset** | $17.869\,\text{m}$ | $0.021\,\text{m}$ | **$-99.88\%$** |
| **Mean Steering Error** | $28.83^\circ$ | $0.05^\circ$ | **$-99.83\%$** |
| **Collision Count** | **$3$ crashes** | **$0$ crashes** | **$100\%$ eliminated** |
| **Near-Misses ($\text{TTC} < 1.5\,\text{s}$)** | $1$ | $0$ | **$100\%$ eliminated** |

### Iteration 2: Traffic Light Compliance & Stop-Line Halting
* **Failure Mode:** Vehicle cruised through red lights at $13.8\,\text{m/s}$ ($0\%$ compliance) because unsorted telemetry payloads truncated stop line barriers behind ambient city traffic.
* **Fix:** Priority injection of virtual stop lines at index 0 of QNX telemetry, Euclidean distance sorting for surrounding obstacles, kinematic stopping profiles, and service brake holding.

| Metric | Pre-Fix 2 (Red Approach) | Post-Fix 2 (Red Approach) | Improvement |
| :--- | :---: | :---: | :---: |
| **Red Light Violations** | **$1$ violation ($13.8\,\text{m/s}$)** | **$0$ violations ($0.0\,\text{m/s}$)** | **Zero violations** |
| **Signal Compliance Rate** | **$50.0\%$** | **$100.0\%$** | **$+50.0\%$ (Perfect compliance)** |
| **Collision Count** | $0$ | $0$ | Maintained zero crashes |
| **Service Brake Engagement** | Never braked | $3$ controlled decelerations | Smooth stop at line |

---

## 🛠 Quickstart & Build Guide

### Prerequisites
- **Ubuntu 22.04+ / Linux x86_64**
- **QNX Software Development Platform (SDP) 8.0**
- **Rust & Cargo** (`1.80+`)
- **Python 3.10+** (with `websockets` package)
- **Google Chrome** (supporting WebGPU)

### 1. Build the QNX ADAS Safety Kernel
Compile the POSIX ADAS daemon for both x86_64 and aarch64 targets using the QNX SDP toolchain:
```bash
# Source the QNX SDP environment
source /home/aauel/qnx800/qnxsdp-env.sh

# Build all architecture targets
make clean && make
```
The output ELF binary is generated at `nto/x86_64/o/helloworld`.

### 2. Build the Native Rust Desktop Host
Compile the multi-threaded HTTP/WebSocket proxy daemon:
```bash
cargo build --release
```
The executable is generated at `target/release/qnx_desktop_app`.

### 3. Launch the Complete System
Use the unified launcher script to check QNX connectivity, launch background daemons, and open the desktop application window:
```bash
chmod +x launch_qnx_autonomous_car.sh
./launch_qnx_autonomous_car.sh
```

### 4. Run Automated CDP Benchmarks
Execute headless automated test routes and receive JSON telemetry reports:
```bash
# Run 25-second 5th Avenue corridor benchmark
python3 run_benchmark.py benchmark_results.json 25.0 0.0

# Run 16-second red-light approach benchmark
python3 run_benchmark.py red_light_results.json 16.0 52.0
```

---

## 📂 Repository Structure

```
neutrino-adas/
├── Makefile                        # Top-level recursive QNX makefile
├── common.mk                       # QNX SDP compiler flags and library paths
├── include/                        # Modular C++ ADAS & IPC Headers
│   ├── adas/
│   │   ├── Types.hpp               # VehicleState, Obstacle, DriveMode & AdasOutput
│   │   ├── IPerceptionSensor.hpp   # Abstract sensor interfaces (IForwardScanner, IBlindSpotScanner)
│   │   ├── ForwardRadar.hpp        # Forward corridor detection & TTC calculation
│   │   ├── BlindSpotMonitor.hpp    # Dual side-mirror blind-spot radar zones
│   │   ├── LongitudinalController.hpp # AEB emergency stop, ACC & kinematic stopping
│   │   ├── LateralController.hpp   # Stanley lateral tracking & lane change state machine
│   │   ├── SignalActuatorManager.hpp # 3 Hz turn signals, hazards, horn & brake lights
│   │   └── QnxAdasController.hpp   # Central safety kernel composition orchestrator
│   ├── ipc/
│   │   ├── TelemetryCodec.hpp      # Zero-overhead newline-delimited JSON parser/generator
│   │   └── QnxTcpServer.hpp        # RAII POSIX TCP socket server on port 9090
│   └── sim/
│       └── ConsoleSelfTest.hpp     # Standalone terminal verification test harness
├── src/                            # Modular C++ ADAS Implementations
│   ├── main.cpp                    # CLI entrypoint (--server / -s / --port / terminal test)
│   ├── Types.cpp                   # Mode stringification and utility methods
│   ├── ForwardRadar.cpp            # Forward LiDAR/radar scanning implementation
│   ├── BlindSpotMonitor.cpp        # Dual side-mirror radar scanning implementation
│   ├── LongitudinalController.cpp  # AEB / ACC / Cruising control laws
│   ├── LateralController.cpp       # Nonlinear Stanley tracking & BSM safety interlock
│   ├── SignalActuatorManager.cpp   # Actuator clocks, hazard overrides & timers
│   ├── QnxAdasController.cpp       # Real-time L3 ADAS safety kernel orchestrator
│   ├── TelemetryCodec.cpp          # High-performance wire protocol serialization
│   ├── QnxTcpServer.cpp            # Multi-client TCP event loop & session handler
│   └── ConsoleSelfTest.cpp         # 120-step verification scenario with cut-in hazard
├── helloworld.use                  # QNX binary metadata & CLI documentation
├── desktop_app.rs                  # Native Rust desktop host (HTTP 8088 / WS 8089)
├── Cargo.toml                      # Rust dependencies & compilation profile
├── launch_qnx_autonomous_car.sh    # Unified launcher script
├── run_benchmark.py                # Automated Chrome DevTools Protocol test runner
├── baseline_metrics.json           # Recorded baseline benchmark data (pre-optimization)
├── test_fix1_metrics.json          # Post-Fix 1 benchmark data (Stanley controller)
├── test_fix2_red_approach.json     # Post-Fix 2 benchmark data (Red-light stopping)
├── test_fix2_full_route.json       # Full corridor regression verification dataset
├── nto/                            # QNX architecture target directories (x86_64 / aarch64)
└── slop-newyork/                   # WebGPU 3D Manhattan digital twin & simulation
    ├── index.html                  # Simulation viewport & glassmorphism HUD overlay
    ├── js/
    │   ├── autonomous-driver.js    # Telemetry packing, radar scan & mirror rendering
    │   ├── driving-metrics.js      # 60 FPS 5-metric instrumentation engine
    │   ├── main.js                 # Three.js animation loop & system coordinator
    │   ├── city-config.js          # Manhattan road network geometry & signal timing
    │   ├── road-network.js         # Street graph, lanes, crosswalks & spatial index
    │   ├── traffic.js              # Ambient dynamic traffic simulation
    │   └── controllers.js          # Vehicle dynamic physics & keyboard inputs
    └── css/                        # Responsive styles & local typography
```

---

## 🤝 Acknowledgements & Credits

This project builds upon phenomenal open-source engineering:

* **[Mr.doob (Ricardo Cabello)](https://github.com/mrdoob)**: Creator of [Three.js](https://threejs.org/) and author of the breathtaking **[Slop New York](https://github.com/mrdoob/slop-newyork)** procedural WebGPU simulation that serves as our 3D digital twin.
* **[BlackBerry QNX](https://blackberry.qnx.com)**: For the rock-solid, POSIX-compliant **QNX Neutrino RTOS 8.0** microkernel powering automotive safety worldwide.
* **The Rust Community**: For asynchronous networking primitives enabling microsecond-level IPC bridge performance.

---

## 📄 License

This repository is licensed under the **MIT License**. See [LICENSE](LICENSE) for full details.  
Third-party simulation assets and 3D models retain their respective licenses under `slop-newyork/`.
