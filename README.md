# QNX 8.0 Neutrino RTOS <-> Native Bevy 3D Autonomous Driving Simulator

A native 60 FPS **Bevy `0.15` + `bevy_egui`** 3D autonomous driving simulator wired directly to a **BlackBerry QNX Neutrino 8.0 C++ Safety Kernel** over TCP (`192.168.122.62:9090`), rendering **real extracted MetaDrive `.gltf` 3D vehicle/pedestrian/cyclist/signal models** and **offline-exported ScenarioNet Waymo HD maps & tracks** (`--map waymo`) alongside our continuous **~10+ km procedural multi-lane city/highway network** (`--map mega10km`), tuned for **Intel Core i3 (6th gen) + Intel HD integrated graphics**.

---

## System Architecture

```text
+-----------------------------------------------------------------------------------+
|                        QNX 8.0 NEUTRINO RTOS VM (192.168.122.62:9090)             |
|  - Ackermann Feedforward + Stanley + Damping Lateral Ctrl (LateralController.cpp) |
|  - Quintic 3.2s S-Curve Lane Changes + 1.5s Blinker & BSM Guard                   |
|  - IDM + Monotonic Stop-Line + Point B + AEB Control (LongitudinalController.cpp) |
|  - Behavior Planner & CHILL/STANDARD/HURRY Profiles (BehaviorPlanner.cpp)         |
|  - Curved-Corridor Radar & Crossing-Pedestrian Filter (ForwardRadar.cpp)          |
|  - Expanded BSM Zone [-12m, +4m] (BlindSpotMonitor.cpp)                           |
|  - Real-Time SCHED_FIFO POSIX TCP Server (QnxTcpServer.cpp)                       |
+-----------------------------------------+-----------------------------------------+
                                          | TCP_NODELAY (JSON line protocol)
+-----------------------------------------v-----------------------------------------+
|                   NATIVE BEVY 3D + BEVY_EGUI SIMULATOR (adas-3d-sim/)             |
|  - SimConfig (config.rs): --map mega10km|waymo, --view 3d|2d, --profile, --wet    |
|  - MegaMapGenerator / ScenarioLoader: ~10+ km loops & Waymo HD JSON maps          |
|  - RenderSetupPlugin (render_setup.rs): Msaa::Off, no shadows, 220m linear fog    |
|  - Scene3dPlugin (scene_3d.rs): Real glTF vehicles/peds/bikes/cones/lights + 180m |
|    chunk culling, textured asphalt/grass/curbs, live FSD ribbon & 3D/2D camera    |
|  - UiHudPlugin (ui_hud.rs): Minimalist egui telemetry HUD, minimap & [M] A->B map |
|  - QnxTcpClient / TelemetryRecorder: p50/p95 RTT, 450 ms watchdog, JSON/CSV export|
+-----------------------------------------------------------------------------------+
```

---

## Build & Usage

### 1. Build the QNX 8.0 C++ Safety Kernel (`qnx-adas-os/`)
```bash
source /home/aauel/qnx800/qnxsdp-env.sh
cd qnx-adas-os && make clean && make
```
Produces `qnx-adas-os/nto/x86_64/o/helloworld` and `qnx-adas-os/nto/aarch64/o-le/helloworld` with `0` compiler warnings.

### 2. Launch the Bevy 3D Simulator (`adas-3d-sim/`)
```bash
# Default: 3D Chase-Cam on continuous ~10+ km road network (press [V] to toggle 3D <-> 2D)
./launch_qnx_autonomous_car.sh --view 3d --map mega10km

# Real exported ScenarioNet Waymo urban HD maps, dynamic signals & tracks
./launch_qnx_autonomous_car.sh --map waymo --view 3d

# Start directly in Top-Down 2D Tesla FSD view
./launch_qnx_autonomous_car.sh --view 2d
```

### Offline ScenarioNet Importer (`adas-3d-sim/tools/export_scenarionet.py`)
Any Waymo, nuScenes, or nuPlan ScenarioNet `.pkl` dataset can be converted once offline into self-contained JSON files under `adas-3d-sim/assets/scenarios/`:
```bash
cd adas-3d-sim
python3 tools/export_scenarionet.py --input-dir /path/to/scenarionet_pkls --output-dir assets/scenarios
```

### CLI Flags
| Flag | Values / Default | Description |
| :--- | :--- | :--- |
| `--map` | `mega10km` \| `waymo` (`mega10km`) | Procedural ~10+ km map or exported ScenarioNet Waymo HD scenes |
| `--view` | `3d` \| `2d` (`3d`) | Initial camera view (`[V]` toggles live between 3D and 2D) |
| `--seed` | integer (`1000`) | Deterministic road network or starting scenario index |
| `--traffic-density` | `0.0` – `0.5` (`0.20`) | Ambient IDM+MOBIL traffic density (`mega10km` mode) |
| `--qnx-host` | string (`192.168.122.62`) | QNX 8.0 target IP address |
| `--qnx-port` | integer (`9090`) | QNX 8.0 TCP server port |
| `--profile` | `chill` \| `standard` \| `hurry` (`standard`) | QNX FSD driving profile |
| `--noise` | flag | Enable seeded Gaussian radar noise & occlusion |
| `--wet` | flag | Start on wet low-friction asphalt (`mu=0.55` vs `0.95`) |
| `--scenarios-dir` | path (`assets/scenarios`) | Directory of exported ScenarioNet JSON scenes |
| `--report-prefix` | string (`qnx_rust_scorecard`) | Prefix for exported `.json` and `.csv` scorecards |

### Interactive Hotkeys (`3d` & `2d`)
- **`V`**: Toggle live between **3D Chase-Cam** and **2D FSD Top-Down**
- **`M`**: Open / Close **Interactive Full-Screen Navigation Map** (click any road to set **Point B**, scroll zoom, drag pan, double-click / `ENTER` / **Drive Here** to route Point A -> B; also opens by clicking the sidebar minimap)
- **`C`**: Clear Point B arrival hold & resume continuous Free Cruise
- **`G`**: Generate next ~10+ km map seed (or advance to next Waymo clip)
- **`P`**: Cycle driving profile (`CHILL` -> `STANDARD` -> `HURRY`)
- **`T`**: Spawn slow/braking traffic jam + roadside safety cones ahead
- **`J`**: Spawn crossing jaywalker ahead
- **`W`**: Toggle road surface (`DRY mu=0.95` <-> `WET mu=0.55`)
- **`N`**: Toggle seeded Gaussian radar noise & occlusion
- **`SPACE`**: Pause / Resume simulation
- **`R`**: Export JSON & CSV telemetry scorecard immediately
- **`ESC` / `Q`**: Close Interactive Map if open (`ESC`), or export scorecard and exit cleanly

---

## Verification Suite

Build release binary and run asset, ScenarioNet Waymo, A* routing, latency, and live QNX closed-loop pro-driver accuracy tests:
```bash
cd adas-3d-sim
cargo build --release && cargo test --release -- --nocapture
```

---

## Repository Structure

```text
helloworld/
├── qnx-adas-os/                        # QNX 8.0 Neutrino C++ ADAS Safety Kernel
│   ├── Makefile                        # Recursive QNX 8.0 build entry point
│   ├── common.mk                       # QNX SDP compiler & socket linker configuration
│   ├── helloworld.use                  # QNX binary usage metadata
│   ├── include/                        # QNX C++ Safety Kernel headers (adas/, ipc/, sim/)
│   ├── src/                            # QNX C++ Safety Kernel implementation
│   └── nto/                            # Compiled QNX binaries (x86_64 & aarch64)
├── adas-3d-sim/                        # Native Bevy 3D Autonomous Driving Simulator
│   ├── Cargo.toml                      # Native Bevy 3D simulator crate configuration
│   ├── Cargo.lock                      # Locked Rust dependency graph
│   ├── assets/
│   │   ├── models/                     # Extracted MetaDrive glTF models (ferra, 130, lada, truck, pedestrian, bicycle, cone, lights)
│   │   ├── textures/                   # Asphalt, grass, and sidewalk surface textures
│   │   └── scenarios/                  # Self-contained exported Waymo ScenarioNet JSON scenes
│   ├── tools/
│   │   └── export_scenarionet.py       # Offline ScenarioNet (.pkl) -> JSON map/track/light exporter
│   └── rust_sim/
│       ├── src/
│       │   ├── lib.rs                  # Crate module exports
│       │   ├── main.rs                 # Bevy App entrypoint & CLI parsing
│       │   ├── config.rs               # CLI parser (--map, --view, --profile, --wet)
│       │   ├── coordinate_mapper.rs    # Frame projections, BSM zones & RNG
│       │   ├── road_network.rs         # ~10+ km MegaMapGenerator, Waymo HD builder, Frenet & A*
│       │   ├── scenario_loader.rs      # ScenarioNet JSON loader & track interpolator
│       │   ├── physics.rs              # Dynamic bicycle model (80 ms lag, DRY/WET)
│       │   ├── traffic.rs              # Signals, IDM+MOBIL traffic, Waymo replay & cones
│       │   ├── perception.rs           # 85 m obstacle/VRU/stop-line scanner
│       │   ├── qnx_client.rs           # TCP_NODELAY client, p50/p95 RTT & watchdog
│       │   ├── recorder.rs             # Telemetry stats & JSON/CSV export
│       │   ├── simulation.rs           # Headless-testable closed-loop SimWorld
│       │   ├── map_overlay.rs          # Interactive map viewport projection & A* preview math
│       │   ├── render_setup.rs         # Bevy Core i3/Intel HD setup (Msaa::Off, no shadows, fog)
│       │   ├── scene_3d.rs             # Real glTF actors, chunked 3D roads, FSD ribbon & cams
│       │   └── ui_hud.rs               # bevy_egui telemetry HUD, minimap & [M] map UI
│       └── tests/
│           └── integration_tests.rs    # Assets, Waymo, MegaMap, perf & live QNX tests
├── launch_qnx_autonomous_car.sh        # QNX VM + Bevy 3D unified launcher
├── .gitignore
├── LICENSE
└── README.md
```
