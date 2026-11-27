//! Native 60 FPS Bevy 3D autonomous simulator wired to QNX 8.0 C++ Safety Kernel (port 9090).

pub mod config;
pub mod coordinate_mapper;
pub mod map_overlay;
pub mod perception;
pub mod physics;
pub mod qnx_client;
pub mod recorder;
pub mod render_setup;
pub mod road_network;
pub mod scenario_loader;
pub mod scene_3d;
pub mod simulation;
pub mod traffic;
pub mod ui_hud;
