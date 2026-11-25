//! Entrypoint for the Native 60 FPS Bevy 3D + MetaDrive glTF + Waymo ScenarioNet + QNX 8.0 ADAS Simulator.
//! Parses CLI arguments cleanly BEFORE launching the Bevy window.

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use qnx_rust_sim::config::SimConfig;
use qnx_rust_sim::render_setup::{RenderSetupPlugin, SimWorldRes};
use qnx_rust_sim::scene_3d::Scene3dPlugin;
use qnx_rust_sim::simulation::SimWorld;
use qnx_rust_sim::ui_hud::UiHudPlugin;
use std::process::ExitCode;

fn main() -> ExitCode {
    let raw_args: Vec<String> = std::env::args().skip(1).collect();
    let cfg = match SimConfig::from_args(&raw_args) {
        Ok(Some(c)) => c,
        Ok(None) => {
            println!("{}", SimConfig::help_text());
            return ExitCode::SUCCESS;
        }
        Err(err) => {
            eprintln!("Argument error: {}", err);
            return ExitCode::FAILURE;
        }
    };

    let title = format!(
        "QNX 8.0 ADAS [{} | {} | {}]",
        cfg.map_mode.label(),
        cfg.view.label(),
        cfg.profile.label()
    );
    let world = SimWorld::new(cfg, false);

    App::new()
        .add_plugins(RenderSetupPlugin { title })
        .add_plugins(EguiPlugin)
        .add_plugins((Scene3dPlugin, UiHudPlugin))
        .insert_resource(SimWorldRes(world))
        .run();

    ExitCode::SUCCESS
}
