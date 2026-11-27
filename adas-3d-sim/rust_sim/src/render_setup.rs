//! Bevy 0.15 engine configuration tuned for locked 60 FPS on Intel Core i3 + Intel HD graphics.

use crate::simulation::SimWorld;
use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy::prelude::*;
use bevy::window::{PresentMode, WindowResolution};
use std::path::PathBuf;

/// Resolve the repository `assets/` directory cleanly whether invoked from workspace root,
/// `cargo run`, or directly as `target/release/qnx_rust_sim` from another directory.
pub fn resolve_assets_dir() -> String {
    if let Ok(root) = std::env::var("BEVY_ASSET_ROOT") {
        let cand = PathBuf::from(root).join("assets");
        if cand.join("models").is_dir() {
            return cand
                .canonicalize()
                .unwrap_or(cand)
                .to_string_lossy()
                .into_owned();
        }
    }
    if let Ok(cwd) = std::env::current_dir() {
        let cand = cwd.join("assets");
        if cand.join("models").is_dir() {
            return cand
                .canonicalize()
                .unwrap_or(cand)
                .to_string_lossy()
                .into_owned();
        }
    }
    if let Ok(exe) = std::env::current_exe() {
        for anc in exe.ancestors().skip(1) {
            let cand = anc.join("assets");
            if cand.join("models").is_dir() {
                return cand
                    .canonicalize()
                    .unwrap_or(cand)
                    .to_string_lossy()
                    .into_owned();
            }
        }
    }
    "assets".to_string()
}

#[inline]
pub fn to_bevy_pos(wx: f32, y: f32, wz: f32) -> Vec3 {
    // QNX (+X East, +Z North) -> Bevy (+X East, +Y Up, -Z North)
    Vec3::new(wx, y, -wz)
}

#[derive(Resource)]
pub struct SimWorldRes(pub SimWorld);

pub struct RenderSetupPlugin {
    pub title: String,
}

impl Plugin for RenderSetupPlugin {
    fn build(&self, app: &mut App) {
        let title = self.title.clone();
        app.add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title,
                        resolution: WindowResolution::new(1360.0, 820.0),
                        present_mode: PresentMode::AutoVsync,
                        resizable: true,
                        ..default()
                    }),
                    ..default()
                })
                .set(AssetPlugin {
                    file_path: resolve_assets_dir(),
                    ..default()
                }),
        )
        .add_plugins(FrameTimeDiagnosticsPlugin)
        // Bright clear sky + zero-shadow daylight ambient for fast Intel HD shading
        .insert_resource(ClearColor(Color::srgb(0.56, 0.78, 0.96)))
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 620.0,
        })
        .add_systems(Startup, setup_lighting_and_camera);
    }
}

#[derive(Component)]
pub struct MainDriveCamera;

fn setup_lighting_and_camera(mut commands: Commands) {
    // Single directional sunlight with shadows disabled for crisp 60 FPS on Intel HD integrated GPUs
    commands.spawn((
        DirectionalLight {
            color: Color::srgb(1.0, 0.98, 0.93),
            illuminance: 10_500.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(90.0, 160.0, 70.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Main camera with MSAA off & 220 m linear distance fog
    commands.spawn((
        MainDriveCamera,
        Camera3d::default(),
        Msaa::Off,
        Projection::Perspective(PerspectiveProjection {
            fov: 52.0f32.to_radians(),
            near: 0.35,
            far: 450.0,
            ..default()
        }),
        DistanceFog {
            color: Color::srgba(0.62, 0.80, 0.95, 1.0),
            falloff: FogFalloff::Linear {
                start: 155.0,
                end: 222.0,
            },
            ..default()
        },
        Transform::from_xyz(0.0, 6.0, 14.0).looking_at(Vec3::new(0.0, 1.2, -18.0), Vec3::Y),
    ));
}
