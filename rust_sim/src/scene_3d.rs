//! Bevy 3D world builder & dynamic actor synchronizer using real MetaDrive `.gltf`
//! vehicle/pedestrian/cyclist/cone/traffic-light models, chunked 220 m visibility-culled
//! roads (for both `mega10km` and `waymo` HD maps), live FSD ribbons, and 3D/2D camera switching.

use crate::config::{ViewMode, LANE_WIDTH_M};
use crate::coordinate_mapper::{forward_vec, normalize_angle_rad, right_vec};
use crate::render_setup::{to_bevy_pos, MainDriveCamera, SimWorldRes};
use crate::road_network::{JunctionType, RoadFeatureKind, RoadNetwork};
use crate::traffic::{LightColor, VehicleClass};
use bevy::image::{ImageAddressMode, ImageSampler, ImageSamplerDescriptor};
use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};
use bevy::render::render_asset::RenderAssetUsages;
use std::collections::{HashMap, HashSet};
use std::f32::consts::{FRAC_PI_2, PI};

const CHUNK_SIZE_M: f32 = 180.0;
const VISIBILITY_RADIUS_M: f32 = 228.0;

#[derive(Clone, Copy)]
pub struct VehicleGltfSpec {
    chassis_offset: Vec3,
    chassis_scale: f32,
    front_wheel_z: f32, // glTF -Z = forward
    rear_wheel_z: f32,  // glTF +Z = rear
    lateral_x: f32,
    tire_radius: f32,
}

#[derive(Resource)]
pub struct LoadedAssets {
    ferra_chassis: Handle<Scene>,
    ferra_tire_f: Handle<Scene>,
    ferra_tire_b: Handle<Scene>,
    sedan_chassis: Handle<Scene>,
    sedan_tire_f: Handle<Scene>,
    sedan_tire_b: Handle<Scene>,
    suv_chassis: Handle<Scene>,
    suv_tire_f: Handle<Scene>,
    suv_tire_b: Handle<Scene>,
    truck_chassis: Handle<Scene>,
    truck_tire_f: Handle<Scene>,
    truck_tire_b: Handle<Scene>,
    pedestrian_scene: Handle<Scene>,
    bicycle_scene: Handle<Scene>,
    cone_scene: Handle<Scene>,
    light_red_scene: Handle<Scene>,
    light_yellow_scene: Handle<Scene>,
    light_green_scene: Handle<Scene>,

    asphalt_tex: Handle<Image>,
    grass_tex: Handle<Image>,
    sidewalk_tex: Handle<Image>,
    sampler_configured: [bool; 3],

    mat_asphalt: Handle<StandardMaterial>,
    mat_sidewalk: Handle<StandardMaterial>,
    mat_yellow_line: Handle<StandardMaterial>,
    mat_white_line: Handle<StandardMaterial>,
    mat_tree_trunk: Handle<StandardMaterial>,
    mat_tree_leaf: Handle<StandardMaterial>,
    mat_pole: Handle<StandardMaterial>,
    mat_ribbon: Handle<StandardMaterial>,
    pole_mesh: Handle<Mesh>,
    arm_mesh: Handle<Mesh>,
}

#[derive(Component)]
pub struct RoadChunk {
    pub center_x: f32,
    pub center_z: f32,
    pub generation: u64,
}

#[derive(Component)]
pub struct GroundFollower;

#[derive(Component)]
pub struct EgoRoot;

#[derive(Component)]
pub struct EgoWheel {
    pub is_front: bool,
    pub is_left: bool,
    pub radius: f32,
}

#[derive(Component)]
pub struct EgoBrakeLight;

#[derive(Component)]
pub struct EgoTurnSignal {
    pub is_left: bool,
}

#[derive(Component)]
pub struct EgoOverlay2D;

#[derive(Component)]
pub struct EgoBsmArc {
    pub is_left: bool,
}

#[derive(Component)]
pub struct FsdRibbonEntity;

#[derive(Component)]
pub struct StopBarEntity;

#[derive(Component)]
pub struct GoalBeaconEntity;

#[derive(Resource, Default)]
pub struct DynamicPools {
    pub loaded_map_generation: u64,
    pub wheel_spin_rad: f32,
    pub npcs: HashMap<usize, (Entity, VehicleClass)>,
    pub peds: HashMap<usize, Entity>,
    pub cyclists: HashMap<usize, Entity>,
    pub cones: Vec<Entity>,
    pub signals: Vec<(Entity, LightColor)>,
    pub ribbon_mesh: Option<Handle<Mesh>>,
    pub cam_initialized: bool,
    pub cam_pos: Vec3,
    pub cam_target: Vec3,
}

pub struct Scene3dPlugin;

impl Plugin for Scene3dPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DynamicPools>()
            .add_systems(Startup, setup_assets_and_ego)
            .add_systems(
                Update,
                (
                    configure_repeat_textures,
                    rebuild_static_world_on_map_change,
                    sync_ego_and_fsd_ribbon,
                    sync_npcs_peds_cyclists,
                    sync_cones_and_signals,
                    update_chunk_visibility_and_ground,
                    update_camera_system,
                )
                    .chain(),
            );
    }
}

fn spec_for_class(class: Option<VehicleClass>) -> VehicleGltfSpec {
    match class {
        None => VehicleGltfSpec {
            // DefaultVehicle (`ferra`)
            chassis_offset: Vec3::new(0.0, 0.02, -0.075),
            chassis_scale: 1.0,
            front_wheel_z: -1.052,
            rear_wheel_z: 1.417,
            lateral_x: 0.815,
            tire_radius: 0.313,
        },
        Some(VehicleClass::Sedan) => VehicleGltfSpec {
            // MVehicle (`130`)
            chassis_offset: Vec3::new(0.0, 0.06, 0.05),
            chassis_scale: 1.0,
            front_wheel_z: -1.285,
            rear_wheel_z: 1.203,
            lateral_x: 0.803,
            tire_radius: 0.39,
        },
        Some(VehicleClass::Suv) => VehicleGltfSpec {
            // LVehicle (`lada`)
            chassis_offset: Vec3::new(0.0, 0.06, 0.27),
            chassis_scale: 1.1,
            front_wheel_z: -1.530,
            rear_wheel_z: 1.218,
            lateral_x: 0.75,
            tire_radius: 0.429,
        },
        Some(VehicleClass::Truck) => VehicleGltfSpec {
            // XLVehicle (`truck`)
            chassis_offset: Vec3::new(0.0, 0.04, -0.25),
            chassis_scale: 1.0,
            front_wheel_z: -1.726,
            rear_wheel_z: 1.075,
            lateral_x: 0.931,
            tire_radius: 0.37,
        },
    }
}

fn setup_assets_and_ego(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut pools: ResMut<DynamicPools>,
) {
    let asphalt_tex: Handle<Image> = asset_server.load("textures/asphalt/diff_2k.png");
    let grass_tex: Handle<Image> = asset_server.load("textures/grass2/color.jpg");
    let sidewalk_tex: Handle<Image> = asset_server.load("textures/sidewalk/color.png");

    let mat_asphalt = materials.add(StandardMaterial {
        base_color: Color::srgb(0.55, 0.56, 0.59),
        base_color_texture: Some(asphalt_tex.clone()),
        perceptual_roughness: 0.92,
        metallic: 0.02,
        ..default()
    });
    let mat_grass = materials.add(StandardMaterial {
        base_color: Color::srgb(0.68, 0.78, 0.62),
        base_color_texture: Some(grass_tex.clone()),
        perceptual_roughness: 0.96,
        metallic: 0.0,
        ..default()
    });
    let mat_sidewalk = materials.add(StandardMaterial {
        base_color: Color::srgb(0.75, 0.76, 0.78),
        base_color_texture: Some(sidewalk_tex.clone()),
        perceptual_roughness: 0.90,
        ..default()
    });
    let mat_yellow_line = materials.add(StandardMaterial {
        base_color: Color::srgb(0.95, 0.74, 0.12),
        unlit: true,
        ..default()
    });
    let mat_white_line = materials.add(StandardMaterial {
        base_color: Color::srgb(0.94, 0.96, 0.98),
        unlit: true,
        ..default()
    });
    let mat_tree_trunk = materials.add(StandardMaterial {
        base_color: Color::srgb(0.36, 0.25, 0.16),
        perceptual_roughness: 0.95,
        ..default()
    });
    let mat_tree_leaf = materials.add(StandardMaterial {
        base_color: Color::srgb(0.18, 0.48, 0.22),
        perceptual_roughness: 0.90,
        ..default()
    });
    let mat_brake_on = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.15, 0.12),
        emissive: LinearRgba::rgb(4.5, 0.2, 0.1),
        unlit: true,
        ..default()
    });
    let mat_turn_on = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.68, 0.08),
        emissive: LinearRgba::rgb(3.5, 1.6, 0.05),
        unlit: true,
        ..default()
    });
    let mat_pole = materials.add(StandardMaterial {
        base_color: Color::srgb(0.22, 0.25, 0.30),
        metallic: 0.6,
        perceptual_roughness: 0.5,
        ..default()
    });
    let mat_ribbon = materials.add(StandardMaterial {
        base_color: Color::srgba(0.18, 0.72, 1.0, 0.85),
        emissive: LinearRgba::rgb(0.15, 0.75, 1.4),
        alpha_mode: AlphaMode::Blend,
        unlit: true,
        ..default()
    });
    let mat_stop_bar = materials.add(StandardMaterial {
        base_color: Color::srgba(1.0, 0.18, 0.15, 0.92),
        emissive: LinearRgba::rgb(2.6, 0.15, 0.1),
        alpha_mode: AlphaMode::Blend,
        unlit: true,
        ..default()
    });
    let mat_goal_beacon = materials.add(StandardMaterial {
        base_color: Color::srgba(0.10, 0.92, 0.55, 0.82),
        emissive: LinearRgba::rgb(0.15, 1.8, 0.8),
        alpha_mode: AlphaMode::Blend,
        unlit: true,
        ..default()
    });
    let mat_radar_cone = materials.add(StandardMaterial {
        base_color: Color::srgba(0.22, 0.74, 0.98, 0.16),
        alpha_mode: AlphaMode::Blend,
        unlit: true,
        ..default()
    });
    let mat_bsm_arc = materials.add(StandardMaterial {
        base_color: Color::srgba(1.0, 0.25, 0.18, 0.85),
        emissive: LinearRgba::rgb(2.5, 0.3, 0.15),
        alpha_mode: AlphaMode::Blend,
        unlit: true,
        ..default()
    });

    let loaded = LoadedAssets {
        ferra_chassis: asset_server.load("models/ferra/vehicle.gltf#Scene0"),
        ferra_tire_f: asset_server.load("models/ferra/right_tire_front.gltf#Scene0"),
        ferra_tire_b: asset_server.load("models/ferra/right_tire_back.gltf#Scene0"),
        sedan_chassis: asset_server.load("models/130/vehicle.gltf#Scene0"),
        sedan_tire_f: asset_server.load("models/130/right_tire_front.gltf#Scene0"),
        sedan_tire_b: asset_server.load("models/130/right_tire_back.gltf#Scene0"),
        suv_chassis: asset_server.load("models/lada/vehicle.gltf#Scene0"),
        suv_tire_f: asset_server.load("models/lada/right_tire_front.gltf#Scene0"),
        suv_tire_b: asset_server.load("models/lada/right_tire_back.gltf#Scene0"),
        truck_chassis: asset_server.load("models/truck/vehicle.gltf#Scene0"),
        truck_tire_f: asset_server.load("models/truck/right_tire_front.gltf#Scene0"),
        truck_tire_b: asset_server.load("models/truck/right_tire_back.gltf#Scene0"),
        pedestrian_scene: asset_server.load("models/pedestrian/scene.gltf#Scene0"),
        bicycle_scene: asset_server.load("models/bicycle/scene.gltf#Scene0"),
        cone_scene: asset_server.load("models/traffic_cone/scene.gltf#Scene0"),
        light_red_scene: asset_server.load("models/traffic_light/red.gltf#Scene0"),
        light_yellow_scene: asset_server.load("models/traffic_light/yellow.gltf#Scene0"),
        light_green_scene: asset_server.load("models/traffic_light/green.gltf#Scene0"),
        asphalt_tex,
        grass_tex,
        sidewalk_tex,
        sampler_configured: [false; 3],
        mat_asphalt: mat_asphalt.clone(),
        mat_sidewalk,
        mat_yellow_line,
        mat_white_line,
        mat_tree_trunk,
        mat_tree_leaf,
        mat_pole,
        mat_ribbon: mat_ribbon.clone(),
        pole_mesh: meshes.add(Cylinder::new(0.14, 5.2)),
        arm_mesh: meshes.add(Cuboid::new(4.2, 0.14, 0.14)),
    };

    // Moving textured ground plane snapped smoothly near ego every 60 m
    let ground_mesh = build_tiled_plane_mesh(620.0, 620.0, 48.0);
    commands.spawn((
        GroundFollower,
        Mesh3d(meshes.add(ground_mesh)),
        MeshMaterial3d(mat_grass),
        Transform::from_xyz(0.0, -0.04, 0.0),
    ));

    // Dynamic FSD Trajectory Ribbon entity
    let rib_handle = meshes.add(Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    ));
    pools.ribbon_mesh = Some(rib_handle.clone());
    commands.spawn((
        FsdRibbonEntity,
        Mesh3d(rib_handle),
        MeshMaterial3d(mat_ribbon),
        Transform::IDENTITY,
    ));

    // Glowing Red Stop-Line Barrier Bar on asphalt
    commands.spawn((
        StopBarEntity,
        Mesh3d(meshes.add(Cuboid::new(4.8, 0.08, 0.45))),
        MeshMaterial3d(mat_stop_bar),
        Transform::IDENTITY,
        Visibility::Hidden,
    ));

    // Point B Glowing Destination Beacon
    commands.spawn((
        GoalBeaconEntity,
        Mesh3d(meshes.add(Cylinder::new(1.3, 6.5))),
        MeshMaterial3d(mat_goal_beacon),
        Transform::IDENTITY,
        Visibility::Hidden,
    ));

    // Spawn Ego Real MetaDrive `ferra` vehicle + 4 rotating tires + brake/signal lamps + 2D FSD overlays
    let spec = spec_for_class(None);
    let lamp_sphere = meshes.add(Sphere::new(0.11));
    let radar_mesh = meshes.add(build_radar_wedge_mesh(52.0, 25.0f32.to_radians()));
    let bsm_mesh = meshes.add(Cuboid::new(0.28, 0.14, 4.2));

    commands
        .spawn((EgoRoot, Transform::IDENTITY, Visibility::Visible))
        .with_children(|parent| {
            parent.spawn((
                SceneRoot(loaded.ferra_chassis.clone()),
                Transform::from_translation(spec.chassis_offset)
                    .with_scale(Vec3::splat(spec.chassis_scale)),
            ));
            for &(is_front, is_left, x, z, ref tire_h) in &[
                (true, false, spec.lateral_x, spec.front_wheel_z, loaded.ferra_tire_f.clone()),
                (true, true, -spec.lateral_x, spec.front_wheel_z, loaded.ferra_tire_f.clone()),
                (false, false, spec.lateral_x, spec.rear_wheel_z, loaded.ferra_tire_b.clone()),
                (false, true, -spec.lateral_x, spec.rear_wheel_z, loaded.ferra_tire_b.clone()),
            ] {
                let mut wheel_parent = parent.spawn((
                    EgoWheel {
                        is_front,
                        is_left,
                        radius: spec.tire_radius,
                    },
                    Transform::from_xyz(x, spec.tire_radius, z),
                ));
                wheel_parent.with_children(|wp| {
                    // Rotate left-side tire 180 deg around Y instead of negative scale to keep outward normals valid
                    let rot = if is_left {
                        Quat::from_rotation_y(PI)
                    } else {
                        Quat::IDENTITY
                    };
                    wp.spawn((
                        SceneRoot(tire_h.clone()),
                        Transform::from_rotation(rot),
                    ));
                });
            }

            // 3D Brake Lights (rear Z = +2.24)
            for bx in [-0.56f32, 0.56] {
                parent.spawn((
                    EgoBrakeLight,
                    Mesh3d(lamp_sphere.clone()),
                    MeshMaterial3d(mat_brake_on.clone()),
                    Transform::from_xyz(bx, 0.62, 2.24),
                    Visibility::Hidden,
                ));
            }
            // Blinking Left/Right Turn Signals (front & rear corners)
            for &(is_left, sx) in &[(true, -0.76f32), (false, 0.76)] {
                for sz in [-2.05f32, 2.18] {
                    parent.spawn((
                        EgoTurnSignal { is_left },
                        Mesh3d(lamp_sphere.clone()),
                        MeshMaterial3d(mat_turn_on.clone()),
                        Transform::from_xyz(sx, 0.56, sz),
                        Visibility::Hidden,
                    ));
                }
            }
            // Forward Radar FOV Wedge (visible in 2D top-down view)
            parent.spawn((
                EgoOverlay2D,
                Mesh3d(radar_mesh),
                MeshMaterial3d(mat_radar_cone),
                Transform::from_xyz(0.0, 0.06, -2.0),
                Visibility::Hidden,
            ));
            // Left & Right BSM Side Warning Strips
            for &(is_left, ax) in &[(true, -1.75f32), (false, 1.75)] {
                parent.spawn((
                    EgoBsmArc { is_left },
                    Mesh3d(bsm_mesh.clone()),
                    MeshMaterial3d(mat_bsm_arc.clone()),
                    Transform::from_xyz(ax, 0.25, 0.2),
                    Visibility::Hidden,
                ));
            }
        });

    commands.insert_resource(loaded);
}

fn configure_repeat_textures(
    mut loaded: ResMut<LoadedAssets>,
    mut images: ResMut<Assets<Image>>,
) {
    let handles = [
        loaded.asphalt_tex.clone(),
        loaded.grass_tex.clone(),
        loaded.sidewalk_tex.clone(),
    ];
    for (idx, h) in handles.iter().enumerate() {
        if loaded.sampler_configured[idx] {
            continue;
        }
        if let Some(img) = images.get_mut(h) {
            img.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
                address_mode_u: ImageAddressMode::Repeat,
                address_mode_v: ImageAddressMode::Repeat,
                ..ImageSamplerDescriptor::linear()
            });
            loaded.sampler_configured[idx] = true;
        }
    }
}

struct ChunkBuffers {
    asphalt: MeshAccum,
    yellow: MeshAccum,
    white: MeshAccum,
    sidewalk: MeshAccum,
    trunks: MeshAccum,
    leaves: MeshAccum,
}

impl ChunkBuffers {
    fn new() -> Self {
        Self {
            asphalt: MeshAccum::default(),
            yellow: MeshAccum::default(),
            white: MeshAccum::default(),
            sidewalk: MeshAccum::default(),
            trunks: MeshAccum::default(),
            leaves: MeshAccum::default(),
        }
    }
}

#[derive(Default)]
struct MeshAccum {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    uvs: Vec<[f32; 2]>,
    indices: Vec<u32>,
}

impl MeshAccum {
    fn is_empty(&self) -> bool {
        self.indices.is_empty()
    }

    fn push_quad_up(&mut self, p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3, u_span: f32, v_span: f32) {
        let base = self.positions.len() as u32;
        self.positions.extend_from_slice(&[
            p0.to_array(),
            p1.to_array(),
            p2.to_array(),
            p3.to_array(),
        ]);
        let n = [0.0, 1.0, 0.0];
        self.normals.extend_from_slice(&[n, n, n, n]);
        self.uvs
            .extend_from_slice(&[[0.0, 0.0], [u_span, 0.0], [u_span, v_span], [0.0, v_span]]);
        // CCW winding when viewed from +Y
        self.indices
            .extend_from_slice(&[base, base + 2, base + 1, base, base + 3, base + 2]);
    }

    fn push_tree(&mut self, x: f32, z: f32, scale: f32, is_trunk: bool) {
        if is_trunk {
            let r = 0.22 * scale;
            let h = 2.1 * scale;
            let b = self.positions.len() as u32;
            for side in 0..4usize {
                let a = (side as f32) * FRAC_PI_2;
                let a2 = ((side + 1) as f32) * FRAC_PI_2;
                let c0 = [x + a.cos() * r, 0.0, z + a.sin() * r];
                let c1 = [x + a2.cos() * r, 0.0, z + a2.sin() * r];
                let t0 = [x + a.cos() * r * 0.7, h, z + a.sin() * r * 0.7];
                let t1 = [x + a2.cos() * r * 0.7, h, z + a2.sin() * r * 0.7];
                let n = [a.cos(), 0.2, a.sin()];
                self.positions.extend_from_slice(&[c0, c1, t1, t0]);
                self.normals.extend_from_slice(&[n, n, n, n]);
                self.uvs.extend_from_slice(&[[0.0; 2]; 4]);
                let ib = b + (side as u32) * 4;
                self.indices
                    .extend_from_slice(&[ib, ib + 1, ib + 2, ib, ib + 2, ib + 3]);
            }
        } else {
            // Simple 4-sided pyramid crown at y = 1.8..5.6
            let y0 = 1.7 * scale;
            let y1 = 5.4 * scale;
            let r = 1.75 * scale;
            let b = self.positions.len() as u32;
            let pts = [
                [x - r, y0, z - r],
                [x + r, y0, z - r],
                [x + r, y0, z + r],
                [x - r, y0, z + r],
                [x, y1, z],
            ];
            let n = [0.0, 0.8, 0.6];
            for p in pts {
                self.positions.push(p);
                self.normals.push(n);
                self.uvs.push([0.0, 0.0]);
            }
            for s in 0..4u32 {
                let sn = (s + 1) % 4;
                self.indices.extend_from_slice(&[b + s, b + sn, b + 4]);
            }
        }
    }

    fn build_mesh(self) -> Mesh {
        let mut m = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        );
        m.insert_attribute(Mesh::ATTRIBUTE_POSITION, self.positions);
        m.insert_attribute(Mesh::ATTRIBUTE_NORMAL, self.normals);
        m.insert_attribute(Mesh::ATTRIBUTE_UV_0, self.uvs);
        m.insert_indices(Indices::U32(self.indices));
        m
    }
}

fn build_tiled_plane_mesh(w: f32, d: f32, tile_m: f32) -> Mesh {
    let mut acc = MeshAccum::default();
    let hw = 0.5 * w;
    let hd = 0.5 * d;
    acc.push_quad_up(
        Vec3::new(-hw, 0.0, -hd),
        Vec3::new(hw, 0.0, -hd),
        Vec3::new(hw, 0.0, hd),
        Vec3::new(-hw, 0.0, hd),
        w / tile_m,
        d / tile_m,
    );
    acc.build_mesh()
}

fn build_radar_wedge_mesh(range: f32, half_fov: f32) -> Mesh {
    let mut acc = MeshAccum::default();
    let mut prev = Vec3::new(0.0, 0.0, 0.0);
    let steps = 8usize;
    for k in 0..=steps {
        let ang = -half_fov + (k as f32 / steps as f32) * 2.0 * half_fov;
        let pt = Vec3::new(ang.sin() * range, 0.0, -ang.cos() * range);
        if k > 0 {
            let b = acc.positions.len() as u32;
            acc.positions.extend_from_slice(&[[0.0, 0.0, 0.0], prev.to_array(), pt.to_array()]);
            acc.normals.extend_from_slice(&[[0.0, 1.0, 0.0]; 3]);
            acc.uvs.extend_from_slice(&[[0.0, 0.0]; 3]);
            acc.indices.extend_from_slice(&[b, b + 1, b + 2]);
        }
        prev = pt;
    }
    acc.build_mesh()
}

#[inline]
fn chunk_coord(wx: f32, wz: f32) -> (i32, i32) {
    (
        (wx / CHUNK_SIZE_M).floor() as i32,
        (wz / CHUNK_SIZE_M).floor() as i32,
    )
}

fn add_strip_segment(
    acc: &mut MeshAccum,
    p0: (f32, f32, f32),
    p1: (f32, f32, f32),
    lat_a: f32,
    lat_b: f32,
    y: f32,
    seg_len: f32,
    uv_scale: f32,
) {
    let (r0x, r0z) = right_vec(p0.2);
    let (r1x, r1z) = right_vec(p1.2);
    let v0 = to_bevy_pos(p0.0 + r0x * lat_a, y, p0.1 + r0z * lat_a);
    let v1 = to_bevy_pos(p0.0 + r0x * lat_b, y, p0.1 + r0z * lat_b);
    let v2 = to_bevy_pos(p1.0 + r1x * lat_b, y, p1.1 + r1z * lat_b);
    let v3 = to_bevy_pos(p1.0 + r1x * lat_a, y, p1.1 + r1z * lat_a);
    acc.push_quad_up(v0, v1, v2, v3, ((lat_b - lat_a) / uv_scale).abs(), seg_len / uv_scale);
}

fn add_line_polyline(
    chunks: &mut HashMap<(i32, i32), ChunkBuffers>,
    pts: &[[f32; 2]],
    half_w: f32,
    y: f32,
    dashed: bool,
    is_yellow: bool,
    is_curb: bool,
) {
    let mut dist_acc = 0.0f32;
    for w in pts.windows(2) {
        let dx = w[1][0] - w[0][0];
        let dz = w[1][1] - w[0][1];
        let seg_len = (dx * dx + dz * dz).sqrt();
        if seg_len < 0.05 {
            continue;
        }
        let prev_acc = dist_acc;
        dist_acc += seg_len;
        if dashed && ((prev_acc / 3.5) as i32) % 2 != 0 {
            continue;
        }
        let rx = dz / seg_len;
        let rz = -dx / seg_len;
        let p0 = to_bevy_pos(w[0][0] - rx * half_w, y, w[0][1] - rz * half_w);
        let p1 = to_bevy_pos(w[0][0] + rx * half_w, y, w[0][1] + rz * half_w);
        let p2 = to_bevy_pos(w[1][0] + rx * half_w, y, w[1][1] + rz * half_w);
        let p3 = to_bevy_pos(w[1][0] - rx * half_w, y, w[1][1] - rz * half_w);
        let key = chunk_coord(0.5 * (w[0][0] + w[1][0]), 0.5 * (w[0][1] + w[1][1]));
        let buf = chunks.entry(key).or_insert_with(ChunkBuffers::new);
        let acc = if is_curb {
            &mut buf.sidewalk
        } else if is_yellow {
            &mut buf.yellow
        } else {
            &mut buf.white
        };
        acc.push_quad_up(p0, p1, p2, p3, 1.0, seg_len / 6.0);
    }
}

fn build_network_chunks(net: &RoadNetwork) -> HashMap<(i32, i32), ChunkBuffers> {
    let mut chunks: HashMap<(i32, i32), ChunkBuffers> = HashMap::new();
    let mut tree_cells: HashSet<(i32, i32)> = HashSet::new();

    if net.is_single_lane_hd {
        // Waymo HD urban road geometry
        for edge in &net.edges {
            let step_m = 4.5f32;
            let n = ((edge.total_length / step_m).ceil() as usize).max(2);
            let ds = edge.total_length / n as f32;
            let mut prev = edge.sample_ref(0.0);
            for k in 1..=n {
                let s = (k as f32 / n as f32) * edge.total_length;
                let cur = edge.sample_ref(s);
                let mx = 0.5 * (prev.0 + cur.0);
                let mz = 0.5 * (prev.1 + cur.1);
                let buf = chunks
                    .entry(chunk_coord(mx, mz))
                    .or_insert_with(ChunkBuffers::new);
                add_strip_segment(&mut buf.asphalt, (prev.0, prev.1, prev.2), (cur.0, cur.1, cur.2), -2.2, 2.2, 0.015, ds, 12.0);
                prev = cur;
            }
        }

        if let Some(ref wv) = net.waymo_visuals {
            for rl in &wv.road_lines {
                match rl.line_type.as_str() {
                    "DOUBLE_YELLOW" => {
                        add_line_polyline(&mut chunks, &rl.polyline, 0.14, 0.028, false, true, false);
                    }
                    "SOLID_YELLOW" => {
                        add_line_polyline(&mut chunks, &rl.polyline, 0.09, 0.026, false, true, false);
                    }
                    "BROKEN_WHITE" => {
                        add_line_polyline(&mut chunks, &rl.polyline, 0.08, 0.025, true, false, false);
                    }
                    "SOLID_WHITE" => {
                        add_line_polyline(&mut chunks, &rl.polyline, 0.09, 0.025, false, false, false);
                    }
                    "MEDIAN" | "BOUNDARY" => {
                        add_line_polyline(&mut chunks, &rl.polyline, 0.35, 0.045, false, false, true);
                    }
                    _ => {}
                }
            }

            // Zebra Crosswalks from Waymo polygons
            for cw in &wv.crosswalks {
                if cw.polygon.len() >= 3 {
                    let p0 = cw.polygon[0];
                    let p1 = cw.polygon[1];
                    let p2 = cw.polygon[2];
                    let p3 = *cw.polygon.get(3).unwrap_or(&cw.polygon[2]);
                    let cx = 0.25 * (p0[0] + p1[0] + p2[0] + p3[0]);
                    let cz = 0.25 * (p0[1] + p1[1] + p2[1] + p3[1]);
                    let buf = chunks.entry(chunk_coord(cx, cz)).or_insert_with(ChunkBuffers::new);
                    buf.white.push_quad_up(
                        to_bevy_pos(p0[0], 0.023, p0[1]),
                        to_bevy_pos(p1[0], 0.023, p1[1]),
                        to_bevy_pos(p2[0], 0.023, p2[1]),
                        to_bevy_pos(p3[0], 0.023, p3[1]),
                        1.0,
                        1.0,
                    );
                }
            }
        }
    } else {
        // Procedural Mega10km Multi-Lane Highway & City Blvd Network
        for edge in &net.edges {
            if !edge.feature.is_physical_corridor()
                && edge.feature != RoadFeatureKind::RoundaboutRing
            {
                continue;
            }
            let step_m = if edge.pieces.len() == 1 && edge.total_length > 220.0 {
                8.0
            } else {
                5.0
            };
            let n = ((edge.total_length / step_m).ceil() as usize).max(2);
            let ds = edge.total_length / n as f32;
            let mut prev = edge.sample_ref(0.0);

            for k in 1..=n {
                let s = (k as f32 / n as f32) * edge.total_length;
                let cur = edge.sample_ref(s);
                let mx = 0.5 * (prev.0 + cur.0);
                let mz = 0.5 * (prev.1 + cur.1);
                let buf = chunks
                    .entry(chunk_coord(mx, mz))
                    .or_insert_with(ChunkBuffers::new);

                let p0 = (prev.0, prev.1, prev.2);
                let p1 = (cur.0, cur.1, cur.2);

                // Main 3-lane carriageway surface + outer sidewalk curb
                add_strip_segment(&mut buf.asphalt, p0, p1, -5.25, 5.25, 0.015, ds, 12.0);
                add_strip_segment(&mut buf.sidewalk, p0, p1, 5.25, 7.2, 0.035, ds, 8.0);
                // Double yellow centerline divider on left edge (-5.25)
                add_strip_segment(&mut buf.yellow, p0, p1, -5.26, -5.10, 0.026, ds, 6.0);
                add_strip_segment(&mut buf.white, p0, p1, 5.10, 5.24, 0.025, ds, 6.0);
                if k % 2 == 0 {
                    add_strip_segment(&mut buf.white, p0, p1, -1.82, -1.68, 0.025, ds, 6.0);
                    add_strip_segment(&mut buf.white, p0, p1, 1.68, 1.82, 0.025, ds, 6.0);
                }

                // Stylized 3D roadside trees spaced cleanly beyond curb (+11.5 m)
                if edge.feature.is_physical_corridor() && k % 9 == 0 && s > 28.0 && s < edge.total_length - 28.0 {
                    let (ux, uz) = right_vec(cur.2);
                    let tx = cur.0 + ux * 11.8;
                    let tz = cur.1 + uz * 11.8;
                    let cell = ((tx / 22.0).round() as i32, (tz / 22.0).round() as i32);
                    let near_junction = net.junctions.iter().any(|j| ((tx - j.center_x).powi(2) + (tz - j.center_z).powi(2)).sqrt() < 42.0);
                    if !near_junction && tree_cells.insert(cell) {
                        let scale = 0.85 + ((edge.id * 17 + k) % 5) as f32 * 0.12;
                        let tbuf = chunks.entry(chunk_coord(tx, tz)).or_insert_with(ChunkBuffers::new);
                        let bpt = to_bevy_pos(tx, 0.0, tz);
                        tbuf.trunks.push_tree(bpt.x, bpt.z, scale, true);
                        tbuf.leaves.push_tree(bpt.x, bpt.z, scale, false);
                    }
                }

                prev = cur;
            }
        }

        // Landscaped central islands on roundabouts
        for j in &net.junctions {
            if j.kind != JunctionType::Roundabout {
                continue;
            }
            let buf = chunks
                .entry(chunk_coord(j.center_x, j.center_z))
                .or_insert_with(ChunkBuffers::new);
            let r = 21.5f32;
            let segs = 16usize;
            for s in 0..segs {
                let a0 = (s as f32 / segs as f32) * std::f32::consts::TAU;
                let a1 = ((s + 1) as f32 / segs as f32) * std::f32::consts::TAU;
                buf.leaves.push_quad_up(
                    to_bevy_pos(j.center_x, 0.12, j.center_z),
                    to_bevy_pos(j.center_x + a0.cos() * r, 0.05, j.center_z + a0.sin() * r),
                    to_bevy_pos(j.center_x + a1.cos() * r, 0.05, j.center_z + a1.sin() * r),
                    to_bevy_pos(j.center_x, 0.12, j.center_z),
                    1.0,
                    1.0,
                );
            }
        }
    }

    chunks
}

fn rebuild_static_world_on_map_change(
    mut commands: Commands,
    world_res: Res<SimWorldRes>,
    loaded: Res<LoadedAssets>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut pools: ResMut<DynamicPools>,
    old_chunks: Query<Entity, With<RoadChunk>>,
    old_ground_cam: Query<&Transform, (With<GroundFollower>, Without<RoadChunk>)>,
) {
    let _ = old_ground_cam;
    let world = &world_res.0;
    if pools.loaded_map_generation == world.map_generation {
        return;
    }

    // Clear previous static chunks & pooled traffic light/cone/NPC entities on map rollover
    for e in &old_chunks {
        commands.entity(e).despawn_recursive();
    }
    for (_, (e, _)) in pools.npcs.drain() {
        commands.entity(e).despawn_recursive();
    }
    for (_, e) in pools.peds.drain() {
        commands.entity(e).despawn_recursive();
    }
    for (_, e) in pools.cyclists.drain() {
        commands.entity(e).despawn_recursive();
    }
    for e in pools.cones.drain(..) {
        commands.entity(e).despawn_recursive();
    }
    for (e, _) in pools.signals.drain(..) {
        commands.entity(e).despawn_recursive();
    }

    let chunks = build_network_chunks(&world.net);
    for ((gx, gz), buf) in chunks {
        let cx = (gx as f32 + 0.5) * CHUNK_SIZE_M;
        let cz = (gz as f32 + 0.5) * CHUNK_SIZE_M;
        commands
            .spawn((
                RoadChunk {
                    center_x: cx,
                    center_z: cz,
                    generation: world.map_generation,
                },
                Transform::IDENTITY,
                Visibility::Visible,
            ))
            .with_children(|parent| {
                if !buf.asphalt.is_empty() {
                    parent.spawn((
                        Mesh3d(meshes.add(buf.asphalt.build_mesh())),
                        MeshMaterial3d(loaded.mat_asphalt.clone()),
                    ));
                }
                if !buf.yellow.is_empty() {
                    parent.spawn((
                        Mesh3d(meshes.add(buf.yellow.build_mesh())),
                        MeshMaterial3d(loaded.mat_yellow_line.clone()),
                    ));
                }
                if !buf.white.is_empty() {
                    parent.spawn((
                        Mesh3d(meshes.add(buf.white.build_mesh())),
                        MeshMaterial3d(loaded.mat_white_line.clone()),
                    ));
                }
                if !buf.sidewalk.is_empty() {
                    parent.spawn((
                        Mesh3d(meshes.add(buf.sidewalk.build_mesh())),
                        MeshMaterial3d(loaded.mat_sidewalk.clone()),
                    ));
                }
                if !buf.trunks.is_empty() {
                    parent.spawn((
                        Mesh3d(meshes.add(buf.trunks.build_mesh())),
                        MeshMaterial3d(loaded.mat_tree_trunk.clone()),
                    ));
                }
                if !buf.leaves.is_empty() {
                    parent.spawn((
                        Mesh3d(meshes.add(buf.leaves.build_mesh())),
                        MeshMaterial3d(loaded.mat_tree_leaf.clone()),
                    ));
                }
            });
    }

    pools.loaded_map_generation = world.map_generation;
    pools.cam_initialized = false;
}

fn spawn_npc_vehicle_gltf(
    commands: &mut Commands,
    loaded: &LoadedAssets,
    class: VehicleClass,
) -> Entity {
    let spec = spec_for_class(Some(class));
    let (chassis, tire_f, tire_b) = match class {
        VehicleClass::Sedan => (
            loaded.sedan_chassis.clone(),
            loaded.sedan_tire_f.clone(),
            loaded.sedan_tire_b.clone(),
        ),
        VehicleClass::Suv => (
            loaded.suv_chassis.clone(),
            loaded.suv_tire_f.clone(),
            loaded.suv_tire_b.clone(),
        ),
        VehicleClass::Truck => (
            loaded.truck_chassis.clone(),
            loaded.truck_tire_f.clone(),
            loaded.truck_tire_b.clone(),
        ),
    };

    commands
        .spawn((Transform::IDENTITY, Visibility::Visible))
        .with_children(|parent| {
            parent.spawn((
                SceneRoot(chassis),
                Transform::from_translation(spec.chassis_offset)
                    .with_scale(Vec3::splat(spec.chassis_scale)),
            ));
            for &(is_left, x, z, ref th) in &[
                (false, spec.lateral_x, spec.front_wheel_z, tire_f.clone()),
                (true, -spec.lateral_x, spec.front_wheel_z, tire_f),
                (false, spec.lateral_x, spec.rear_wheel_z, tire_b.clone()),
                (true, -spec.lateral_x, spec.rear_wheel_z, tire_b),
            ] {
                let rot = if is_left {
                    Quat::from_rotation_y(PI)
                } else {
                    Quat::IDENTITY
                };
                parent.spawn((
                    SceneRoot(th.clone()),
                    Transform::from_xyz(x, spec.tire_radius, z).with_rotation(rot),
                ));
            }
        })
        .id()
}

fn sync_ego_and_fsd_ribbon(
    time: Res<Time>,
    world_res: Res<SimWorldRes>,
    loaded: Res<LoadedAssets>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut pools: ResMut<DynamicPools>,
    mut q_ego: Query<&mut Transform, (With<EgoRoot>, Without<EgoWheel>, Without<StopBarEntity>, Without<GoalBeaconEntity>)>,
    mut q_wheels: Query<(&EgoWheel, &mut Transform), (Without<EgoRoot>, Without<StopBarEntity>, Without<GoalBeaconEntity>)>,
    mut q_lights: ParamSet<(
        Query<&mut Visibility, With<EgoBrakeLight>>,
        Query<(&EgoTurnSignal, &mut Visibility)>,
        Query<&mut Visibility, With<EgoOverlay2D>>,
        Query<(&EgoBsmArc, &mut Visibility)>,
    )>,
    mut q_stop: Query<(&mut Transform, &mut Visibility), (With<StopBarEntity>, Without<EgoRoot>, Without<EgoWheel>, Without<GoalBeaconEntity>)>,
    mut q_goal: Query<(&mut Transform, &mut Visibility), (With<GoalBeaconEntity>, Without<EgoRoot>, Without<EgoWheel>, Without<StopBarEntity>)>,
) {
    let world = &world_res.0;
    let dt = time.delta_secs().min(0.05);
    let now_s = time.elapsed_secs_f64();
    let ex = world.ego.x;
    let ez = world.ego.z;

    if let Ok(mut tf) = q_ego.get_single_mut() {
        tf.translation = to_bevy_pos(ex, 0.0, ez);
        tf.rotation = Quat::from_rotation_y(-world.ego.heading);
    }
    pools.wheel_spin_rad += (world.ego.speed / 0.313) * dt;
    for (ew, mut wtf) in &mut q_wheels {
        let steer = if ew.is_front { -world.ego.steer_angle } else { 0.0 };
        let roll = if ew.is_left { pools.wheel_spin_rad } else { -pools.wheel_spin_rad };
        wtf.rotation = Quat::from_rotation_y(steer) * Quat::from_rotation_x(roll);
    }

    let brake_vis = if world.last_adas.brake_lights || world.last_adas.brake > 0.08 || world.is_arrived_point_b() {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };
    for mut v in &mut q_lights.p0() {
        *v = brake_vis;
    }

    let blink_on = ((now_s * 3.0) as u64) % 2 == 0;
    for (ts, mut v) in &mut q_lights.p1() {
        let active = if ts.is_left {
            world.last_adas.left_turn_signal || world.last_adas.hazard_lights
        } else {
            world.last_adas.right_turn_signal || world.last_adas.hazard_lights
        };
        *v = if blink_on && active {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }

    let is_2d = world.config.view == ViewMode::View2D;
    for mut v in &mut q_lights.p2() {
        *v = if is_2d { Visibility::Visible } else { Visibility::Hidden };
    }
    for (bsm, mut v) in &mut q_lights.p3() {
        let warn = if bsm.is_left {
            world.last_adas.left_mirror_blind_spot
        } else {
            world.last_adas.right_mirror_blind_spot
        };
        *v = if warn { Visibility::Visible } else { Visibility::Hidden };
    }

    // Glowing 3D FSD Trajectory Ribbon
    if let Some(ref rib_h) = pools.ribbon_mesh {
        if let Some(mesh) = meshes.get_mut(rib_h) {
            let mut acc = MeshAccum::default();
            if !world.active_edges.is_empty() {
                let mut cursor = world.route_cursor;
                let mut s_cur = world.current_s_on_edge;
                let target_lat = if world.net.is_single_lane_hd {
                    world.last_adas.lateral_nudge
                } else {
                    (world.last_cue.current_lane as f32 - 1.0) * LANE_WIDTH_M + world.last_adas.lateral_nudge
                };
                let mut remaining = if let Some(ref p2p) = world.p2p {
                    if p2p.arrived { 2.0 } else { world.last_cue.dist_to_goal.clamp(4.0, 58.0) }
                } else {
                    58.0
                };
                let mut prev: Option<(Vec3, Vec3)> = None;
                let half_w = 0.58f32;
                while remaining > 0.0 && cursor < world.active_edges.len() {
                    let e = &world.net.edges[world.active_edges[cursor]];
                    while s_cur <= e.total_length && remaining > 0.0 {
                        let (px, pz, ph, _) = e.sample_at(s_cur, target_lat);
                        let (rx, rz) = right_vec(ph);
                        let pl = to_bevy_pos(px - rx * half_w, 0.045, pz - rz * half_w);
                        let pr = to_bevy_pos(px + rx * half_w, 0.045, pz + rz * half_w);
                        if let Some((al, ar)) = prev {
                            acc.push_quad_up(al, ar, pr, pl, 1.0, 1.0);
                        }
                        prev = Some((pl, pr));
                        s_cur += 3.5;
                        remaining -= 3.5;
                    }
                    s_cur -= e.total_length;
                    cursor += 1;
                }
            }
            *mesh = acc.build_mesh();
        }
        if let Some(mat) = materials.get_mut(&loaded.mat_ribbon) {
            if world.last_adas.drive_mode == 5
                || world
                    .perception
                    .last_signal_hud
                    .map(|s| s.color == LightColor::Red && s.distance_m < 32.0)
                    .unwrap_or(false)
            {
                mat.base_color = Color::srgba(1.0, 0.22, 0.18, 0.88);
                mat.emissive = LinearRgba::rgb(2.2, 0.25, 0.15);
            } else if matches!(world.last_adas.drive_mode, 2 | 3 | 4) {
                mat.base_color = Color::srgba(1.0, 0.72, 0.16, 0.85);
                mat.emissive = LinearRgba::rgb(1.6, 0.95, 0.1);
            } else {
                mat.base_color = Color::srgba(0.18, 0.72, 1.0, 0.85);
                mat.emissive = LinearRgba::rgb(0.15, 0.75, 1.4);
            }
        }
    }

    if let Ok((mut stf, mut svis)) = q_stop.get_single_mut() {
        if let Some(ref bar) = world.perception.last_stop_line {
            let sig_h = world
                .traffic
                .iter_signals()
                .min_by(|a, b| {
                    let da = (a.0.stop_x - bar.x).powi(2) + (a.0.stop_z - bar.z).powi(2);
                    let db = (b.0.stop_x - bar.x).powi(2) + (b.0.stop_z - bar.z).powi(2);
                    da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
                })
                .map(|(a, _, _)| a.heading)
                .unwrap_or(world.ego.heading);
            stf.translation = to_bevy_pos(bar.x, 0.05, bar.z);
            stf.rotation = Quat::from_rotation_y(-sig_h);
            *svis = Visibility::Visible;
        } else {
            *svis = Visibility::Hidden;
        }
    }
    if let Ok((mut gtf, mut gvis)) = q_goal.get_single_mut() {
        if let Some(ref p2p) = world.p2p {
            gtf.translation = to_bevy_pos(p2p.goal_world.0, 3.25, p2p.goal_world.1);
            *gvis = Visibility::Visible;
        } else {
            *gvis = Visibility::Hidden;
        }
    }
}

fn sync_npcs_peds_cyclists(
    mut commands: Commands,
    world_res: Res<SimWorldRes>,
    loaded: Res<LoadedAssets>,
    mut pools: ResMut<DynamicPools>,
    mut q_tf: Query<&mut Transform>,
) {
    let world = &world_res.0;
    let ex = world.ego.x;
    let ez = world.ego.z;

    let mut active_npc_ids = HashSet::with_capacity(world.traffic.vehicles.len());
    for v in &world.traffic.vehicles {
        if (v.x - ex).powi(2) + (v.z - ez).powi(2) > VISIBILITY_RADIUS_M * VISIBILITY_RADIUS_M {
            continue;
        }
        active_npc_ids.insert(v.id);
        let entity = if let Some(&(ent, cls)) = pools.npcs.get(&v.id) {
            if cls == v.class {
                ent
            } else {
                commands.entity(ent).despawn_recursive();
                let ne = spawn_npc_vehicle_gltf(&mut commands, &loaded, v.class);
                pools.npcs.insert(v.id, (ne, v.class));
                ne
            }
        } else {
            let ne = spawn_npc_vehicle_gltf(&mut commands, &loaded, v.class);
            pools.npcs.insert(v.id, (ne, v.class));
            ne
        };
        if let Ok(mut tf) = q_tf.get_mut(entity) {
            tf.translation = to_bevy_pos(v.x, 0.0, v.z);
            tf.rotation = Quat::from_rotation_y(-v.heading);
        }
    }
    pools.npcs.retain(|id, (ent, _)| {
        if active_npc_ids.contains(id) {
            true
        } else {
            commands.entity(*ent).despawn_recursive();
            false
        }
    });

    let mut active_ped_ids = HashSet::with_capacity(world.traffic.pedestrians.len());
    for p in &world.traffic.pedestrians {
        if (p.x - ex).powi(2) + (p.z - ez).powi(2) > VISIBILITY_RADIUS_M * VISIBILITY_RADIUS_M {
            continue;
        }
        active_ped_ids.insert(p.id);
        let entity = *pools.peds.entry(p.id).or_insert_with(|| {
            commands
                .spawn((Transform::IDENTITY, Visibility::Visible))
                .with_children(|par| {
                    par.spawn((
                        SceneRoot(loaded.pedestrian_scene.clone()),
                        Transform::from_xyz(0.0, 0.01, 0.0)
                            .with_rotation(Quat::from_rotation_y(PI))
                            .with_scale(Vec3::splat(0.0096)),
                    ));
                })
                .id()
        });
        if let Ok(mut tf) = q_tf.get_mut(entity) {
            tf.translation = to_bevy_pos(p.x, 0.0, p.z);
            tf.rotation = Quat::from_rotation_y(-p.heading);
        }
    }
    pools.peds.retain(|id, ent| {
        if active_ped_ids.contains(id) {
            true
        } else {
            commands.entity(*ent).despawn_recursive();
            false
        }
    });

    let mut active_cyc_ids = HashSet::with_capacity(world.traffic.cyclists.len());
    for cyc in &world.traffic.cyclists {
        if (cyc.x - ex).powi(2) + (cyc.z - ez).powi(2) > VISIBILITY_RADIUS_M * VISIBILITY_RADIUS_M {
            continue;
        }
        active_cyc_ids.insert(cyc.id);
        let entity = *pools.cyclists.entry(cyc.id).or_insert_with(|| {
            commands
                .spawn((Transform::IDENTITY, Visibility::Visible))
                .with_children(|par| {
                    par.spawn((
                        SceneRoot(loaded.bicycle_scene.clone()),
                        Transform::from_xyz(0.0, 0.45, 0.0)
                            .with_rotation(Quat::from_rotation_y(-FRAC_PI_2))
                            .with_scale(Vec3::splat(0.11)),
                    ));
                })
                .id()
        });
        if let Ok(mut tf) = q_tf.get_mut(entity) {
            tf.translation = to_bevy_pos(cyc.x, 0.0, cyc.z);
            tf.rotation = Quat::from_rotation_y(-cyc.heading);
        }
    }
    pools.cyclists.retain(|id, ent| {
        if active_cyc_ids.contains(id) {
            true
        } else {
            commands.entity(*ent).despawn_recursive();
            false
        }
    });
}

fn sync_cones_and_signals(
    mut commands: Commands,
    world_res: Res<SimWorldRes>,
    loaded: Res<LoadedAssets>,
    mut pools: ResMut<DynamicPools>,
    mut q_tf: Query<&mut Transform>,
    mut q_vis: Query<&mut Visibility>,
) {
    let world = &world_res.0;
    let ex = world.ego.x;
    let ez = world.ego.z;

    while pools.cones.len() < world.traffic.cones.len() {
        let e = commands
            .spawn((
                SceneRoot(loaded.cone_scene.clone()),
                Transform::from_scale(Vec3::splat(0.028)),
                Visibility::Hidden,
            ))
            .id();
        pools.cones.push(e);
    }
    for (idx, ent) in pools.cones.iter().enumerate() {
        if let Some(cone) = world.traffic.cones.get(idx) {
            if let Ok(mut tf) = q_tf.get_mut(*ent) {
                tf.translation = to_bevy_pos(cone.x, 0.0, cone.z);
            }
            if let Ok(mut v) = q_vis.get_mut(*ent) {
                *v = Visibility::Visible;
            }
        } else if let Ok(mut v) = q_vis.get_mut(*ent) {
            *v = Visibility::Hidden;
        }
    }

    let nearby_sigs: Vec<(f32, f32, f32, LightColor)> = world
        .traffic
        .iter_signals()
        .filter(|(app, _, _)| {
            (app.stop_x - ex).powi(2) + (app.stop_z - ez).powi(2)
                <= VISIBILITY_RADIUS_M * VISIBILITY_RADIUS_M
        })
        .take(24)
        .map(|(app, col, _)| {
            let (rx, rz) = right_vec(app.heading);
            (app.stop_x + rx * 4.5, app.stop_z + rz * 4.5, app.heading, col)
        })
        .collect();

    for (idx, &(px, pz, h, col)) in nearby_sigs.iter().enumerate() {
        let need_spawn = match pools.signals.get(idx) {
            Some((_, prev_col)) => *prev_col != col,
            None => true,
        };
        if need_spawn {
            if let Some((old_e, _)) = pools.signals.get(idx) {
                commands.entity(*old_e).despawn_recursive();
            }
            let (head_scene, head_scale) = match col {
                LightColor::Red => (loaded.light_red_scene.clone(), Vec3::new(0.70, 1.15, 1.15)),
                LightColor::Yellow => (loaded.light_yellow_scene.clone(), Vec3::new(1.30, 1.30, 1.15)),
                LightColor::Green => (loaded.light_green_scene.clone(), Vec3::new(0.70, 1.15, 1.15)),
            };
            let new_e = commands
                .spawn((
                    Transform::from_translation(to_bevy_pos(px, 0.0, pz))
                        .with_rotation(Quat::from_rotation_y(-h)),
                    Visibility::Visible,
                ))
                .with_children(|par| {
                    par.spawn((
                        Mesh3d(loaded.pole_mesh.clone()),
                        MeshMaterial3d(loaded.mat_pole.clone()),
                        Transform::from_xyz(0.0, 2.6, 0.0),
                    ));
                    par.spawn((
                        Mesh3d(loaded.arm_mesh.clone()),
                        MeshMaterial3d(loaded.mat_pole.clone()),
                        Transform::from_xyz(-2.0, 5.05, 0.0),
                    ));
                    par.spawn((
                        SceneRoot(head_scene),
                        Transform::from_xyz(-3.8, 5.05, 0.0).with_scale(head_scale),
                    ));
                })
                .id();
            if idx < pools.signals.len() {
                pools.signals[idx] = (new_e, col);
            } else {
                pools.signals.push((new_e, col));
            }
        } else {
            let ent = pools.signals[idx].0;
            if let Ok(mut tf) = q_tf.get_mut(ent) {
                tf.translation = to_bevy_pos(px, 0.0, pz);
                tf.rotation = Quat::from_rotation_y(-h);
            }
            if let Ok(mut v) = q_vis.get_mut(ent) {
                *v = Visibility::Visible;
            }
        }
    }
    for &(ent, _) in pools.signals.iter().skip(nearby_sigs.len()) {
        if let Ok(mut v) = q_vis.get_mut(ent) {
            *v = Visibility::Hidden;
        }
    }
}

fn update_chunk_visibility_and_ground(
    world_res: Res<SimWorldRes>,
    mut q_chunks: Query<(&RoadChunk, &mut Visibility)>,
    mut q_ground: Query<&mut Transform, With<GroundFollower>>,
) {
    let world = &world_res.0;
    let ex = world.ego.x;
    let ez = world.ego.z;

    let max_d = VISIBILITY_RADIUS_M + CHUNK_SIZE_M * 0.75;
    let max_d2 = max_d * max_d;
    for (chunk, mut vis) in &mut q_chunks {
        let d2 = (chunk.center_x - ex).powi(2) + (chunk.center_z - ez).powi(2);
        *vis = if d2 <= max_d2 {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }

    // Snap textured ground plane in 48 m grid increments to match UV tile scale cleanly with zero swimming
    if let Ok(mut gtf) = q_ground.get_single_mut() {
        let sx = (ex / 48.0).round() * 48.0;
        let sz = (ez / 48.0).round() * 48.0;
        gtf.translation = to_bevy_pos(sx, -0.04, sz);
    }
}

fn update_camera_system(
    time: Res<Time>,
    world_res: Res<SimWorldRes>,
    mut pools: ResMut<DynamicPools>,
    mut q_cam: Query<&mut Transform, With<MainDriveCamera>>,
) {
    let world = &world_res.0;
    let dt = time.delta_secs().clamp(0.001, 0.05);
    let Ok(mut cam_tf) = q_cam.get_single_mut() else {
        return;
    };

    let (efx, efz) = forward_vec(world.ego.heading);
    let _ = normalize_angle_rad;

    match world.config.view {
        ViewMode::View3D => {
            let desired_pos = to_bevy_pos(
                world.ego.x - efx * 11.8,
                4.85,
                world.ego.z - efz * 11.8,
            );
            let desired_tgt = to_bevy_pos(
                world.ego.x + efx * 18.5,
                1.25,
                world.ego.z + efz * 18.5,
            );
            if !pools.cam_initialized || pools.cam_pos.distance(desired_pos) > 45.0 {
                pools.cam_pos = desired_pos;
                pools.cam_target = desired_tgt;
                pools.cam_initialized = true;
            } else {
                let alpha = (1.0 - (-9.0 * dt).exp()).clamp(0.05, 0.45);
                pools.cam_pos = pools.cam_pos.lerp(desired_pos, alpha);
                pools.cam_target = pools.cam_target.lerp(desired_tgt, alpha);
            }
            *cam_tf = Transform::from_translation(pools.cam_pos).looking_at(pools.cam_target, Vec3::Y);
        }
        ViewMode::View2D => {
            // Crisp Tesla FSD Top-Down Bird's-Eye-View camera with ego facing straight up
            let pos = to_bevy_pos(
                world.ego.x + efx * 14.0,
                76.0,
                world.ego.z + efz * 14.0,
            );
            let tgt = to_bevy_pos(world.ego.x + efx * 14.5, 0.0, world.ego.z + efz * 14.5);
            let fwd_bev = to_bevy_pos(efx, 0.0, efz).normalize_or_zero();
            let up = if fwd_bev.length_squared() > 0.5 { fwd_bev } else { Vec3::NEG_Z };
            pools.cam_pos = pos;
            pools.cam_target = tgt;
            *cam_tf = Transform::from_translation(pos).looking_at(tgt, up);
        }
    }
}
