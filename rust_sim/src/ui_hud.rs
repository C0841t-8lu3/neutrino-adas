//! `bevy_egui` minimalist 60 FPS Telemetry HUD, clickable minimap, full-screen interactive `[M]`
//! Point A -> Point B navigation map, fixed-timestep simulation ticker, and hotkey controller.

use crate::config::{MapMode, DT_60HZ};
use crate::map_overlay::MapViewport;
use crate::render_setup::SimWorldRes;
use crate::road_network::RoadFeatureKind;
use crate::scene_3d::DynamicPools;
use crate::simulation::SimWorld;
use crate::traffic::LightColor;
use bevy::app::AppExit;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use bevy::window::WindowCloseRequested;
use bevy_egui::{egui, EguiContexts};

#[derive(Resource, Default)]
pub struct SimUiState {
    pub accumulator_s: f32,
    pub map_vp: Option<MapViewport>,
    pub prev_map_gen: u64,
    pub exported_on_exit: bool,
}

pub struct UiHudPlugin;

impl Plugin for UiHudPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SimUiState>()
            .add_systems(Update, (handle_hotkeys_and_step, draw_ui_hud).chain());
    }
}

fn export_and_log(world: &SimWorld) {
    let rtt = world.rtt_stats();
    let summary = world.recorder.summary(rtt);
    if let Ok((jp, cp)) = world.recorder.export_all(&world.config.report_prefix, rtt) {
        println!(
            "QNX 8.0 ADAS Summary: ticks={} odom={:.3}km rms_lat={:.3}m collisions={} p50={:.0}us -> {}, {}",
            summary.total_ticks,
            summary.odometer_km,
            summary.rms_lateral_error_m,
            summary.collision_ticks,
            summary.qnx_rtt_p50_us,
            jp.display(),
            cp.display()
        );
    }
}

#[allow(clippy::too_many_arguments)]
fn handle_hotkeys_and_step(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mut world_res: ResMut<SimWorldRes>,
    mut ui_state: ResMut<SimUiState>,
    mut pools: ResMut<DynamicPools>,
    mut windows: Query<&mut Window>,
    mut close_ev: EventReader<WindowCloseRequested>,
    mut exit_ev: EventWriter<AppExit>,
) {
    let world = &mut world_res.0;

    if !close_ev.is_empty() {
        close_ev.clear();
        if !ui_state.exported_on_exit {
            export_and_log(world);
            ui_state.exported_on_exit = true;
        }
        exit_ev.send(AppExit::Success);
        return;
    }

    if keys.just_pressed(KeyCode::KeyV) {
        world.config.view = world.config.view.toggle();
        pools.cam_initialized = false;
        world.show_toast(format!("Switched View: {}", world.config.view.label()), 2.0);
        if let Ok(mut win) = windows.get_single_mut() {
            win.title = format!(
                "QNX 8.0 ADAS [{} | {} | {}]",
                world.config.map_mode.label(),
                world.config.view.label(),
                world.config.profile.label()
            );
        }
    }
    if keys.just_pressed(KeyCode::KeyM) {
        world.map_open = !world.map_open;
        if world.map_open {
            ui_state.map_vp = Some(MapViewport::fit_network(&world.net, 960.0, 680.0));
        }
    }
    if keys.just_pressed(KeyCode::KeyC) {
        world.resume_free_cruise();
        if let Some(ref mut vp) = ui_state.map_vp {
            vp.preview = None;
        }
    }
    if keys.just_pressed(KeyCode::KeyG) {
        let next_seed = world.config.seed + 1;
        world.regenerate_new_map(next_seed, false);
        ui_state.map_vp = Some(MapViewport::fit_network(&world.net, 960.0, 680.0));
    }
    if keys.just_pressed(KeyCode::KeyP) {
        world.cycle_profile();
    }
    if keys.just_pressed(KeyCode::KeyT) {
        world.trigger_traffic_jam();
    }
    if keys.just_pressed(KeyCode::KeyJ) {
        world.trigger_jaywalker();
    }
    if keys.just_pressed(KeyCode::KeyW) {
        world.toggle_wet();
    }
    if keys.just_pressed(KeyCode::KeyN) {
        world.toggle_noise();
    }
    if keys.just_pressed(KeyCode::Space) {
        world.paused = !world.paused;
        world.show_toast(if world.paused { "PAUSED" } else { "RESUMED" }, 1.6);
    }
    if keys.just_pressed(KeyCode::KeyR) {
        match world
            .recorder
            .export_all(&world.config.report_prefix, world.rtt_stats())
        {
            Ok((j, c)) => {
                world.show_toast(format!("Exported {} & {}", j.display(), c.display()), 3.0)
            }
            Err(e) => world.show_toast(format!("Export failed: {}", e), 3.0),
        }
    }
    if keys.just_pressed(KeyCode::Enter) && world.map_open {
        let goal = ui_state
            .map_vp
            .as_ref()
            .and_then(|vp| vp.preview.as_ref().map(|p| p.goal_world));
        if let Some((gx, gz)) = goal {
            world.set_destination(gx, gz);
            world.map_open = false;
            if let Some(ref mut vp) = ui_state.map_vp {
                vp.preview = None;
            }
        }
    }
    if keys.just_pressed(KeyCode::KeyQ) {
        if !ui_state.exported_on_exit {
            export_and_log(world);
            ui_state.exported_on_exit = true;
        }
        exit_ev.send(AppExit::Success);
        return;
    }
    if keys.just_pressed(KeyCode::Escape) {
        if world.map_open {
            world.map_open = false;
            if let Some(ref mut vp) = ui_state.map_vp {
                vp.preview = None;
            }
        } else {
            if !ui_state.exported_on_exit {
                export_and_log(world);
                ui_state.exported_on_exit = true;
            }
            exit_ev.send(AppExit::Success);
            return;
        }
    }

    let frame_dt = time.delta_secs().min(0.05);
    ui_state.accumulator_s += frame_dt;
    let mut substeps = 0u32;
    while ui_state.accumulator_s >= DT_60HZ && substeps < 4 {
        world.tick_fixed(DT_60HZ);
        ui_state.accumulator_s -= DT_60HZ;
        substeps += 1;
    }
    if substeps == 4 && ui_state.accumulator_s > DT_60HZ {
        ui_state.accumulator_s = 0.0;
    }

    if ui_state.prev_map_gen != world.map_generation {
        ui_state.prev_map_gen = world.map_generation;
        ui_state.map_vp = Some(MapViewport::fit_network(&world.net, 960.0, 680.0));
    }
}

fn draw_ui_hud(
    mut contexts: EguiContexts,
    time: Res<Time>,
    diagnostics: Res<DiagnosticsStore>,
    mut world_res: ResMut<SimWorldRes>,
    mut ui_state: ResMut<SimUiState>,
) {
    let ctx = contexts.ctx_mut();
    let world = &mut world_res.0;
    let fps = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FPS)
        .and_then(|d| d.smoothed())
        .unwrap_or(60.0) as f32;
    let now_s = time.elapsed_secs_f64();

    // 1. Minimalist Right Telemetry Sidebar
    egui::SidePanel::right("qnx_telemetry_sidebar")
        .resizable(false)
        .exact_width(316.0)
        .frame(
            egui::Frame::NONE
                .fill(egui::Color32::from_rgba_unmultiplied(12, 18, 30, 236))
                .inner_margin(egui::Margin::same(14)),
        )
        .show(ctx, |ui| {
            ui.label(
                egui::RichText::new("QNX 8.0 SAFETY KERNEL")
                    .size(17.5)
                    .strong()
                    .color(egui::Color32::from_rgb(56, 189, 248)),
            );
            let map_tag = match world.waymo_scenario_id() {
                Some(id) => format!("WAYMO: {}", id),
                None => format!("MEGA-10KM (Seed {})", world.config.seed),
            };
            ui.label(
                egui::RichText::new(format!(
                    "{}  |  {}  |  {:.0} FPS",
                    world.config.view.label(),
                    map_tag,
                    fps
                ))
                .size(11.5)
                .color(egui::Color32::from_rgb(148, 163, 184)),
            );
            ui.add_space(8.0);

            egui::Frame::NONE
                .fill(egui::Color32::from_rgb(24, 34, 52))
                .corner_radius(6.0_f32)
                .inner_margin(egui::Margin::same(10))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.label(
                            egui::RichText::new(format!("{:02.0} km/h", world.ego.speed * 3.6))
                                .size(30.0)
                                .strong()
                                .color(egui::Color32::WHITE),
                        );
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            let mode_col = match world.last_adas.drive_mode {
                                5 => egui::Color32::from_rgb(248, 113, 113),
                                2 | 3 | 4 => egui::Color32::from_rgb(250, 204, 21),
                                6 => egui::Color32::from_rgb(56, 189, 248),
                                _ => egui::Color32::from_rgb(52, 211, 153),
                            };
                            ui.label(
                                egui::RichText::new(world.last_adas.mode_name())
                                    .size(13.0)
                                    .strong()
                                    .color(mode_col),
                            );
                        });
                    });
                });

            ui.add_space(8.0);
            let rtt = world.rtt_stats();
            let lat_cm = (world
                .recorder
                .records
                .back()
                .map(|r| r.lateral_error_m)
                .unwrap_or(0.0)
                * 100.0)
                .abs();
            let rms_cm = world.recorder.rms_lateral_error_m() * 100.0;

            let sig_txt = match world.last_signal_hud() {
                Some(s) => format!(
                    "{} in {:.0}m ({:.0}s)",
                    s.color.label(),
                    s.distance_m,
                    s.countdown_s
                ),
                None => "Clear Ahead".to_string(),
            };
            let lead_txt = if world.last_adas.lead_distance < 90.0 {
                format!(
                    "{:.1}m | TTC {:.1}s",
                    world.last_adas.lead_distance, world.last_adas.time_to_collision
                )
            } else {
                "Open Lane".to_string()
            };
            let route_txt = if let Some(ref p2p) = world.p2p {
                if p2p.arrived {
                    "Arrived Point B".to_string()
                } else {
                    format!("Point B: {:.0} m", world.last_cue.dist_to_goal)
                }
            } else if world.config.map_mode == MapMode::Waymo {
                let dur = world
                    .waymo_scenarios
                    .get(world.waymo_idx)
                    .map(|s| s.duration_s())
                    .unwrap_or(9.0);
                format!("Clip {:.1}/{:.1}s", world.waymo_time_s, dur)
            } else {
                format!("{:.2} km Loop", world.net.grand_tour_length_m / 1000.0)
            };

            egui::Grid::new("hud_kv")
                .num_columns(2)
                .spacing([12.0, 5.0])
                .show(ui, |ui| {
                    let mut row = |k: &str, v: String, col: egui::Color32| {
                        ui.label(
                            egui::RichText::new(k)
                                .size(12.5)
                                .color(egui::Color32::from_rgb(148, 163, 184)),
                        );
                        ui.label(egui::RichText::new(v).size(12.5).strong().color(col));
                        ui.end_row();
                    };
                    row(
                        "Profile [P]",
                        world.config.profile.label().to_string(),
                        egui::Color32::GOLD,
                    );
                    row(
                        "Surface [W]",
                        format!(
                            "{} (mu={:.2})",
                            world.config.surface_label(),
                            world.config.current_friction()
                        ),
                        if world.config.wet_surface {
                            egui::Color32::LIGHT_BLUE
                        } else {
                            egui::Color32::WHITE
                        },
                    );
                    row(
                        "Feature",
                        world.last_cue.feature.label().to_string(),
                        egui::Color32::WHITE,
                    );
                    row("Signal", sig_txt, egui::Color32::WHITE);
                    row("Lead / TTC", lead_txt, egui::Color32::WHITE);
                    row(
                        "Lane Error",
                        format!("{:.1} cm (RMS {:.1} cm)", lat_cm, rms_cm),
                        if rms_cm < 12.0 {
                            egui::Color32::from_rgb(52, 211, 153)
                        } else {
                            egui::Color32::GOLD
                        },
                    );
                    row("Route", route_txt, egui::Color32::from_rgb(56, 189, 248));
                    row(
                        "Odometer",
                        format!(
                            "{:.3} km  |  {}",
                            world.recorder.cumulative_odometer_m / 1000.0,
                            world.recorder.formatted_drive_time()
                        ),
                        egui::Color32::WHITE,
                    );
                    row(
                        "BSM L / R",
                        format!(
                            "{} / {}",
                            if world.last_adas.left_mirror_blind_spot {
                                "WARN"
                            } else {
                                "OK"
                            },
                            if world.last_adas.right_mirror_blind_spot {
                                "WARN"
                            } else {
                                "OK"
                            }
                        ),
                        if world.last_adas.left_mirror_blind_spot
                            || world.last_adas.right_mirror_blind_spot
                        {
                            egui::Color32::from_rgb(251, 146, 60)
                        } else {
                            egui::Color32::from_rgb(52, 211, 153)
                        },
                    );
                    row(
                        "QNX RTT",
                        format!(
                            "p50 {:.0} us | WD {}",
                            rtt.p50_us,
                            if world.last_adas.watchdog_ok {
                                "OK"
                            } else {
                                "TRIP"
                            }
                        ),
                        if world.last_adas.watchdog_ok {
                            egui::Color32::from_rgb(52, 211, 153)
                        } else {
                            egui::Color32::RED
                        },
                    );
                });

            ui.add_space(10.0);

            let mm_size = egui::vec2(288.0, 172.0);
            let (mm_rect, mm_resp) = ui.allocate_exact_size(mm_size, egui::Sense::click());
            let painter = ui.painter_at(mm_rect);
            painter.rect_filled(mm_rect, 6.0_f32, egui::Color32::from_rgb(9, 14, 24));
            painter.rect_stroke(
                mm_rect,
                6.0_f32,
                egui::Stroke::new(
                    1.2_f32,
                    if mm_resp.hovered() {
                        egui::Color32::GOLD
                    } else {
                        egui::Color32::from_rgb(56, 189, 248)
                    },
                ),
                egui::StrokeKind::Inside,
            );
            draw_minimap_painter(&painter, mm_rect, world);
            painter.text(
                mm_rect.left_top() + egui::vec2(8.0, 6.0),
                egui::Align2::LEFT_TOP,
                "CLICK MAP [M] TO SET POINT B",
                egui::FontId::proportional(11.0),
                egui::Color32::from_rgb(56, 189, 248),
            );
            if mm_resp.clicked() {
                world.map_open = true;
                ui_state.map_vp = Some(MapViewport::fit_network(&world.net, 960.0, 680.0));
            }

            ui.add_space(10.0);
            ui.separator();
            ui.label(
                egui::RichText::new("HOTKEYS")
                    .size(12.0)
                    .strong()
                    .color(egui::Color32::from_rgb(56, 189, 248)),
            );
            for line in [
                "[V] Toggle 3D/2D    [M] Nav Map",
                "[C] Free Cruise     [G] New Map/Scene",
                "[P] Profile         [W] Wet/Dry",
                "[T] Traffic Jam     [J] Jaywalker",
                "[N] Radar Noise     [SPACE] Pause",
                "[R] Export Report   [Q/ESC] Quit",
            ] {
                ui.label(
                    egui::RichText::new(line)
                        .size(11.5)
                        .color(egui::Color32::from_rgb(203, 213, 225)),
                );
            }
        });

    // 2. Top Banners (Arrival Hold / Toast Notification)
    if world.is_arrived_point_b() {
        egui::Area::new(egui::Id::new("arrival_banner"))
            .anchor(egui::Align2::CENTER_TOP, egui::vec2(-158.0, 14.0))
            .show(ctx, |ui| {
                egui::Frame::NONE
                    .fill(egui::Color32::from_rgba_unmultiplied(6, 78, 59, 236))
                    .stroke(egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(52, 211, 153)))
                    .corner_radius(8.0_f32)
                    .inner_margin(egui::Margin::symmetric(20, 10))
                    .show(ui, |ui| {
                        ui.label(
                            egui::RichText::new("ARRIVED AT POINT B - SAFE STOP HELD")
                                .size(17.0)
                                .strong()
                                .color(egui::Color32::from_rgb(167, 243, 208)),
                        );
                        ui.label(
                            egui::RichText::new("Press [M] for New Destination  or  [C] to Resume Free Cruise")
                                .size(13.0)
                                .color(egui::Color32::WHITE),
                        );
                    });
            });
    } else if let Some(msg) = world.active_toast() {
        egui::Area::new(egui::Id::new("toast_banner"))
            .anchor(egui::Align2::CENTER_TOP, egui::vec2(-158.0, 14.0))
            .show(ctx, |ui| {
                egui::Frame::NONE
                    .fill(egui::Color32::from_rgba_unmultiplied(15, 23, 42, 228))
                    .stroke(egui::Stroke::new(1.5_f32, egui::Color32::from_rgb(56, 189, 248)))
                    .corner_radius(6.0_f32)
                    .inner_margin(egui::Margin::symmetric(16, 8))
                    .show(ui, |ui| {
                        ui.label(egui::RichText::new(msg).size(15.0).color(egui::Color32::GOLD));
                    });
            });
    }

    // 3. Full-Screen Interactive [M] Point A -> Point B Navigation Map
    if world.map_open {
        let screen_r = ctx.screen_rect();
        egui::Area::new(egui::Id::new("fullscreen_nav_map"))
            .fixed_pos(screen_r.left_top())
            .order(egui::Order::Foreground)
            .show(ctx, |ui| {
                ui.set_min_size(screen_r.size());
                ui.set_max_size(screen_r.size());
                egui::Frame::NONE
                    .fill(egui::Color32::from_rgba_unmultiplied(8, 12, 20, 246))
                    .inner_margin(egui::Margin::same(14))
                    .show(ui, |ui| {
                        draw_fullscreen_map(ui, world, &mut ui_state, now_s);
                    });
            });
    }
}

fn draw_minimap_painter(painter: &egui::Painter, rect: egui::Rect, world: &SimWorld) {
    let (min_x, max_x, min_z, max_z) = world.net.bounds;
    let span = (max_x - min_x + 80.0).max(max_z - min_z + 80.0).max(60.0);
    let pad = 12.0_f32;
    let inner_w = rect.width() - 2.0 * pad;
    let inner_h = rect.height() - 2.0 * pad;
    let scale = inner_w.min(inner_h) / span;
    let cx_w = 0.5 * (min_x + max_x);
    let cz_w = 0.5 * (min_z + max_z);

    let to_mm = |wx: f32, wz: f32| -> egui::Pos2 {
        egui::pos2(
            rect.center().x + (wx - cx_w) * scale,
            rect.center().y - (wz - cz_w) * scale,
        )
    };

    for edge in &world.net.edges {
        if !edge.feature.is_physical_corridor() && edge.feature != RoadFeatureKind::WaymoLane {
            continue;
        }
        let n = 6usize;
        let mut pts = Vec::with_capacity(n + 1);
        for k in 0..=n {
            let s = (k as f32 / n as f32) * edge.total_length;
            let (wx, wz, _, _) = edge.sample_ref(s);
            pts.push(to_mm(wx, wz));
        }
        painter.add(egui::Shape::line(
            pts,
            egui::Stroke::new(1.4_f32, egui::Color32::from_rgb(100, 116, 139)),
        ));
    }

    if let Some(ref p2p) = world.p2p {
        let pts: Vec<egui::Pos2> = p2p.polyline.iter().map(|&(x, z)| to_mm(x, z)).collect();
        painter.add(egui::Shape::line(
            pts,
            egui::Stroke::new(2.2_f32, egui::Color32::from_rgb(16, 185, 129)),
        ));
        let gp = to_mm(p2p.goal_world.0, p2p.goal_world.1);
        painter.circle_filled(gp, 4.0, egui::Color32::RED);
    }

    let ep = to_mm(world.ego.x, world.ego.z);
    painter.circle_filled(ep, 4.2, egui::Color32::from_rgb(56, 189, 248));
    painter.circle_stroke(ep, 6.2, egui::Stroke::new(1.2_f32, egui::Color32::WHITE));
}

fn draw_fullscreen_map(
    ui: &mut egui::Ui,
    world: &mut SimWorld,
    ui_state: &mut SimUiState,
    now_s: f64,
) {
    let mut close_now = false;
    let mut fit_now = false;
    let mut commit_now = false;
    let mut cruise_now = false;

    ui.horizontal(|ui| {
        let title = match world.waymo_scenario_id() {
            Some(id) => format!("WAYMO HD NAVIGATION MAP ({})", id),
            None => format!(
                "NAVIGATION MAP (Seed {} | {:.2} km Loop)",
                world.config.seed,
                world.net.grand_tour_length_m / 1000.0
            ),
        };
        ui.label(
            egui::RichText::new(title)
                .size(18.0)
                .strong()
                .color(egui::Color32::WHITE),
        );
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui
                .button(egui::RichText::new(" CLOSE [M/ESC] ").color(egui::Color32::WHITE))
                .clicked()
            {
                close_now = true;
            }
            if ui.button(" FIT MAP ").clicked() {
                fit_now = true;
            }
        });
    });

    ui.add_space(6.0);
    let canvas_h = (ui.available_height() - 68.0).max(200.0);
    let canvas_w = ui.available_width().max(320.0);
    let (canvas_rect, resp) =
        ui.allocate_exact_size(egui::vec2(canvas_w, canvas_h), egui::Sense::click_and_drag());

    let vp = ui_state
        .map_vp
        .get_or_insert_with(|| MapViewport::fit_network(&world.net, canvas_w, canvas_h));
    vp.update_screen_size(canvas_w, canvas_h);

    if fit_now {
        let keep = vp.preview.clone();
        *vp = MapViewport::fit_network(&world.net, canvas_w, canvas_h);
        vp.preview = keep;
    }

    if resp.hovered() {
        let scroll = ui.input(|i| i.raw_scroll_delta.y);
        if scroll.abs() > 0.5 {
            if let Some(mpos) = resp.hover_pos() {
                let factor = if scroll > 0.0 { 0.85 } else { 1.18 };
                vp.zoom_at(
                    mpos.x - canvas_rect.left(),
                    mpos.y - canvas_rect.top(),
                    factor,
                );
            }
        }
    }
    if resp.dragged() {
        let delta = resp.drag_delta();
        vp.pan_by_pixels(delta.x, delta.y);
    }
    if resp.double_clicked() {
        if let Some(mpos) = resp.interact_pointer_pos() {
            let (wx, wz) =
                vp.screen_to_world(mpos.x - canvas_rect.left(), mpos.y - canvas_rect.top());
            world.set_destination(wx, wz);
            vp.preview = None;
            world.map_open = false;
            return;
        }
    } else if resp.clicked() {
        if let Some(mpos) = resp.interact_pointer_pos() {
            vp.preview_click(world, mpos.x - canvas_rect.left(), mpos.y - canvas_rect.top());
        }
    }

    let to_canvas = |vp: &MapViewport, wx: f32, wz: f32| -> egui::Pos2 {
        let (sx, sy) = vp.world_to_screen(wx, wz);
        egui::pos2(canvas_rect.left() + sx, canvas_rect.top() + sy)
    };

    let painter = ui.painter_at(canvas_rect);
    painter.rect_filled(canvas_rect, 4.0_f32, egui::Color32::from_rgb(12, 17, 27));

    let road_px_w = if world.net.is_single_lane_hd {
        (4.5_f32 / vp.zoom_m_per_px).clamp(2.2, 12.0)
    } else {
        (21.0_f32 / vp.zoom_m_per_px).clamp(3.0, 22.0)
    };
    for edge in &world.net.edges {
        if !edge.feature.is_physical_corridor()
            && edge.feature != RoadFeatureKind::RoundaboutRing
        {
            continue;
        }
        let step_m = (10.0 * vp.zoom_m_per_px).clamp(4.0, 26.0);
        let n_pts = ((edge.total_length / step_m).ceil() as usize).max(2);
        let pts: Vec<egui::Pos2> = (0..=n_pts)
            .map(|k| {
                let s = (k as f32 / n_pts as f32) * edge.total_length;
                let (wx, wz, _, _) = edge.sample_ref(s);
                to_canvas(vp, wx, wz)
            })
            .collect();
        painter.add(egui::Shape::line(
            pts.clone(),
            egui::Stroke::new(road_px_w, egui::Color32::from_rgb(48, 58, 74)),
        ));
        painter.add(egui::Shape::line(
            pts,
            egui::Stroke::new((road_px_w * 0.14).max(1.0), egui::Color32::from_rgb(138, 152, 172)),
        ));
    }

    if let Some(ref p2p) = world.p2p {
        let pts: Vec<egui::Pos2> = p2p.polyline.iter().map(|&(x, z)| to_canvas(vp, x, z)).collect();
        painter.add(egui::Shape::line(
            pts,
            egui::Stroke::new(5.0_f32, egui::Color32::from_rgb(16, 185, 129)),
        ));
        let gp = to_canvas(vp, p2p.goal_world.0, p2p.goal_world.1);
        painter.circle_filled(gp, 7.5, egui::Color32::from_rgb(16, 185, 129));
        painter.circle_stroke(gp, 12.0, egui::Stroke::new(2.0_f32, egui::Color32::WHITE));
    }

    if let Some(ref plan) = vp.preview {
        let pts: Vec<egui::Pos2> = plan.polyline.iter().map(|&(x, z)| to_canvas(vp, x, z)).collect();
        painter.add(egui::Shape::line(
            pts,
            egui::Stroke::new(5.5_f32, egui::Color32::from_rgb(56, 189, 248)),
        ));
        let gp = to_canvas(vp, plan.goal_world.0, plan.goal_world.1);
        let pulse = 11.0 + 3.0 * (now_s * 5.0).sin() as f32;
        painter.circle_stroke(gp, pulse, egui::Stroke::new(2.5_f32, egui::Color32::GOLD));
        painter.circle_filled(gp, 7.0, egui::Color32::RED);
        painter.text(
            gp + egui::vec2(12.0, -8.0),
            egui::Align2::LEFT_BOTTOM,
            "POINT B",
            egui::FontId::proportional(15.0),
            egui::Color32::GOLD,
        );
    }

    for (app, col, _) in world.traffic.iter_signals() {
        let c = to_canvas(vp, app.stop_x, app.stop_z);
        let rgb = match col {
            LightColor::Green => egui::Color32::from_rgb(34, 197, 94),
            LightColor::Yellow => egui::Color32::from_rgb(250, 204, 21),
            LightColor::Red => egui::Color32::from_rgb(239, 68, 68),
        };
        painter.circle_filled(c, 3.5, rgb);
    }
    for v in &world.traffic.vehicles {
        painter.circle_filled(to_canvas(vp, v.x, v.z), 2.8, egui::Color32::LIGHT_GRAY);
    }
    let ep = to_canvas(vp, world.ego.x, world.ego.z);
    let hx = world.ego.heading.sin();
    let hy = -world.ego.heading.cos();
    let tip = ep + egui::vec2(hx * 13.0, hy * 13.0);
    painter.circle_filled(ep, 6.5, egui::Color32::from_rgb(56, 189, 248));
    painter.line_segment([ep, tip], egui::Stroke::new(3.5_f32, egui::Color32::WHITE));
    painter.circle_stroke(ep, 12.0, egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(59, 130, 246)));
    painter.text(
        ep + egui::vec2(14.0, 2.0),
        egui::Align2::LEFT_CENTER,
        "POINT A (EGO)",
        egui::FontId::proportional(14.0),
        egui::Color32::LIGHT_BLUE,
    );

    ui.add_space(6.0);
    ui.horizontal(|ui| {
        if let Some(ref plan) = vp.preview {
            let eta_s = (plan.total_distance_m / 12.5).round() as u32;
            ui.label(
                egui::RichText::new(format!(
                    "ROUTE: {:.2} km ({:.0} m)  |  ETA {:02}:{:02}  |  {}",
                    plan.total_distance_m / 1000.0,
                    plan.total_distance_m,
                    eta_s / 60,
                    eta_s % 60,
                    plan.reason
                ))
                .size(15.0)
                .color(egui::Color32::GOLD),
            );
        } else if let Some(ref p2p) = world.p2p {
            ui.label(
                egui::RichText::new(format!(
                    "{}  |  Remaining {:.0} m  |  Click map to pick a new destination",
                    if p2p.arrived {
                        "ARRIVED AT POINT B"
                    } else {
                        "DRIVING POINT A -> B"
                    },
                    world.last_cue.dist_to_goal
                ))
                .size(15.0)
                .color(egui::Color32::from_rgb(110, 231, 183)),
            );
        } else {
            ui.label(
                egui::RichText::new("Click any road to preview Point B, Scroll to Zoom, Drag to Pan, ENTER or Double-Click to Drive")
                    .size(14.0)
                    .color(egui::Color32::from_rgb(148, 163, 184)),
            );
        }

        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui.button(" FREE CRUISE [C] ").clicked() {
                cruise_now = true;
            }
            let has_prev = vp.preview.is_some();
            if ui
                .add_enabled(has_prev, egui::Button::new(" DRIVE HERE [ENTER] "))
                .clicked()
            {
                commit_now = true;
            }
        });
    });

    if close_now {
        world.map_open = false;
    }
    if commit_now {
        if let Some(ref p) = vp.preview {
            world.set_destination(p.goal_world.0, p.goal_world.1);
            vp.preview = None;
            world.map_open = false;
        }
    }
    if cruise_now {
        world.resume_free_cruise();
        vp.preview = None;
        world.map_open = false;
    }
}
