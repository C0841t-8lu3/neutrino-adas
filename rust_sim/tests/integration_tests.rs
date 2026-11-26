//! Comprehensive unit, integration, pro-driver QNX accuracy, and performance tests.

use qnx_rust_sim::config::{MapMode, SimConfig, DT_60HZ, MU_DRY, MU_WET, WHEELBASE_M};
use qnx_rust_sim::coordinate_mapper::{
    curved_corridor_offset, is_in_bsm_zone, normalize_angle_rad, project_relative,
};
use qnx_rust_sim::map_overlay::MapViewport;
use qnx_rust_sim::physics::VehiclePhysics;
use qnx_rust_sim::road_network::{MegaMapGenerator, RoadNetwork};
use qnx_rust_sim::scenario_loader::load_scenarios_from_dir;
use std::path::Path;
use qnx_rust_sim::simulation::SimWorld;
use qnx_rust_sim::traffic::LightColor;
use std::sync::Mutex;
use std::time::Instant;

/// Serialize live TCP tests against QNX VM (192.168.122.62:9090) so parallel Cargo workers
/// do not interleave stateful ADAS sessions on port 9090.
static QNX_LOCK: Mutex<()> = Mutex::new(());

#[test]
fn test_mega_map_length_g1_continuity_and_non_overlap() {
    for seed in [1000u64, 1001, 1002] {
        let net = MegaMapGenerator::generate(seed);
        assert!(
            net.grand_tour_length_m >= 10_000.0,
            "Seed {} tour length {:.1}m < 10,000m",
            seed,
            net.grand_tour_length_m
        );
        assert!(
            net.unique_corridor_length_m >= 10_000.0,
            "Seed {} unique corridor {:.1}m < 10,000m",
            seed,
            net.unique_corridor_length_m
        );
        assert!(net.grand_tour_edges.len() >= 12);

        // Verify G1 continuity (position + heading) across every edge transition in the tour
        for i in 0..net.grand_tour_edges.len() {
            let ea = &net.edges[net.grand_tour_edges[i]];
            let eb = &net.edges[net.grand_tour_edges[(i + 1) % net.grand_tour_edges.len()]];
            assert_eq!(
                ea.to_node, eb.from_node,
                "Seed {} tour edge {} -> {} disconnected graph node",
                seed, ea.id, eb.id
            );
            let (ax, az, ah, _) = ea.sample_ref(ea.total_length);
            let (bx, bz, bh, _) = eb.sample_ref(0.0);
            let gap = ((ax - bx).powi(2) + (az - bz).powi(2)).sqrt();
            let d_h = normalize_angle_rad(ah - bh).abs();
            assert!(
                gap <= 0.35,
                "Seed {} tour step {} (edge {}->{}) position gap {:.3}m > 0.35m",
                seed,
                i,
                ea.id,
                eb.id,
                gap
            );
            assert!(
                d_h <= 0.10,
                "Seed {} tour step {} (edge {}->{}) heading jump {:.3} rad",
                seed,
                i,
                ea.id,
                eb.id,
                d_h
            );
        }

        // Verify distinct non-adjacent physical corridors never cross/overlap unintentionally
        let phys: Vec<usize> = net
            .edges
            .iter()
            .filter(|e| e.feature.is_physical_corridor())
            .map(|e| e.id)
            .collect();
        for &id_a in &phys {
            let ea = &net.edges[id_a];
            for &id_b in &phys {
                if id_b <= id_a {
                    continue;
                }
                let eb = &net.edges[id_b];
                // Skip opposing carriageway / shared junction edges
                if ea.from_node == eb.to_node
                    || ea.to_node == eb.from_node
                    || ea.from_node == eb.from_node
                    || ea.to_node == eb.to_node
                {
                    continue;
                }
                let samples_a = (ea.total_length / 25.0).ceil() as usize;
                let samples_b = (eb.total_length / 25.0).ceil() as usize;
                for ka in 1..samples_a {
                    let (ax, az, ah, _) = ea.sample_ref((ka as f32 / samples_a as f32) * ea.total_length);
                    if net
                        .junctions
                        .iter()
                        .any(|j| ((ax - j.center_x).powi(2) + (az - j.center_z).powi(2)).sqrt() < 55.0)
                    {
                        continue;
                    }
                    for kb in 1..samples_b {
                        let (bx, bz, bh, _) =
                            eb.sample_ref((kb as f32 / samples_b as f32) * eb.total_length);
                        let d = ((ax - bx).powi(2) + (az - bz).powi(2)).sqrt();
                        // Divided two-way corridors place opposite carriageways at 10.50m (5.25+5.25) with opposite headings;
                        // flag any closer overlap (< 9.5m) or non-opposing crossing (< 18m).
                        let opposite_lanes = normalize_angle_rad(ah - bh).abs() > 2.80;
                        let legal_divided_median = opposite_lanes && d >= 9.5;
                        assert!(
                            d > 18.0 || legal_divided_median,
                            "Seed {} edges {} and {} mid-corridor overlap d={:.2}m (opposite={}) at ({:.1},{:.1})",
                            seed,
                            id_a,
                            id_b,
                            d,
                            opposite_lanes,
                            ax,
                            az
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn test_coordinate_curvature_and_bsm_parity() {
    let (fwd, lat) = project_relative(0.0, 0.0, 0.0, 12.0, 30.0);
    assert!((fwd - 30.0).abs() < 1e-4);
    assert!((lat - 12.0).abs() < 1e-4);

    // Facing East (+X -> heading = +PI/2): target (+20, -5) is 20m forward and +5m to the right
    let (fwd_e, lat_e) =
        project_relative(0.0, 0.0, std::f32::consts::FRAC_PI_2, 20.0, -5.0);
    assert!((fwd_e - 20.0).abs() < 1e-3);
    assert!((lat_e - 5.0).abs() < 1e-3);

    // Curved corridor offset matching ForwardRadar.cpp
    let off = curved_corridor_offset(0.01, 20.0);
    assert!((off - 1.99334).abs() < 0.02);

    assert_eq!(is_in_bsm_zone(-4.0, -3.5), (true, false));
    assert_eq!(is_in_bsm_zone(-4.0, 3.5), (false, true));
    assert_eq!(is_in_bsm_zone(10.0, 3.5), (false, false));
}

#[test]
fn test_physics_bicycle_circle_and_wet_braking() {
    let mut veh = VehiclePhysics::new(0.0, 0.0, 0.0);
    veh.speed = 10.0;
    let r_target = 40.0f32;
    let steer = (WHEELBASE_M / r_target).atan();
    for _ in 0..300 {
        veh.step(steer, 0.20, 0.0, MU_DRY, DT_60HZ);
    }
    assert!(veh.speed > 5.0 && veh.x > 5.0, "Vehicle should turn right cleanly");

    let mut dry = VehiclePhysics::new(0.0, 0.0, 0.0);
    dry.speed = 16.0;
    let mut d_dry = 0.0f32;
    while dry.speed > 0.05 && d_dry < 200.0 {
        d_dry += dry.step(0.0, 0.0, 1.0, MU_DRY, DT_60HZ);
    }

    let mut wet = VehiclePhysics::new(0.0, 0.0, 0.0);
    wet.speed = 16.0;
    let mut d_wet = 0.0f32;
    while wet.speed > 0.05 && d_wet < 200.0 {
        d_wet += wet.step(0.0, 0.0, 1.0, MU_WET, DT_60HZ);
    }

    assert!(
        d_wet > d_dry * 1.25,
        "Wet stopping distance {:.2}m should be substantially longer than dry {:.2}m",
        d_wet,
        d_dry
    );
}

#[test]
fn test_interactive_map_viewport_and_astar_routing() {
    let net = MegaMapGenerator::generate(1000);
    let mut vp = MapViewport::fit_network(&net, 1280.0, 800.0);

    let (sx, sy) = vp.world_to_screen(120.0, -340.0);
    let (wx, wz) = vp.screen_to_world(sx, sy);
    assert!((wx - 120.0).abs() < 1e-2 && (wz + 340.0).abs() < 1e-2);

    let before_z = vp.screen_to_world(640.0, 400.0);
    vp.zoom_at(640.0, 400.0, 0.5);
    let after_z = vp.screen_to_world(640.0, 400.0);
    assert!((before_z.0 - after_z.0).abs() < 1e-2 && (before_z.1 - after_z.1).abs() < 1e-2);

    // A* routing between non-trivial distant edges on tour
    let start_e = net.grand_tour_edges[0];
    let goal_e = net.grand_tour_edges[net.grand_tour_edges.len() / 2];
    let goal_s = net.edges[goal_e].total_length * 0.5;
    let plan = net
        .plan_a_star(start_e, 15.0, goal_e, goal_s)
        .expect("A* must find path between grand tour edges");
    assert!(plan.total_distance_m > 500.0);
    assert!(plan.polyline.len() >= 20);
    for w in plan.edge_ids.windows(2) {
        assert_eq!(net.edges[w[0]].to_node, net.edges[w[1]].from_node);
    }
}

#[test]
fn test_qnx_pro_driver_cruise_accuracy_and_perf() {
    let _guard = QNX_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut cfg = SimConfig::default();
    cfg.seed = 1000;
    cfg.traffic_density = 0.20;
    let mut world = SimWorld::new(cfg, true);

    assert!(
        world.qnx.connected,
        "QNX 8.0 server must be reachable on 192.168.122.62:9090"
    );

    // Let vehicle launch and cruise 2,400 ticks (~40s across straight & intersection turns)
    let mut rust_and_net_ns = 0u128;
    let total_ticks = 2400usize;
    for _ in 0..total_ticks {
        let t0 = Instant::now();
        world.tick_fixed(DT_60HZ);
        rust_and_net_ns += t0.elapsed().as_nanos();
    }

    let rtt = world.rtt_stats();
    let sum = world.recorder.summary(rtt);
    println!(
        "[PRO-DRIVER CRUISE] odom={:.1}m rms_lat={:.4}m max_lat={:.4}m collisions={} false_aeb={} wd_trips={} p50={:.0}us avg_loop={:.3}ms",
        sum.odometer_km * 1000.0,
        sum.rms_lateral_error_m,
        sum.max_abs_lateral_error_m,
        world.collision_count,
        world.false_aeb_count,
        sum.watchdog_trips,
        sum.qnx_rtt_p50_us,
        (rust_and_net_ns as f64 / total_ticks as f64) / 1e6
    );

    assert_eq!(sum.watchdog_trips, 0, "Watchdog tripped during cruise");
    assert_eq!(world.collision_count, 0, "Unexpected collision during cruise");
    assert_eq!(
        world.false_aeb_count, 0,
        "Unexpected false AEB stop during flowing cruise"
    );
    assert!(
        sum.odometer_km * 1000.0 > 220.0,
        "Ego barely moved: {:.1}m",
        sum.odometer_km * 1000.0
    );
    assert!(
        sum.rms_lateral_error_m < 0.15,
        "RMS lateral error {:.4}m exceeded 0.15m Pro-Driver budget!",
        sum.rms_lateral_error_m
    );

    // Also explicitly test Ackermann feedforward cornering through a full Sweeping Highway S-Arch
    world.regenerate_new_map(1000, false);
    if let Some((idx, &arch_eid)) = world
        .net
        .grand_tour_edges
        .iter()
        .enumerate()
        .find(|(_, &eid)| world.net.edges[eid].feature == qnx_rust_sim::road_network::RoadFeatureKind::CurveArch)
    {
        let p = world.net.edges[arch_eid].sample_ref(20.0);
        world.ego.reset(p.0, p.1, p.2);
        world.ego.speed = 12.0;
        world.route_cursor = idx;
        world.current_s_on_edge = 20.0;
        world.recorder = qnx_rust_sim::recorder::TelemetryRecorder::new(1000, world.net.grand_tour_length_m as f64, 8000);
        for _ in 0..3000 {
            world.tick_fixed(DT_60HZ);
        }
        let s2 = world.recorder.summary(world.rtt_stats());
        println!(
            "[PRO-DRIVER S-ARCH] odom={:.1}m rms_lat={:.4}m max_lat={:.4}m collisions={} false_aeb={}",
            s2.odometer_km * 1000.0, s2.rms_lateral_error_m, s2.max_abs_lateral_error_m,
            world.collision_count, world.false_aeb_count
        );
        assert_eq!(world.collision_count, 0, "Collision on sweeping S-arch!");
        assert_eq!(world.false_aeb_count, 0, "False AEB on sweeping S-arch!");
        assert!(s2.odometer_km * 1000.0 > 450.0, "Should traverse S-arch cleanly: {:.1}m", s2.odometer_km * 1000.0);
        assert!(
            s2.rms_lateral_error_m < 0.15,
            "S-Arch RMS lateral error {:.4}m exceeded 0.15m Pro-Driver target!",
            s2.rms_lateral_error_m
        );
    }
}

#[test]
fn test_qnx_smooth_acc_following_red_light_jaywalker_and_point_b() {
    let _guard = QNX_LOCK.lock().unwrap_or_else(|e| e.into_inner());

    // 1. Smooth IDM ACC following behind slow moving lead vehicle (no yo-yo, gap > 12m)
    {
        let mut cfg = SimConfig::default();
        cfg.seed = 1000;
        cfg.traffic_density = 0.0;
        let mut world = SimWorld::new(cfg, true);
        // Accelerate cleanly first
        for _ in 0..240 {
            world.tick_fixed(DT_60HZ);
        }
        world.trigger_traffic_jam();
        assert!(
            !world.traffic.vehicles.is_empty(),
            "Expected traffic jam vehicles ahead"
        );
        // Let slow lead vehicles flow steadily ~7.5 m/s so ACC settles behind them
        for v in &mut world.traffic.vehicles {
            v.speed = 7.5;
            v.desired_speed = 7.5;
            v.hold_timer_s = 0.0;
        }
        let mut min_gap_settled = f32::MAX;
        let mut min_speed_kmh_settled = f32::MAX;
        for step in 0..480 {
            world.tick_fixed(DT_60HZ);
            if step >= 180 && world.last_adas.lead_distance < 80.0 {
                min_gap_settled = min_gap_settled.min(world.last_adas.lead_distance);
                min_speed_kmh_settled = min_speed_kmh_settled.min(world.ego.speed * 3.6);
            }
        }
        println!(
            "[ACC SMOOTH] min_gap={:.2}m min_speed={:.1}km/h collisions={}",
            min_gap_settled, min_speed_kmh_settled, world.collision_count
        );
        assert_eq!(world.collision_count, 0);
        assert!(
            min_gap_settled > 12.0,
            "ACC bumper gap {:.2}m too tight / tailgating (<12m)",
            min_gap_settled
        );
        assert!(
            min_speed_kmh_settled > 10.0,
            "ACC yo-yo brake check dropped speed to {:.1} km/h (<10 km/h) behind moving traffic",
            min_speed_kmh_settled
        );
    }

    // 2. Red-Light Stop [0.5, 2.5] m behind stop bar & clean Green Resume
    {
        let mut cfg = SimConfig::default();
        cfg.seed = 1000;
        cfg.traffic_density = 0.0;
        let mut world = SimWorld::new(cfg, true);
        // Position ego 48m ahead of the first traffic-signal intersection approach
        let first_eid = world.net.grand_tour_edges[0];
        let e_len = world.net.edges[first_eid].total_length;
        let s_pos = (e_len - 48.0).max(5.0);
        let p = world.net.edges[first_eid].sample_ref(s_pos);
        world.ego.reset(p.0, p.1, p.2);
        world.ego.speed = 11.5;
        world.current_s_on_edge = s_pos;

        assert!(world
            .traffic
            .force_upcoming_signal(&world.active_edges, world.route_cursor, LightColor::Red, 10.0));

        for _ in 0..420 {
            world.tick_fixed(DT_60HZ);
        }
        let stop_gap = world.last_signal_hud().map(|s| s.distance_m).unwrap_or(999.0);
        let bumper_gap = stop_gap - 2.2;
        println!(
            "[RED LIGHT] stopped speed={:.2}m/s center_to_bar={:.2}m bumper_gap={:.2}m",
            world.ego.speed, stop_gap, bumper_gap
        );
        assert!(
            world.ego.speed < 0.35,
            "Vehicle failed to stop for red light: speed={:.2}m/s",
            world.ego.speed
        );
        assert!(
            (0.5..=2.5).contains(&bumper_gap),
            "Front bumper stop distance {:.2}m outside [0.5, 2.5] m behind stop bar",
            bumper_gap
        );

        assert!(world.traffic.force_upcoming_signal(
            &world.active_edges,
            world.route_cursor,
            LightColor::Green,
            10.0
        ));
        for _ in 0..240 {
            world.tick_fixed(DT_60HZ);
        }
        println!("[GREEN RESUME] speed after green={:.2}m/s", world.ego.speed);
        assert!(
            world.ego.speed > 5.0,
            "Vehicle failed to resume on green light: speed={:.2}m/s",
            world.ego.speed
        );
    }

    // 3. Sudden Jaywalker AEB Stop (0 collisions)
    {
        let mut cfg = SimConfig::default();
        cfg.seed = 1000;
        cfg.traffic_density = 0.0;
        let mut world = SimWorld::new(cfg, true);
        for _ in 0..220 {
            world.tick_fixed(DT_60HZ);
        }
        world.trigger_jaywalker();
        let mut saw_hard_brake = false;
        for _ in 0..260 {
            world.tick_fixed(DT_60HZ);
            if world.last_adas.drive_mode == 5 || world.last_adas.brake > 0.65 {
                saw_hard_brake = true;
            }
        }
        println!(
            "[JAYWALKER AEB] saw_hard_brake={} collisions={} aeb_events={}",
            saw_hard_brake, world.collision_count, world.recorder.aeb_events
        );
        assert_eq!(world.collision_count, 0, "Hit spawned jaywalker!");
        assert!(saw_hard_brake, "QNX failed to brake hard for crossing jaywalker");
    }

    // 4. Interactive Point A -> Point B arrival & Free Cruise [C] resume without closing
    {
        let mut cfg = SimConfig::default();
        cfg.seed = 1000;
        cfg.traffic_density = 0.0;
        let mut world = SimWorld::new(cfg, true);
        let first_eid = world.net.grand_tour_edges[0];
        let dest = world.net.edges[first_eid].sample_ref(145.0);
        let plan = world
            .set_destination(dest.0, dest.1)
            .expect("Point B snap must succeed");
        assert!(plan.total_distance_m > 80.0);

        for _ in 0..950 {
            world.tick_fixed(DT_60HZ);
            if world.is_arrived_point_b() {
                break;
            }
        }
        assert!(world.is_arrived_point_b(), "Did not arrive cleanly at Point B");
        for _ in 0..30 {
            world.tick_fixed(DT_60HZ);
        }
        assert!(world.ego.speed < 0.25, "Did not hold stopped at Point B");

        world.resume_free_cruise();
        assert!(!world.is_arrived_point_b());
        for _ in 0..200 {
            world.tick_fixed(DT_60HZ);
        }
        assert!(
            world.ego.speed > 4.0,
            "Failed to resume cruising after [C]: speed={:.2}m/s",
            world.ego.speed
        );
    }
}

#[test]
fn test_pure_rust_simulation_tick_latency_budget() {
    // Measure non-GL simulation step cost cleanly (without sleeping for GL swap or blocking network)
    let net = MegaMapGenerator::generate(1000);
    let mut cfg = SimConfig::default();
    cfg.seed = 1000;
    cfg.traffic_density = 0.20;
    let mut traffic = qnx_rust_sim::traffic::TrafficDirector::new(cfg.seed, cfg.traffic_density, &net);
    let mut perc = qnx_rust_sim::perception::PerceptionEngine::new(85.0, false, cfg.seed);
    let mut ego = VehiclePhysics::new(net.edges[net.grand_tour_edges[0]].sample_ref(15.0).0, net.edges[net.grand_tour_edges[0]].sample_ref(15.0).1, net.edges[net.grand_tour_edges[0]].sample_ref(15.0).2);
    let mut cursor = 0usize;

    let t0 = Instant::now();
    for _ in 0..1000 {
        let (nc, s_on, _cue) =
            net.compute_route_cue(ego.x, ego.z, ego.heading, &net.grand_tour_edges, cursor, None);
        cursor = nc;
        let _obs = perc.scan(&traffic, ego.x, ego.z, ego.heading, ego.speed);
        traffic.step(&net, (ego.x, ego.z), ego.heading, ego.speed, &net.grand_tour_edges, cursor, s_on, DT_60HZ);
        ego.step(0.0, 0.3, 0.0, MU_DRY, DT_60HZ);
    }
    let avg_ms = (t0.elapsed().as_secs_f64() * 1000.0) / 1000.0;
    println!("[PERF] 1,000-tick pure Rust sim step avg = {:.4} ms/tick", avg_ms);
    assert!(
        avg_ms < 0.50,
        "Pure Rust simulation step {:.4} ms exceeds 0.50 ms budget",
        avg_ms
    );
}

#[test]
fn test_extracted_assets_and_waymo_scenario_loading() {
    let required_gltfs = [
        "assets/models/ferra/vehicle.gltf",
        "assets/models/ferra/right_tire_front.gltf",
        "assets/models/ferra/right_tire_back.gltf",
        "assets/models/130/vehicle.gltf",
        "assets/models/lada/vehicle.gltf",
        "assets/models/truck/vehicle.gltf",
        "assets/models/pedestrian/scene.gltf",
        "assets/models/bicycle/scene.gltf",
        "assets/models/traffic_cone/scene.gltf",
        "assets/models/traffic_light/red.gltf",
        "assets/models/traffic_light/yellow.gltf",
        "assets/models/traffic_light/green.gltf",
    ];
    for gltf_rel in required_gltfs {
        let p = Path::new(gltf_rel);
        assert!(p.is_file(), "Missing extracted glTF asset: {}", gltf_rel);
        let dir = p.parent().unwrap();
        let txt = std::fs::read_to_string(p).unwrap();
        let val: serde_json::Value = serde_json::from_str(&txt).unwrap();
        for key in ["buffers", "images"] {
            if let Some(arr) = val.get(key).and_then(|v| v.as_array()) {
                for item in arr {
                    if let Some(uri) = item.get("uri").and_then(|u| u.as_str()) {
                        if !uri.is_empty() && !uri.starts_with("data:") {
                            let target = dir.join(uri);
                            assert!(
                                target.is_file(),
                                "{} references missing URI {}",
                                gltf_rel,
                                target.display()
                            );
                        }
                    }
                }
            }
        }
    }

    for tex in [
        "assets/textures/asphalt/diff_2k.png",
        "assets/textures/grass2/color.jpg",
        "assets/textures/sidewalk/color.png",
    ] {
        assert!(Path::new(tex).is_file(), "Missing extracted texture: {}", tex);
    }

    let scenarios = load_scenarios_from_dir(Path::new("assets/scenarios"))
        .expect("Failed reading assets/scenarios JSON files");
    assert!(
        scenarios.len() >= 3,
        "Expected >= 3 exported Waymo ScenarioNet clips, got {}",
        scenarios.len()
    );

    for sc in &scenarios {
        assert!(!sc.lanes.is_empty(), "{} missing lanes", sc.scenario_id);
        assert!(!sc.tracks.is_empty(), "{} missing tracks", sc.scenario_id);
        assert!(!sc.traffic_lights.is_empty(), "{} missing traffic lights", sc.scenario_id);
        let net = RoadNetwork::from_waymo_scenario(sc);
        assert!(net.is_single_lane_hd);
        assert!(net.edges.len() >= sc.lanes.len().min(10));
        assert!(net.grand_tour_length_m > 20.0);
        let snap = net.find_snap(sc.ego.goal_pose[0], sc.ego.goal_pose[1]);
        assert!(snap.is_some(), "Waymo {} goal snap failed", sc.scenario_id);
    }
}

#[test]
fn test_qnx_waymo_live_closed_loop_and_rollover() {
    let _guard = QNX_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut cfg = SimConfig::default();
    cfg.map_mode = MapMode::Waymo;
    cfg.seed = 0; // Start on first Waymo clip cleanly
    let mut world = SimWorld::new(cfg, true);

    assert!(world.qnx.connected);
    let first_id = world
        .waymo_scenario_id()
        .expect("Waymo scenario must be loaded")
        .to_string();

    // Run 720 ticks (~12.0s) so clip (~9.0s) completes and rolls seamlessly into scenario #2
    for _ in 0..720 {
        world.tick_fixed(DT_60HZ);
    }

    let second_id = world
        .waymo_scenario_id()
        .expect("Waymo scenario must still be active after rollover")
        .to_string();
    let rtt = world.rtt_stats();
    let sum = world.recorder.summary(rtt);
    println!(
        "[WAYMO LIVE] {} -> {} rollovers={} odom={:.1}m rms_lat={:.4}m collisions={} false_aeb={} wd_trips={}",
        first_id,
        second_id,
        world.recorder.rollovers,
        sum.odometer_km * 1000.0,
        sum.rms_lateral_error_m,
        world.collision_count,
        world.false_aeb_count,
        sum.watchdog_trips
    );

    assert_ne!(
        first_id, second_id,
        "Waymo mode must roll over seamlessly onto next clip without closing"
    );
    assert_eq!(sum.watchdog_trips, 0);
    assert_eq!(world.collision_count, 0, "Collision during live Waymo clip");
    assert_eq!(world.false_aeb_count, 0, "False AEB panic during Waymo clip");
    assert!(
        sum.odometer_km * 1000.0 > 45.0,
        "Waymo ego barely moved: {:.1}m",
        sum.odometer_km * 1000.0
    );
    assert!(
        sum.rms_lateral_error_m < 0.15,
        "Waymo RMS lateral error {:.4}m exceeded 0.15m",
        sum.rms_lateral_error_m
    );
}

#[test]
fn test_bevy_scene_ecs_and_asset_resolution() {
    use bevy::prelude::*;
    use qnx_rust_sim::config::ViewMode;
    use qnx_rust_sim::render_setup::{resolve_assets_dir, MainDriveCamera, SimWorldRes};
    use qnx_rust_sim::scene_3d::Scene3dPlugin;

    let assets_dir = resolve_assets_dir();
    let assets_path = Path::new(&assets_dir);
    assert!(
        assets_path.is_absolute(),
        "resolve_assets_dir must return an absolute path so binaries outside repo root work: {assets_dir}"
    );
    assert!(assets_path.join("models/ferra/vehicle.gltf").is_file());
    assert!(assets_path.join("textures/asphalt/diff_2k.png").is_file());

    let mut cfg = SimConfig::default();
    cfg.qnx_host = "127.0.0.1".to_string();
    cfg.qnx_port = 19999; // Fast fail-stop without touching QNX lock
    let world = SimWorld::new(cfg, true);

    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins,
        AssetPlugin {
            file_path: assets_dir,
            ..default()
        },
        bevy::scene::ScenePlugin,
    ))
    .init_asset::<Image>()
    .init_asset::<Mesh>()
    .init_asset::<StandardMaterial>()
    .init_asset::<bevy::gltf::Gltf>()
    .insert_resource(SimWorldRes(world))
    .add_plugins(Scene3dPlugin);

    // Spawn MainDriveCamera needed by update_camera_system
    app.world_mut()
        .spawn((MainDriveCamera, Transform::IDENTITY));

    // Step across 3D Mega10km, 2D Mega10km, and Waymo rollover to validate every ECS schedule & query
    app.update();
    app.world_mut().resource_mut::<SimWorldRes>().0.config.view = ViewMode::View2D;
    app.update();
    {
        let mut sw = app.world_mut().resource_mut::<SimWorldRes>();
        sw.0.config.map_mode = MapMode::Waymo;
        sw.0.regenerate_new_map(0, false);
    }
    app.update();
}
