//! Pure-Rust closed-loop simulation kernel unifying RoadNetwork, VehiclePhysics,
//! TrafficDirector, PerceptionEngine, QnxTcpClient, Point A->B navigation, and TelemetryRecorder.

use crate::config::{MapMode, SimConfig, DT_60HZ};
use crate::coordinate_mapper::project_relative;
use crate::perception::{PerceptionEngine, SignalHudInfo};
use crate::physics::VehiclePhysics;
use crate::qnx_client::{AdasOutput, QnxTcpClient, RttStats, TelemetryObstacle, TelemetryPacket};
use crate::recorder::TelemetryRecorder;
use crate::road_network::{MegaMapGenerator, PlannedRoute, RoadNetwork, RouteCue};
use crate::scenario_loader::{load_scenarios_from_dir, WaymoScenario};
use crate::traffic::{LightColor, TrafficDirector};
use std::path::Path;
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
pub struct ActiveP2P {
    pub goal_edge_id: usize,
    pub goal_s: f32,
    pub goal_world: (f32, f32),
    pub total_distance_m: f32,
    pub polyline: Vec<(f32, f32)>,
    pub reason: String,
    pub arrived: bool,
}

pub struct SimWorld {
    pub config: SimConfig,
    pub net: RoadNetwork,
    pub ego: VehiclePhysics,
    pub traffic: TrafficDirector,
    pub perception: PerceptionEngine,
    pub qnx: QnxTcpClient,
    pub recorder: TelemetryRecorder,

    pub active_edges: Vec<usize>,
    pub route_cursor: usize,
    pub current_s_on_edge: f32,
    pub p2p: Option<ActiveP2P>,

    pub waymo_scenarios: Vec<WaymoScenario>,
    pub waymo_idx: usize,
    pub waymo_time_s: f32,
    pub map_generation: u64,

    pub seq: u64,
    pub paused: bool,
    pub map_open: bool,
    pub last_adas: AdasOutput,
    pub last_cue: RouteCue,
    pub last_obs: Vec<TelemetryObstacle>,
    pub toast: Option<(String, Instant)>,

    stuck_timer_s: f32,
    pub false_aeb_count: u32,
    pub collision_count: u32,
    prev_collision: bool,
}

impl SimWorld {
    pub fn new(config: SimConfig, disable_random_events: bool) -> Self {
        let mut waymo_scenarios = Vec::new();
        let mut waymo_idx = 0usize;

        let (net, mut ego, active_edges, traffic, route_len_m) = match config.map_mode {
            MapMode::Waymo => {
                let loaded = load_scenarios_from_dir(Path::new(&config.scenarios_dir))
                    .unwrap_or_default();
                if loaded.is_empty() {
                    let net = MegaMapGenerator::generate(config.seed);
                    let start_eid = net.grand_tour_edges[0];
                    let p0 = net.edges[start_eid].sample_ref(15.0);
                    let ego = VehiclePhysics::new(p0.0, p0.1, p0.2);
                    let mut traffic = TrafficDirector::new(config.seed, config.traffic_density, &net);
                    traffic.disable_random_events = disable_random_events;
                    traffic.rebuild_for_network(config.seed, config.traffic_density, &net, (ego.x, ego.z));
                    let ae = net.grand_tour_edges.clone();
                    let rl = net.grand_tour_length_m;
                    (net, ego, ae, traffic, rl)
                } else {
                    waymo_idx = (config.seed as usize) % loaded.len();
                    let sc = &loaded[waymo_idx];
                    let net = RoadNetwork::from_waymo_scenario(sc);
                    let mut ego = VehiclePhysics::new(
                        sc.ego.start_pose[0],
                        sc.ego.start_pose[1],
                        sc.ego.start_pose[2],
                    );
                    ego.speed = sc.ego.start_pose[3];
                    let mut traffic = TrafficDirector::new(config.seed, 0.0, &net);
                    traffic.disable_random_events = true;
                    traffic.reset_for_waymo();
                    traffic.load_waymo_frame(sc, &net, 0.0);
                    let ae = net.grand_tour_edges.clone();
                    let rl = net.grand_tour_length_m;
                    waymo_scenarios = loaded;
                    (net, ego, ae, traffic, rl)
                }
            }
            MapMode::Mega10km => {
                let net = MegaMapGenerator::generate(config.seed);
                let start_eid = net.grand_tour_edges[0];
                let p0 = net.edges[start_eid].sample_ref(15.0);
                let ego = VehiclePhysics::new(p0.0, p0.1, p0.2);
                let mut traffic = TrafficDirector::new(config.seed, config.traffic_density, &net);
                traffic.disable_random_events = disable_random_events;
                traffic.rebuild_for_network(config.seed, config.traffic_density, &net, (ego.x, ego.z));
                let ae = net.grand_tour_edges.clone();
                let rl = net.grand_tour_length_m;
                (net, ego, ae, traffic, rl)
            }
        };

        let perception = PerceptionEngine::new(config.detection_radius_m, config.noise, config.seed);
        let mut qnx = QnxTcpClient::new(
            config.qnx_host.clone(),
            config.qnx_port,
            config.qnx_watchdog_timeout_ms,
        );
        let _ = qnx.connect();

        let recorder = TelemetryRecorder::new(config.seed, route_len_m as f64, 24000);
        let initial_s = if config.map_mode == MapMode::Waymo && !waymo_scenarios.is_empty() {
            0.0
        } else {
            15.0
        };
        let (init_cursor, cur_s, initial_cue) =
            net.compute_route_cue(ego.x, ego.z, ego.heading, &active_edges, 0, None);
        let _ = (init_cursor, &mut ego);

        Self {
            config,
            net,
            ego,
            traffic,
            perception,
            qnx,
            recorder,
            active_edges,
            route_cursor: 0,
            current_s_on_edge: if cur_s > 0.01 { cur_s } else { initial_s },
            p2p: None,
            waymo_scenarios,
            waymo_idx,
            waymo_time_s: 0.0,
            map_generation: 1,
            seq: 0,
            paused: false,
            map_open: false,
            last_adas: AdasOutput::default(),
            last_cue: initial_cue,
            last_obs: Vec::new(),
            toast: None,
            stuck_timer_s: 0.0,
            false_aeb_count: 0,
            collision_count: 0,
            prev_collision: false,
        }
    }

    pub fn waymo_scenario_id(&self) -> Option<&str> {
        if self.config.map_mode == MapMode::Waymo && !self.waymo_scenarios.is_empty() {
            Some(self.waymo_scenarios[self.waymo_idx].scenario_id.as_str())
        } else {
            None
        }
    }

    pub fn show_toast(&mut self, msg: impl Into<String>, secs: f32) {
        self.toast = Some((msg.into(), Instant::now() + Duration::from_secs_f32(secs)));
    }

    pub fn active_toast(&self) -> Option<&str> {
        if let Some((ref msg, until)) = self.toast {
            if Instant::now() < until {
                return Some(msg.as_str());
            }
        }
        None
    }

    /// Generate a brand-new randomized 10+ km map seed (or cycle next Waymo scene) cleanly ('G' or rollover).
    pub fn regenerate_new_map(&mut self, next_seed: u64, completed_previous: bool) {
        self.config.seed = next_seed;
        self.map_generation += 1;

        if self.config.map_mode == MapMode::Waymo && !self.waymo_scenarios.is_empty() {
            self.waymo_idx = (self.waymo_idx + 1) % self.waymo_scenarios.len();
            self.waymo_time_s = 0.0;
            let sc = &self.waymo_scenarios[self.waymo_idx];
            self.net = RoadNetwork::from_waymo_scenario(sc);
            self.ego.reset(sc.ego.start_pose[0], sc.ego.start_pose[1], sc.ego.start_pose[2]);
            self.ego.speed = sc.ego.start_pose[3];
            self.active_edges = self.net.grand_tour_edges.clone();
            self.route_cursor = 0;
            self.current_s_on_edge = 0.0;
            self.p2p = None;
            self.stuck_timer_s = 0.0;
            self.traffic.reset_for_waymo();
            self.traffic.load_waymo_frame(sc, &self.net, 0.0);
            self.perception.reseed(next_seed);
            self.recorder
                .on_new_map(next_seed, self.net.grand_tour_length_m as f64, completed_previous);
            let _ = self.qnx.connect();
            self.show_toast(
                format!("Waymo Scenario {}/{}: {}", self.waymo_idx + 1, self.waymo_scenarios.len(), sc.scenario_id),
                3.0,
            );
            return;
        }

        self.net = MegaMapGenerator::generate(next_seed);
        let start_eid = self.net.grand_tour_edges[0];
        let p0 = self.net.edges[start_eid].sample_ref(15.0);
        self.ego.reset(p0.0, p0.1, p0.2);

        self.active_edges = self.net.grand_tour_edges.clone();
        self.route_cursor = 0;
        self.current_s_on_edge = 15.0;
        self.p2p = None;
        self.stuck_timer_s = 0.0;

        self.traffic.rebuild_for_network(
            next_seed,
            self.config.traffic_density,
            &self.net,
            (self.ego.x, self.ego.z),
        );
        self.perception.reseed(next_seed);
        self.recorder
            .on_new_map(next_seed, self.net.grand_tour_length_m as f64, completed_previous);
        // Reconnect QNX socket so controller resets cleanly onto new route
        let _ = self.qnx.connect();
        self.show_toast(
            format!(
                "New Map Seed {} ({:.2} km tour)",
                next_seed,
                self.net.grand_tour_length_m / 1000.0
            ),
            3.0,
        );
    }

    /// Preview a Point A -> Point B shortest path without modifying the active driving route.
    pub fn preview_point_to_point(&self, world_x: f32, world_z: f32) -> Option<PlannedRoute> {
        let cur_eid = self.active_edges.get(self.route_cursor).copied()?;
        let snap = self.net.find_snap(world_x, world_z)?;
        if let Some(plan) =
            self.net
                .plan_a_star(cur_eid, self.current_s_on_edge, snap.edge_id, snap.s)
        {
            return Some(plan);
        }

        // Clean fallback if clicked road is unreachable from current edge
        let ahead_eid = *self
            .active_edges
            .get((self.route_cursor + 1).min(self.active_edges.len() - 1))?;
        self.net
            .plan_a_star(cur_eid, self.current_s_on_edge, ahead_eid, self.net.edges[ahead_eid].total_length * 0.5)
            .map(|mut p| {
                p.is_fallback = true;
                p.reason = "Fallback: routed to nearest forward junction".to_string();
                p
            })
    }

    /// Commit Point A -> Point B route seamlessly mid-drive (no environment reset!).
    pub fn set_destination(&mut self, world_x: f32, world_z: f32) -> Option<PlannedRoute> {
        let plan = self.preview_point_to_point(world_x, world_z)?;
        self.active_edges = plan.edge_ids.clone();
        self.route_cursor = 0;
        self.p2p = Some(ActiveP2P {
            goal_edge_id: plan.goal_edge_id,
            goal_s: plan.goal_s,
            goal_world: plan.goal_world,
            total_distance_m: plan.total_distance_m,
            polyline: plan.polyline.clone(),
            reason: plan.reason.clone(),
            arrived: false,
        });
        self.stuck_timer_s = 0.0;
        self.show_toast(
            format!("Route Set: {:.2} km to Point B", plan.total_distance_m / 1000.0),
            3.0,
        );
        Some(plan)
    }

    /// Clear Point B hold and seamlessly resume endless 10+ km free-cruise loop (or Waymo HD corridor) ('C' hotkey).
    pub fn resume_free_cruise(&mut self) {
        self.p2p = None;
        self.stuck_timer_s = 0.0;
        if self.config.map_mode == MapMode::Waymo && !self.waymo_scenarios.is_empty() {
            if let Some(snap) = self.net.find_snap(self.ego.x, self.ego.z) {
                let mut edges = vec![snap.edge_id];
                let mut cur_e = snap.edge_id;
                for _ in 0..8 {
                    let outs = &self.net.outgoing[self.net.edges[cur_e].to_node];
                    let Some(&nxt) = outs.iter().find(|&&id| !edges.contains(&id)) else {
                        break;
                    };
                    edges.push(nxt);
                    cur_e = nxt;
                }
                self.active_edges = edges;
                self.route_cursor = 0;
                self.current_s_on_edge = snap.s;
                self.show_toast("Resumed Waymo HD Cruise [C]", 2.8);
                return;
            }
        }
        // Build seamless continuation starting from ego's current edge + A* back onto Grand Tour loop
        if let Some(snap) = self.net.find_snap(self.ego.x, self.ego.z) {
            let tour_join_eid = self.net.grand_tour_edges[0];
            if let Some(bridge) = self.net.plan_a_star(snap.edge_id, snap.s, tour_join_eid, 5.0) {
                let mut edges = bridge.edge_ids;
                // Exclude duplicate join edge at end since tour starts with tour_join_eid
                if edges.last() == Some(&tour_join_eid) {
                    edges.pop();
                }
                edges.extend_from_slice(&self.net.grand_tour_edges);
                self.active_edges = edges;
                self.route_cursor = 0;
                self.current_s_on_edge = snap.s;
                self.show_toast("Resumed Continuous Free Cruise [C]", 2.8);
                return;
            }
        }
        let next_seed = self.config.seed + 1;
        self.regenerate_new_map(next_seed, false);
    }

    pub fn is_arrived_point_b(&self) -> bool {
        self.p2p.as_ref().map(|p| p.arrived).unwrap_or(false)
    }

    pub fn last_signal_hud(&self) -> Option<SignalHudInfo> {
        self.perception.last_signal_hud
    }

    pub fn rtt_stats(&self) -> RttStats {
        self.qnx.rtt_stats()
    }

    pub fn cycle_profile(&mut self) {
        self.config.profile = self.config.profile.cycle();
        self.show_toast(format!("Profile: {}", self.config.profile.label()), 2.2);
    }

    pub fn toggle_wet(&mut self) {
        self.config.toggle_wet();
        self.show_toast(
            format!(
                "Surface: {} (mu={:.2})",
                self.config.surface_label(),
                self.config.current_friction()
            ),
            2.4,
        );
    }

    pub fn toggle_noise(&mut self) {
        self.config.noise = !self.config.noise;
        self.perception.noise_enabled = self.config.noise;
        self.show_toast(
            format!("Radar Noise: {}", if self.config.noise { "ON" } else { "OFF" }),
            2.2,
        );
    }

    pub fn trigger_jaywalker(&mut self) {
        let msg = self.traffic.trigger_jaywalker(
            &self.net,
            (self.ego.x, self.ego.z),
            self.ego.heading,
            &self.active_edges,
            self.route_cursor,
            self.current_s_on_edge,
        );
        self.show_toast(msg, 2.8);
    }

    pub fn trigger_traffic_jam(&mut self) {
        let lane = (self.last_cue.current_lane.clamp(0, 2)) as usize;
        let msg = self.traffic.trigger_traffic_jam(
            &self.net,
            (self.ego.x, self.ego.z),
            &self.active_edges,
            self.route_cursor,
            self.current_s_on_edge,
            lane,
        );
        self.show_toast(msg, 2.8);
    }

    /// Execute one deterministic fixed-timestep 60 Hz simulation + QNX control tick.
    pub fn tick_fixed(&mut self, dt: f32) {
        if self.paused {
            return;
        }

        // Advance Waymo scenario clip OR procedural Tour loop seamlessly
        if self.config.map_mode == MapMode::Waymo && !self.waymo_scenarios.is_empty() {
            let dur = self.waymo_scenarios[self.waymo_idx].duration_s();
            if self.p2p.is_none() && self.waymo_time_s >= dur {
                let next_s = self.config.seed + 1;
                self.regenerate_new_map(next_s, true);
                return;
            }
        } else if self.p2p.is_none() && self.route_cursor + 4 >= self.active_edges.len() {
            let last_eid = *self.active_edges.last().unwrap();
            let join_eid = self.net.grand_tour_edges[0];
            if last_eid == join_eid {
                self.active_edges.pop();
            }
            self.active_edges.extend_from_slice(&self.net.grand_tour_edges);
        }

        let goal_s_tail = self.p2p.as_ref().map(|p| p.goal_s);
        let (new_cursor, s_on_edge, cue) = self.net.compute_route_cue(
            self.ego.x,
            self.ego.z,
            self.ego.heading,
            &self.active_edges,
            self.route_cursor,
            goal_s_tail,
        );
        self.route_cursor = new_cursor;
        self.current_s_on_edge = s_on_edge;
        self.last_cue = cue;

        // Check Point B arrival (<= 3.5 m longitudinal or Euclidean)
        if let Some(ref mut p2p) = self.p2p {
            let euclid = ((self.ego.x - p2p.goal_world.0).powi(2)
                + (self.ego.z - p2p.goal_world.1).powi(2))
            .sqrt();
            if !p2p.arrived && (cue.dist_to_goal <= 3.5 || euclid <= 3.5) {
                p2p.arrived = true;
                self.show_toast(
                    "ARRIVED AT POINT B — Press M (New Dest) or C (Free Cruise)",
                    5.0,
                );
            }
        }

        self.last_obs = self.perception.scan(
            &self.traffic,
            self.ego.x,
            self.ego.z,
            self.ego.heading,
            self.ego.speed,
        );

        self.seq += 1;
        let pkt = TelemetryPacket {
            x: self.ego.x,
            y: 0.0,
            z: self.ego.z,
            speed: self.ego.speed,
            heading: self.ego.heading,
            steer_angle: self.ego.steer_angle,
            current_lane: cue.current_lane,
            target_lane: cue.current_lane,
            has_route: cue.has_route,
            ref_x: cue.ref_x,
            ref_z: cue.ref_z,
            ref_heading: cue.ref_heading,
            path_curvature: cue.path_curvature,
            next_curvature: cue.next_curvature,
            dist_to_goal: cue.dist_to_goal,
            turn_signal_hint: cue.turn_signal_hint,
            dist_to_next_turn: cue.dist_to_next_turn,
            pre_turn_lane_hint: cue.pre_turn_lane_hint,
            in_intersection: cue.in_intersection,
            dt,
            seq: self.seq,
            drive_profile: self.config.profile.id(),
            obs: &self.last_obs,
        };

        let adas_out = self.qnx.send_telemetry(&pkt);

        // Track false AEB stops during normal flowing traffic (when no genuine close VRU/vehicle hazard exists)
        if adas_out.drive_mode == 5 && self.last_adas.drive_mode != 5 {
            let genuine_ped = self.traffic.pedestrians.iter().any(|p| {
                let (pf, pl) = project_relative(self.ego.x, self.ego.z, self.ego.heading, p.x, p.z);
                pf > 0.0 && pf < 21.0 && pl.abs() < 3.0
            }) || self.traffic.cyclists.iter().any(|c| {
                let (cf, cl) = project_relative(self.ego.x, self.ego.z, self.ego.heading, c.x, c.z);
                cf > 0.0 && cf < 21.0 && cl.abs() < 2.2
            });
            let genuine_veh = self.traffic.vehicles.iter().any(|v| {
                let (vf, vl) = project_relative(self.ego.x, self.ego.z, self.ego.heading, v.x, v.z);
                vf > 0.0 && vf < 20.0 && vl.abs() < 1.65 && (vf < 14.0 || (self.ego.speed - v.speed * (v.heading - self.ego.heading).cos()) > 3.5)
            });
            if !genuine_ped && !genuine_veh {
                self.false_aeb_count += 1;
            }
        }
        self.last_adas = adas_out.clone();

        // Advance traffic (Waymo HD replay OR procedural IDM+MOBIL) and ego bicycle dynamics
        if self.config.map_mode == MapMode::Waymo && !self.waymo_scenarios.is_empty() {
            self.waymo_time_s += dt;
            let sc = &self.waymo_scenarios[self.waymo_idx];
            self.traffic.load_waymo_frame(sc, &self.net, self.waymo_time_s);
            self.traffic.step_waymo_addons((self.ego.x, self.ego.z), dt);
        } else {
            self.traffic.step(
                &self.net,
                (self.ego.x, self.ego.z),
                self.ego.heading,
                self.ego.speed,
                &self.active_edges,
                self.route_cursor,
                self.current_s_on_edge,
                dt,
            );
        }

        let arrived = self.is_arrived_point_b();
        let (cmd_steer, cmd_thr, cmd_brk) = if arrived {
            (0.0, 0.0, 0.85)
        } else {
            (adas_out.steer_cmd, adas_out.throttle, adas_out.brake)
        };
        let ds = self.ego.step(
            cmd_steer,
            cmd_thr,
            cmd_brk,
            self.config.current_friction(),
            dt,
        );

        // Check actual physical collisions with vehicles/pedestrians
        let mut collision = false;
        for v in &self.traffic.vehicles {
            let (fwd, lat) = project_relative(self.ego.x, self.ego.z, self.ego.heading, v.x, v.z);
            if fwd.abs() < (2.25 + v.class.length_m() * 0.48)
                && lat.abs() < (0.95 + v.class.width_m() * 0.45)
            {
                collision = true;
                break;
            }
        }
        if !collision {
            for p in &self.traffic.pedestrians {
                let (fwd, lat) = project_relative(self.ego.x, self.ego.z, self.ego.heading, p.x, p.z);
                if fwd.abs() < 2.35 && lat.abs() < 1.15 {
                    collision = true;
                    break;
                }
            }
        }
        if !collision {
            for cyc in &self.traffic.cyclists {
                let (fwd, lat) = project_relative(self.ego.x, self.ego.z, self.ego.heading, cyc.x, cyc.z);
                if fwd.abs() < 2.6 && lat.abs() < 1.25 {
                    collision = true;
                    break;
                }
            }
        }
        if collision && !self.prev_collision {
            self.collision_count += 1;
        }
        self.prev_collision = collision;

        // Lateral error relative to target lane center
        let (_, raw_lat, _) = self.net.edges[self.active_edges[self.route_cursor]]
            .project_world(self.ego.x, self.ego.z);
        let lane_center_lat = (cue.current_lane as f32 - 1.0) * 3.5;
        let lat_err = raw_lat - lane_center_lat;
        let lc_active = adas_out.drive_mode == 3
            || adas_out.left_turn_signal
            || adas_out.right_turn_signal;

        self.recorder.record_step(
            self.seq,
            dt,
            ds,
            self.ego.speed,
            lat_err,
            lc_active,
            &adas_out,
            collision,
        );

        // Soft stuck recovery (> 5s only if neither stopped at red/traffic/Point B)
        let legal_wait = arrived
            || (adas_out.drive_mode == 6)
            || (adas_out.drive_mode == 2 && adas_out.lead_distance < 35.0)
            || self
                .perception
                .last_signal_hud
                .map(|s| s.color != LightColor::Green && s.distance_m < 45.0)
                .unwrap_or(false);
        if (collision || self.ego.speed < 0.25) && !legal_wait {
            self.stuck_timer_s += dt;
            if self.stuck_timer_s >= 5.0 {
                let next_s = self.config.seed + 1;
                self.regenerate_new_map(next_s, false);
            }
        } else {
            self.stuck_timer_s = 0.0;
        }
    }

    pub fn dt_default() -> f32 {
        DT_60HZ
    }
}
