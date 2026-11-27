//! Unpredictable IDM + MOBIL NPC traffic, cycling intersection traffic signals,
//! crosswalk & jaywalking pedestrians, and interactive jam/hazard injectors.

use crate::config::LANE_WIDTH_M;
use crate::coordinate_mapper::{normalize_angle_rad, project_relative, right_vec, DeterministicRng};
use crate::road_network::{JunctionType, RoadNetwork};
use crate::scenario_loader::WaymoScenario;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LightColor {
    Green,
    Yellow,
    Red,
}

impl LightColor {
    pub fn label(self) -> &'static str {
        match self {
            LightColor::Green => "GREEN",
            LightColor::Yellow => "YELLOW",
            LightColor::Red => "RED",
        }
    }
}

#[derive(Debug, Clone)]
pub struct SignalApproach {
    pub edge_id: usize,
    pub is_ns: bool,
    pub stop_x: f32,
    pub stop_z: f32,
    pub heading: f32,
}

#[derive(Debug, Clone)]
pub struct IntersectionSignal {
    pub junction_id: usize,
    pub center_x: f32,
    pub center_z: f32,
    pub phase_s: f32,
    pub forced_ns: Option<(LightColor, f32)>,
    pub approaches: Vec<SignalApproach>,
}

const GREEN_DUR: f32 = 12.0;
const YELLOW_DUR: f32 = 3.0;
const ALL_RED_GAP: f32 = 1.5;
const HALF_CYCLE: f32 = GREEN_DUR + YELLOW_DUR + ALL_RED_GAP;
const FULL_CYCLE: f32 = 2.0 * HALF_CYCLE;

impl IntersectionSignal {
    pub fn step(&mut self, dt: f32) {
        if let Some((_, ref mut rem)) = self.forced_ns {
            *rem -= dt;
            if *rem <= 0.0 {
                self.forced_ns = None;
            }
        }
        self.phase_s = (self.phase_s + dt) % FULL_CYCLE;
    }

    pub fn color_for_approach(&self, is_ns: bool) -> (LightColor, f32) {
        if let Some((forced_c, rem)) = self.forced_ns {
            let color = if is_ns {
                forced_c
            } else {
                match forced_c {
                    LightColor::Green | LightColor::Yellow => LightColor::Red,
                    LightColor::Red => LightColor::Green,
                }
            };
            return (color, rem.max(0.0));
        }

        let t = self.phase_s % FULL_CYCLE;
        if is_ns {
            if t < GREEN_DUR {
                (LightColor::Green, GREEN_DUR - t)
            } else if t < GREEN_DUR + YELLOW_DUR {
                (LightColor::Yellow, GREEN_DUR + YELLOW_DUR - t)
            } else {
                (LightColor::Red, FULL_CYCLE - t)
            }
        } else if t < HALF_CYCLE {
            (LightColor::Red, HALF_CYCLE - t)
        } else {
            let te = t - HALF_CYCLE;
            if te < GREEN_DUR {
                (LightColor::Green, GREEN_DUR - te)
            } else if te < GREEN_DUR + YELLOW_DUR {
                (LightColor::Yellow, GREEN_DUR + YELLOW_DUR - te)
            } else {
                (LightColor::Red, FULL_CYCLE - t)
            }
        }
    }

    pub fn force_ns(&mut self, color: LightColor, duration_s: f32) {
        self.forced_ns = Some((color, duration_s));
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleClass {
    Sedan,
    Suv,
    Truck,
}

impl VehicleClass {
    pub fn length_m(self) -> f32 {
        match self {
            VehicleClass::Sedan => 4.5,
            VehicleClass::Suv => 4.9,
            VehicleClass::Truck => 7.2,
        }
    }
    pub fn width_m(self) -> f32 {
        match self {
            VehicleClass::Sedan => 1.9,
            VehicleClass::Suv => 2.05,
            VehicleClass::Truck => 2.45,
        }
    }
    pub fn qnx_type(self) -> i32 {
        match self {
            VehicleClass::Truck => 1,
            _ => 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriverStyle {
    Cautious,
    Normal,
    Aggressive,
}

#[derive(Debug, Clone)]
pub struct NpcVehicle {
    pub id: usize,
    pub edge_id: usize,
    pub s: f32,
    pub lane_id: usize,
    pub lane_f: f32,
    pub speed: f32,
    pub desired_speed: f32,
    pub class: VehicleClass,
    pub style: DriverStyle,
    pub x: f32,
    pub z: f32,
    pub heading: f32,
    pub color_rgb: (u8, u8, u8),
    pub hold_timer_s: f32,
    pub lc_cooldown_s: f32,
    pub lifetime_s: f32,
    pub is_temp_event: bool,
}

#[derive(Debug, Clone)]
pub struct Pedestrian {
    pub id: usize,
    pub x: f32,
    pub z: f32,
    pub vx: f32,
    pub vz: f32,
    pub heading: f32,
    pub walk_speed: f32,
    pub remaining_travel_m: f32,
    pub pause_s: f32,
    pub is_jaywalker: bool,
    pub ttl_s: f32,
}

#[derive(Debug, Clone)]
pub struct CyclistActor {
    pub id: usize,
    pub x: f32,
    pub z: f32,
    pub vx: f32,
    pub vz: f32,
    pub heading: f32,
    pub speed: f32,
    pub length_m: f32,
    pub width_m: f32,
}

#[derive(Debug, Clone)]
pub struct TrafficConeActor {
    pub x: f32,
    pub z: f32,
}

pub struct TrafficDirector {
    rng: DeterministicRng,
    next_id: usize,
    pub signals: Vec<IntersectionSignal>,
    pub waymo_signals: Vec<(SignalApproach, LightColor, f32)>,
    pub vehicles: Vec<NpcVehicle>,
    pub pedestrians: Vec<Pedestrian>,
    pub cyclists: Vec<CyclistActor>,
    pub cones: Vec<TrafficConeActor>,
    pub target_vehicle_count: usize,
    next_jaywalk_event_s: f32,
    next_jam_event_s: f32,
    elapsed_s: f32,
    pub disable_random_events: bool,
}

impl TrafficDirector {
    pub fn new(seed: u64, traffic_density: f32, net: &RoadNetwork) -> Self {
        let mut dir = Self {
            rng: DeterministicRng::new(seed ^ 0x51A7_C301),
            next_id: 1,
            signals: Vec::new(),
            waymo_signals: Vec::new(),
            vehicles: Vec::new(),
            pedestrians: Vec::new(),
            cyclists: Vec::new(),
            cones: Vec::new(),
            target_vehicle_count: if traffic_density <= 0.01 {
                0
            } else {
                ((30.0 + 80.0 * traffic_density).round() as usize).clamp(12, 65)
            },
            next_jaywalk_event_s: 28.0,
            next_jam_event_s: 42.0,
            elapsed_s: 0.0,
            disable_random_events: false,
        };
        dir.rebuild_for_network(seed, traffic_density, net, (0.0, 0.0));
        dir
    }

    pub fn rebuild_for_network(
        &mut self,
        seed: u64,
        traffic_density: f32,
        net: &RoadNetwork,
        ego_pos: (f32, f32),
    ) {
        self.rng.reseed(seed ^ 0x51A7_C301);
        self.signals.clear();
        self.waymo_signals.clear();
        self.vehicles.clear();
        self.pedestrians.clear();
        self.cyclists.clear();
        self.cones.clear();
        self.elapsed_s = 0.0;
        self.next_jaywalk_event_s = self.rng.gen_range(24.0, 44.0);
        self.next_jam_event_s = self.rng.gen_range(38.0, 62.0);
        self.target_vehicle_count = if traffic_density <= 0.01 {
            0
        } else {
            ((30.0 + 80.0 * traffic_density).round() as usize).clamp(12, 65)
        };

        // 1. Build cycling signals for every 4-way intersection
        for j in &net.junctions {
            if j.kind != JunctionType::Signal4Way {
                continue;
            }
            let mut approaches = Vec::new();
            for e in &net.edges {
                if !e.feature.is_physical_corridor() {
                    continue;
                }
                let end_pt = e.sample_ref(e.total_length);
                let dist_to_center =
                    ((end_pt.0 - j.center_x).powi(2) + (end_pt.1 - j.center_z).powi(2)).sqrt();
                if dist_to_center < 28.0 {
                    // Heading near 0 or PI -> N/S approach; near +-PI/2 -> E/W approach
                    let is_ns = end_pt.2.cos().abs() >= end_pt.2.sin().abs();
                    let s_bar = (e.total_length - 1.2).max(0.5);
                    let bar = e.sample_ref(s_bar);
                    approaches.push(SignalApproach {
                        edge_id: e.id,
                        is_ns,
                        stop_x: bar.0,
                        stop_z: bar.1,
                        heading: bar.2,
                    });
                }
            }
            self.signals.push(IntersectionSignal {
                junction_id: j.id,
                center_x: j.center_x,
                center_z: j.center_z,
                phase_s: self.rng.gen_range(0.0, FULL_CYCLE),
                forced_ns: None,
                approaches,
            });
        }

        // 2. Populate distributed NPC vehicles across physical corridors
        let physical_edges: Vec<usize> = net
            .edges
            .iter()
            .filter(|e| e.feature.is_physical_corridor() && e.total_length > 60.0)
            .map(|e| e.id)
            .collect();

        if !physical_edges.is_empty() && self.target_vehicle_count > 0 {
            let mut attempts = 0usize;
            while self.vehicles.len() < self.target_vehicle_count && attempts < 400 {
                attempts += 1;
                let eid = physical_edges[self.rng.gen_usize(physical_edges.len())];
                let e = &net.edges[eid];
                let s = self.rng.gen_range(22.0, (e.total_length - 25.0).max(24.0));
                let lane_id = self.rng.gen_usize(3);
                let (wx, wz, wh, _) = e.sample_lane(s, lane_id);

                if ((wx - ego_pos.0).powi(2) + (wz - ego_pos.1).powi(2)).sqrt() < 42.0 {
                    continue;
                }
                let too_close = self
                    .vehicles
                    .iter()
                    .any(|v| ((wx - v.x).powi(2) + (wz - v.z).powi(2)).sqrt() < 24.0);
                if too_close {
                    continue;
                }

                let style_roll = self.rng.next_f32();
                let style = if style_roll < 0.25 {
                    DriverStyle::Cautious
                } else if style_roll < 0.80 {
                    DriverStyle::Normal
                } else {
                    DriverStyle::Aggressive
                };
                let cls_roll = self.rng.next_f32();
                let class = if cls_roll < 0.58 {
                    VehicleClass::Sedan
                } else if cls_roll < 0.86 {
                    VehicleClass::Suv
                } else {
                    VehicleClass::Truck
                };

                let base_spd = match style {
                    DriverStyle::Cautious => self.rng.gen_range(9.0, 11.0),
                    DriverStyle::Normal => self.rng.gen_range(11.5, 13.8),
                    DriverStyle::Aggressive => self.rng.gen_range(14.5, 16.5),
                };
                let desired_speed = if class == VehicleClass::Truck {
                    base_spd.min(11.5)
                } else {
                    base_spd
                };

                let colors = [
                    (190, 200, 215),
                    (78, 124, 178),
                    (205, 92, 92),
                    (130, 145, 162),
                    (222, 165, 65),
                    (92, 164, 132),
                ];
                let col = colors[self.rng.gen_usize(colors.len())];

                let id = self.next_id;
                self.next_id += 1;
                self.vehicles.push(NpcVehicle {
                    id,
                    edge_id: eid,
                    s,
                    lane_id,
                    lane_f: lane_id as f32,
                    speed: desired_speed * 0.7,
                    desired_speed,
                    class,
                    style,
                    x: wx,
                    z: wz,
                    heading: wh,
                    color_rgb: col,
                    hold_timer_s: 0.0,
                    lc_cooldown_s: self.rng.gen_range(1.0, 4.0),
                    lifetime_s: 0.0,
                    is_temp_event: false,
                });
            }
        }

        // 3. Spawn ambient sidewalk pedestrians near signal intersections
        for sig in self.signals.iter().take(10) {
            if self.rng.next_f32() < 0.7 {
                let offset_s = if self.rng.next_f32() < 0.5 { 21.0 } else { -21.0 };
                let offset_x = if self.rng.next_f32() < 0.5 { 13.0 } else { -13.0 };
                let px = sig.center_x + offset_x;
                let pz = sig.center_z + offset_s;
                let pid = self.next_id;
                self.next_id += 1;
                self.pedestrians.push(Pedestrian {
                    id: pid,
                    x: px,
                    z: pz,
                    vx: 0.0,
                    vz: 0.0,
                    heading: 0.0,
                    walk_speed: 1.5,
                    remaining_travel_m: 0.0,
                    pause_s: 1.0,
                    is_jaywalker: false,
                    ttl_s: 99999.0,
                });
            }
        }
    }

    pub fn active_signal_for_edge(&self, edge_id: usize) -> Option<(&SignalApproach, LightColor, f32)> {
        for sig in &self.signals {
            for app in &sig.approaches {
                if app.edge_id == edge_id {
                    let (col, rem) = sig.color_for_approach(app.is_ns);
                    return Some((app, col, rem));
                }
            }
        }
        None
    }

    /// Collect all signalized intersection approaches (procedural + active Waymo dynamic signals).
    pub fn iter_signals(&self) -> impl Iterator<Item = (&SignalApproach, LightColor, f32)> {
        let proc_iter = self.signals.iter().flat_map(|sig| {
            sig.approaches.iter().map(move |app| {
                let (col, rem) = sig.color_for_approach(app.is_ns);
                (app, col, rem)
            })
        });
        let waymo_iter = self.waymo_signals.iter().map(|(app, c, rem)| (app, *c, *rem));
        proc_iter.chain(waymo_iter)
    }

    /// Clear ambient procedural state cleanly before populating Waymo HD scene frame.
    pub fn reset_for_waymo(&mut self) {
        self.signals.clear();
        self.waymo_signals.clear();
        self.vehicles.clear();
        self.pedestrians.clear();
        self.cyclists.clear();
        self.cones.clear();
        self.elapsed_s = 0.0;
    }

    /// Populate traffic lights, vehicles, trucks, pedestrians, and cyclists from Waymo ScenarioNet at time `t_s`.
    pub fn load_waymo_frame(&mut self, sc: &WaymoScenario, net: &RoadNetwork, t_s: f32) {
        // Keep any manually injected temporary jam vehicles & active jaywalkers across Waymo playback!
        self.vehicles.retain(|v| v.is_temp_event);
        self.pedestrians.retain(|p| p.is_jaywalker);
        self.cyclists.clear();
        self.waymo_signals.clear();

        let dt_frame = sc.dt_s.max(0.05);
        let t_idx = ((t_s / dt_frame).floor() as usize).min(sc.length.saturating_sub(1));

        // 1. Dynamic Traffic Lights aligned to approach lane heading
        for tl in &sc.traffic_lights {
            let state_str = tl
                .states
                .get(t_idx)
                .map(|s| s.as_str())
                .unwrap_or("UNKNOWN");
            let col = match state_str {
                "RED" => LightColor::Red,
                "YELLOW" => LightColor::Yellow,
                "GREEN" => LightColor::Green,
                _ => continue,
            };
            let mut rem = 0usize;
            for s in tl.states.iter().skip(t_idx) {
                if s == state_str {
                    rem += 1;
                } else {
                    break;
                }
            }
            let (eid, heading) = net
                .waymo_visuals
                .as_ref()
                .and_then(|v| v.lane_id_to_edge.get(&tl.lane_id).copied())
                .map(|eid| {
                    let e = &net.edges[eid];
                    let (s, _, _) = e.project_world(tl.stop_point[0], tl.stop_point[1]);
                    let (_, _, h, _) = e.sample_ref(s);
                    (eid, h)
                })
                .unwrap_or((0, 0.0));
            self.waymo_signals.push((
                SignalApproach {
                    edge_id: eid,
                    is_ns: true,
                    stop_x: tl.stop_point[0],
                    stop_z: tl.stop_point[1],
                    heading,
                },
                col,
                (rem as f32) * dt_frame,
            ));
        }

        // 2. Surrounding Waymo Tracks (Vehicles, Pedestrians, Cyclists)
        let colors = [
            (190, 200, 215),
            (78, 124, 178),
            (205, 92, 92),
            (130, 145, 162),
            (222, 165, 65),
            (92, 164, 132),
        ];
        for (i, tr) in sc.tracks.iter().enumerate() {
            let Some((x, z, h, spd)) = sc.interpolate_track(tr, t_s) else {
                continue;
            };
            let id = 10_000 + i;
            match tr.obj_type.as_str() {
                "PEDESTRIAN" => {
                    self.pedestrians.push(Pedestrian {
                        id,
                        x,
                        z,
                        vx: h.sin() * spd,
                        vz: h.cos() * spd,
                        heading: h,
                        walk_speed: spd,
                        remaining_travel_m: 0.0,
                        pause_s: 0.0,
                        is_jaywalker: false,
                        ttl_s: 1.0,
                    });
                }
                "CYCLIST" => {
                    self.cyclists.push(CyclistActor {
                        id,
                        x,
                        z,
                        vx: h.sin() * spd,
                        vz: h.cos() * spd,
                        heading: h,
                        speed: spd,
                        length_m: tr.length_m.max(1.5),
                        width_m: tr.width_m.max(0.6),
                    });
                }
                _ => {
                    let class = if tr.length_m >= 5.5 || tr.width_m >= 2.3 {
                        VehicleClass::Truck
                    } else if tr.length_m >= 4.75 {
                        VehicleClass::Suv
                    } else {
                        VehicleClass::Sedan
                    };
                    self.vehicles.push(NpcVehicle {
                        id,
                        edge_id: 0,
                        s: 0.0,
                        lane_id: 1,
                        lane_f: 1.0,
                        speed: spd,
                        desired_speed: spd,
                        class,
                        style: DriverStyle::Normal,
                        x,
                        z,
                        heading: h,
                        color_rgb: colors[i % colors.len()],
                        hold_timer_s: 0.0,
                        lc_cooldown_s: 10.0,
                        lifetime_s: t_s,
                        is_temp_event: false,
                    });
                }
            }
        }
    }

    /// Step manually injected jaywalkers/cones/jam vehicles even while Waymo tracks drive from replay.
    pub fn step_waymo_addons(&mut self, ego_pos: (f32, f32), dt: f32) {
        self.elapsed_s += dt;
        for p in &mut self.pedestrians {
            if !p.is_jaywalker {
                continue;
            }
            p.ttl_s -= dt;
            if p.remaining_travel_m > 0.05 {
                let step_d = p.walk_speed * dt;
                p.vx = p.heading.sin() * p.walk_speed;
                p.vz = p.heading.cos() * p.walk_speed;
                p.x += p.vx * dt;
                p.z += p.vz * dt;
                p.remaining_travel_m = (p.remaining_travel_m - step_d).max(0.0);
            } else {
                p.vx = 0.0;
                p.vz = 0.0;
            }
        }
        self.pedestrians.retain(|p| {
            !p.is_jaywalker
                || (p.ttl_s > 0.0
                    && ((p.x - ego_pos.0).powi(2) + (p.z - ego_pos.1).powi(2)).sqrt() < 160.0)
        });
    }

    /// Force upcoming signal on active edges to RED/YELLOW/GREEN (interactive/test use).
    pub fn force_upcoming_signal(
        &mut self,
        active_edges: &[usize],
        cursor: usize,
        color: LightColor,
        duration_s: f32,
    ) -> bool {
        let end = (cursor + 5).min(active_edges.len());
        for &eid in &active_edges[cursor..end] {
            for sig in &mut self.signals {
                if let Some(app) = sig.approaches.iter().find(|a| a.edge_id == eid) {
                    let target_ns = match color {
                        LightColor::Green => {
                            if app.is_ns { LightColor::Green } else { LightColor::Red }
                        }
                        LightColor::Red => {
                            if app.is_ns { LightColor::Red } else { LightColor::Green }
                        }
                        LightColor::Yellow => LightColor::Yellow,
                    };
                    sig.force_ns(target_ns, duration_s);
                    return true;
                }
            }
        }
        false
    }

    /// Step all traffic signals, IDM+MOBIL vehicles, pedestrians, and periodic surprise events.
    pub fn step(
        &mut self,
        net: &RoadNetwork,
        ego_pos: (f32, f32),
        ego_heading: f32,
        ego_speed: f32,
        ego_active_edges: &[usize],
        ego_cursor: usize,
        ego_s: f32,
        dt: f32,
    ) {
        self.elapsed_s += dt;
        for sig in &mut self.signals {
            sig.step(dt);
        }

        // Snapshot positions for lock-free O(N^2) with N<=65 (~4000 pairs = ~12 microseconds in Rust!)
        let snapshot: Vec<(usize, usize, f32, usize, f32, f32, f32, f32)> = self
            .vehicles
            .iter()
            .map(|v| (v.id, v.edge_id, v.s, v.lane_id, v.speed, v.x, v.z, v.class.length_m()))
            .collect();

        for idx in 0..self.vehicles.len() {
            let mut v = self.vehicles[idx].clone();
            v.lifetime_s += dt;
            v.lc_cooldown_s = (v.lc_cooldown_s - dt).max(0.0);
            if v.hold_timer_s > 0.0 {
                v.hold_timer_s -= dt;
            }

            let cur_edge = &net.edges[v.edge_id];
            // Check lead vehicle/ego in same lane + next edge segment
            let mut min_gap = 70.0f32;
            let mut lead_speed = v.desired_speed;

            for &(oid, oedge, os, olane, ospd, _, _, olen) in &snapshot {
                if oid == v.id {
                    continue;
                }
                if oedge == v.edge_id && olane == v.lane_id && os > v.s {
                    let gap = (os - v.s) - 0.5 * (v.class.length_m() + olen);
                    if gap < min_gap {
                        min_gap = gap.max(0.2);
                        lead_speed = ospd;
                    }
                }
            }

            // Check ego vehicle if directly ahead of this NPC in corridor
            let (e_fwd, e_lat) = project_relative(v.x, v.z, v.heading, ego_pos.0, ego_pos.1);
            if e_fwd > 0.5 && e_fwd < 60.0 && e_lat.abs() < 1.75 {
                let gap = e_fwd - 0.5 * (v.class.length_m() + 4.5);
                if gap < min_gap {
                    min_gap = gap.max(0.2);
                    lead_speed = ego_speed;
                }
            }

            // Check upcoming Red / Yellow traffic signal stop line
            if let Some((app, color, _)) = self.active_signal_for_edge(v.edge_id) {
                if matches!(color, LightColor::Red | LightColor::Yellow) {
                    let d_sig = ((app.stop_x - v.x).powi(2) + (app.stop_z - v.z).powi(2)).sqrt();
                    let d_along = (cur_edge.total_length - 1.2 - v.s).max(0.0);
                    if d_along > 0.0 && d_sig < 48.0 && d_along - 2.5 < min_gap {
                        if color == LightColor::Red || d_along > 12.0 {
                            min_gap = (d_along - 2.5).max(0.2);
                            lead_speed = 0.0;
                        }
                    }
                }
            }

            // Check pedestrian ahead of NPC in corridor
            for p in &self.pedestrians {
                let (pf, pl) = project_relative(v.x, v.z, v.heading, p.x, p.z);
                if pf > 0.5 && pf < 25.0 && pl.abs() < 1.8 {
                    let gap = pf - 2.8;
                    if gap < min_gap {
                        min_gap = gap.max(0.2);
                        lead_speed = 0.0;
                    }
                }
            }

            // IDM longitudinal step for NPC
            let target_v = if v.hold_timer_s > 0.0 {
                0.0
            } else {
                v.desired_speed.min(cur_edge.speed_limit_mps)
            };
            let acc = if v.hold_timer_s > 0.0 && min_gap > 5.0 {
                -3.0
            } else if min_gap < 60.0 {
                let s0 = if v.style == DriverStyle::Aggressive { 4.5 } else { 6.0 };
                let t_head = if v.style == DriverStyle::Aggressive { 1.0 } else { 1.4 };
                let dv = v.speed - lead_speed;
                let s_star = s0 + (v.speed * t_head + (v.speed * dv) / 4.2).max(0.0);
                let v_r = (v.speed / target_v.max(0.5)).powi(4);
                let s_r = (s_star / min_gap.max(0.4)).powi(2);
                (2.2 * (1.0 - v_r - s_r)).clamp(-5.5, 2.2)
            } else {
                let v_r = (v.speed / target_v.max(0.5)).powi(4);
                (2.0 * (1.0 - v_r)).clamp(-3.0, 2.0)
            };

            v.speed = (v.speed + acc * dt).max(0.0);
            v.s += v.speed * dt;

            // Occasional MOBIL lane change on straights/arches when blocked
            if v.lc_cooldown_s <= 0.0
                && !v.is_temp_event
                && cur_edge.feature.is_physical_corridor()
                && min_gap < 20.0
                && v.s < cur_edge.total_length - 25.0
                && (v.style == DriverStyle::Aggressive || (v.style == DriverStyle::Normal && self.rng.next_f32() < 0.02))
            {
                let candidate_lane = if v.lane_id == 1 {
                    if self.rng.next_f32() < 0.5 { 0 } else { 2 }
                } else {
                    1
                };
                let lane_clear_npcs = snapshot.iter().all(|&(oid, oedge, os, olane, _, _, _, _)| {
                    oid == v.id || oedge != v.edge_id || olane != candidate_lane || (os - v.s).abs() > 18.0
                });
                let lane_clear_ego = e_fwd.abs() > 18.0 || e_lat.abs() < 1.8; // Avoid cutting inside 18m of ego
                if lane_clear_npcs && lane_clear_ego {
                    v.lane_id = candidate_lane;
                    v.lc_cooldown_s = 6.0;
                }
            }

            v.lane_f += ((v.lane_id as f32 - v.lane_f) * 2.4 * dt).clamp(-2.0 * dt, 2.0 * dt);

            // Transition smoothly across junction connectors onto connected road
            if v.s >= cur_edge.total_length {
                let overflow = v.s - cur_edge.total_length;
                let outs = &net.outgoing[cur_edge.to_node];
                if !outs.is_empty() {
                    let choice = outs[self.rng.gen_usize(outs.len())];
                    v.edge_id = choice;
                    v.s = overflow.min(net.edges[choice].total_length * 0.5);
                } else {
                    v.s = 0.0;
                }
            }

            let lat = (v.lane_f - 1.0) * LANE_WIDTH_M;
            let (nx, nz, nh, _) = net.edges[v.edge_id].sample_at(v.s, lat);
            v.x = nx;
            v.z = nz;
            v.heading = nh;
            self.vehicles[idx] = v;
        }

        // Prune expired temporary jam blockers far from ego
        self.vehicles.retain(|v| {
            !v.is_temp_event
                || (v.lifetime_s < 45.0
                    && ((v.x - ego_pos.0).powi(2) + (v.z - ego_pos.1).powi(2)).sqrt() < 180.0)
        });

        // Step pedestrians
        for p in &mut self.pedestrians {
            p.ttl_s -= dt;
            if p.remaining_travel_m > 0.05 {
                if p.pause_s > 0.0 {
                    p.pause_s -= dt;
                    p.vx = 0.0;
                    p.vz = 0.0;
                } else {
                    let step_d = p.walk_speed * dt;
                    p.vx = p.heading.sin() * p.walk_speed;
                    p.vz = p.heading.cos() * p.walk_speed;
                    p.x += p.vx * dt;
                    p.z += p.vz * dt;
                    p.remaining_travel_m = (p.remaining_travel_m - step_d).max(0.0);
                }
            } else {
                p.vx = 0.0;
                p.vz = 0.0;
            }
        }
        self.pedestrians.retain(|p| {
            !p.is_jaywalker
                || (p.ttl_s > 0.0
                    && ((p.x - ego_pos.0).powi(2) + (p.z - ego_pos.1).powi(2)).sqrt() < 160.0)
        });

        // Periodic random urban surprise events (when not disabled in unit tests)
        if !self.disable_random_events {
            if self.elapsed_s >= self.next_jaywalk_event_s {
                let _ = self.trigger_jaywalker(net, ego_pos, ego_heading, ego_active_edges, ego_cursor, ego_s);
                self.next_jaywalk_event_s = self.elapsed_s + self.rng.gen_range(40.0, 68.0);
            }
            if self.elapsed_s >= self.next_jam_event_s {
                let _ = self.trigger_traffic_jam(net, ego_pos, ego_active_edges, ego_cursor, ego_s, 1);
                self.next_jam_event_s = self.elapsed_s + self.rng.gen_range(50.0, 82.0);
            }
        }
    }

    fn find_ahead_route_point(
        &self,
        net: &RoadNetwork,
        active_edges: &[usize],
        cursor: usize,
        s_ego: f32,
        target_dist_m: f32,
    ) -> Option<(usize, f32)> {
        if active_edges.is_empty() {
            return None;
        }
        let mut rem = s_ego + target_dist_m;
        for &eid in &active_edges[cursor.min(active_edges.len() - 1)..] {
            let e = &net.edges[eid];
            if rem <= e.total_length - 12.0 {
                if e.feature.is_physical_corridor() && rem >= 8.0 {
                    return Some((eid, rem));
                }
                return None;
            }
            rem -= e.total_length;
        }
        None
    }

    /// Spawn a sudden jaywalking pedestrian at curbside 32-38 m ahead walking into ego lane ('J').
    pub fn trigger_jaywalker(
        &mut self,
        net: &RoadNetwork,
        ego_pos: (f32, f32),
        _ego_heading: f32,
        active_edges: &[usize],
        cursor: usize,
        s_ego: f32,
    ) -> String {
        let ahead = self.find_ahead_route_point(net, active_edges, cursor, s_ego, 34.0);
        let (rx, rz, road_h) = match ahead {
            Some((eid, s_t)) => {
                let p = net.edges[eid].sample_ref(s_t);
                (p.0, p.1, p.2)
            }
            None => {
                let _ = ego_pos;
                return "No clear road ahead for jaywalker".to_string();
            }
        };

        let side = if self.rng.next_f32() < 0.5 { -1.0 } else { 1.0 };
        let (right_x, right_z) = right_vec(road_h);
        // Spawn right on roadside curb +/-5.4m laterally, walking perpendicular across ego lane
        let spawn_x = rx + right_x * (side * 5.4);
        let spawn_z = rz + right_z * (side * 5.4);
        let walk_h = if side > 0.0 {
            normalize_angle_rad(road_h - std::f32::consts::FRAC_PI_2)
        } else {
            normalize_angle_rad(road_h + std::f32::consts::FRAC_PI_2)
        };

        let id = self.next_id;
        self.next_id += 1;
        let walk_speed = 2.2;
        self.pedestrians.push(Pedestrian {
            id,
            x: spawn_x,
            z: spawn_z,
            vx: walk_h.sin() * walk_speed,
            vz: walk_h.cos() * walk_speed,
            heading: walk_h,
            walk_speed,
            remaining_travel_m: 14.5,
            pause_s: 0.0,
            is_jaywalker: true,
            ttl_s: 20.0,
        });
        "Jaywalker spawned 34m ahead (Crossing Lane)".to_string()
    }

    /// Spawn a sudden slow/braking traffic jam cut-in ahead ('T').
    pub fn trigger_traffic_jam(
        &mut self,
        net: &RoadNetwork,
        ego_pos: (f32, f32),
        active_edges: &[usize],
        cursor: usize,
        s_ego: f32,
        target_lane: usize,
    ) -> String {
        let mut spawned = 0usize;
        for dist_ahead in [44.0f32, 58.0f32] {
            if let Some((eid, st)) =
                self.find_ahead_route_point(net, active_edges, cursor, s_ego, dist_ahead)
            {
                let lane = target_lane.min(2);
                let (wx, wz, wh, _) = net.edges[eid].sample_lane(st, lane);
                if ((wx - ego_pos.0).powi(2) + (wz - ego_pos.1).powi(2)).sqrt() < 26.0 {
                    continue;
                }
                let clear = self
                    .vehicles
                    .iter()
                    .all(|v| ((wx - v.x).powi(2) + (wz - v.z).powi(2)).sqrt() > 11.0);
                if !clear {
                    continue;
                }
                let id = self.next_id;
                self.next_id += 1;
                self.vehicles.push(NpcVehicle {
                    id,
                    edge_id: eid,
                    s: st,
                    lane_id: lane,
                    lane_f: lane as f32,
                    speed: 5.5,
                    desired_speed: 6.0,
                    class: if spawned == 0 {
                        VehicleClass::Sedan
                    } else {
                        VehicleClass::Truck
                    },
                    style: DriverStyle::Cautious,
                    x: wx,
                    z: wz,
                    heading: wh,
                    color_rgb: (230, 120, 48),
                    hold_timer_s: 4.0,
                    lc_cooldown_s: 12.0,
                    lifetime_s: 0.0,
                    is_temp_event: true,
                });
                // Place roadside safety cones around the slow breakdown zone
                let (_, right_z_dx) = (wh, right_vec(wh));
                for cone_back in [-4.5f32, 0.0, 4.5] {
                    self.cones.push(TrafficConeActor {
                        x: wx + wh.sin() * cone_back + right_z_dx.0 * 1.9,
                        z: wz + wh.cos() * cone_back + right_z_dx.1 * 1.9,
                    });
                }
                if self.cones.len() > 24 {
                    let drop = self.cones.len() - 24;
                    self.cones.drain(0..drop);
                }
                spawned += 1;
            }
        }
        if spawned > 0 {
            format!("Traffic Jam / Slow Cut-In ({} vehicles) ahead", spawned)
        } else {
            "No clear stretch ahead for traffic jam".to_string()
        }
    }
}
