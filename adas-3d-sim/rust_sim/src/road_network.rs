//! Continuous 10+ km tangent-continuous (G1/C1) procedural 3-lane-per-direction road network,
//! analytical Frenet/curvature sampling, snap-to-lane, and A* Point A -> B shortest-path router.

use crate::config::LANE_WIDTH_M;
use crate::coordinate_mapper::{normalize_angle_rad, right_vec, DeterministicRng};
use crate::scenario_loader::{WaymoCrosswalk, WaymoRoadLine, WaymoScenario, WaymoStopSign};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::f32::consts::{FRAC_PI_2, PI};

pub const CARRIAGEWAY_OFFSET_M: f32 = 5.25; // Lane 1 centerline offset right of double-yellow median
pub const INTERSECTION_HALF_SIZE_M: f32 = 18.0;
pub const ROUNDABOUT_RING_RADIUS_M: f32 = 28.0;
pub const ROUNDABOUT_ARM_DIST_M: f32 = 46.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RoadFeatureKind {
    Straight,
    CurveArch,
    IntersectionStraight,
    IntersectionTurn,
    RoundaboutRing,
    RoundaboutFlare,
    HighwayRamp,
    WaymoLane,
}

impl RoadFeatureKind {
    pub fn label(self) -> &'static str {
        match self {
            RoadFeatureKind::Straight => "Multi-Lane Boulevard",
            RoadFeatureKind::CurveArch => "Sweeping Highway S-Arch",
            RoadFeatureKind::IntersectionStraight => "4-Way Signal Intersection",
            RoadFeatureKind::IntersectionTurn => "Intersection Turn Connector",
            RoadFeatureKind::RoundaboutRing => "Multi-Lane Roundabout",
            RoadFeatureKind::RoundaboutFlare => "Roundabout Approach/Exit",
            RoadFeatureKind::HighwayRamp => "Highway Merge/Diverge Ramp",
            RoadFeatureKind::WaymoLane => "Waymo HD Urban Lane",
        }
    }

    pub fn is_junction_box(self) -> bool {
        matches!(
            self,
            RoadFeatureKind::IntersectionStraight
                | RoadFeatureKind::IntersectionTurn
                | RoadFeatureKind::RoundaboutRing
        )
    }

    pub fn is_physical_corridor(self) -> bool {
        matches!(
            self,
            RoadFeatureKind::Straight
                | RoadFeatureKind::CurveArch
                | RoadFeatureKind::HighwayRamp
                | RoadFeatureKind::WaymoLane
        )
    }
}

#[derive(Debug, Clone)]
pub struct HermiteLutSample {
    pub s: f32,
    pub x: f32,
    pub z: f32,
    pub heading: f32,
    pub curvature: f32,
}

#[derive(Debug, Clone)]
pub enum GeometryPiece {
    Straight {
        start_x: f32,
        start_z: f32,
        heading: f32,
        length: f32,
    },
    Arc {
        center_x: f32,
        center_z: f32,
        start_angle_rad: f32, // Angle from center to start point in atan2(x - cx, z - cz) -> tangent matches QNX heading
        radius: f32,
        delta_heading: f32,   // + = right (CW), - = left (CCW)
        length: f32,
    },
    HermiteLut {
        samples: Vec<HermiteLutSample>,
        length: f32,
    },
}

impl GeometryPiece {
    pub fn straight(start_x: f32, start_z: f32, heading: f32, length: f32) -> Self {
        Self::Straight {
            start_x,
            start_z,
            heading: normalize_angle_rad(heading),
            length: length.max(0.05),
        }
    }

    /// Circular arc in QNX frame (heading=0 is +Z, +right).
    /// Left turn (`delta_heading < 0`) curves CCW around center to left;
    /// Right turn (`delta_heading > 0`) curves CW around center to right.
    pub fn arc(
        start_x: f32,
        start_z: f32,
        start_heading: f32,
        radius: f32,
        delta_heading: f32,
    ) -> Self {
        let sign = if delta_heading >= 0.0 { 1.0 } else { -1.0 };
        let h = normalize_angle_rad(start_heading);
        // Right turn (+1): center is right of start (`+(cos h, -sin h) * R`)
        // Left turn (-1): center is left of start (`-(cos h, -sin h) * R`)
        let (rx, rz) = right_vec(h);
        let cx = start_x + sign * rx * radius;
        let cz = start_z + sign * rz * radius;
        // Vector from center to start: -sign * (rx, rz) = (sin(alpha0), cos(alpha0))
        let alpha0 = (-sign * rx).atan2(-sign * rz);
        let length = radius * delta_heading.abs();
        Self::Arc {
            center_x: cx,
            center_z: cz,
            start_angle_rad: alpha0,
            radius,
            delta_heading,
            length: length.max(0.05),
        }
    }

    /// Tangent-continuous cubic Hermite curve with precomputed arc-length lookup table.
    pub fn cubic_hermite(
        p0: (f32, f32),
        h0: f32,
        p1: (f32, f32),
        h1: f32,
    ) -> Self {
        let chord = ((p1.0 - p0.0).powi(2) + (p1.1 - p0.1).powi(2)).sqrt().max(1.0);
        let m = chord * 0.85;
        let t0 = (h0.sin() * m, h0.cos() * m);
        let t1 = (h1.sin() * m, h1.cos() * m);

        let steps = 32usize;
        let mut raw = Vec::with_capacity(steps + 1);
        let mut s_acc = 0.0f32;
        let mut prev_pt = p0;

        for k in 0..=steps {
            let u = k as f32 / steps as f32;
            let u2 = u * u;
            let u3 = u2 * u;

            let h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
            let h10 = u3 - 2.0 * u2 + u;
            let h01 = -2.0 * u3 + 3.0 * u2;
            let h11 = u3 - u2;

            let x = h00 * p0.0 + h10 * t0.0 + h01 * p1.0 + h11 * t1.0;
            let z = h00 * p0.1 + h10 * t0.1 + h01 * p1.1 + h11 * t1.1;

            let dh00 = 6.0 * u2 - 6.0 * u;
            let dh10 = 3.0 * u2 - 4.0 * u + 1.0;
            let dh01 = -6.0 * u2 + 6.0 * u;
            let dh11 = 3.0 * u2 - 2.0 * u;

            let dx = dh00 * p0.0 + dh10 * t0.0 + dh01 * p1.0 + dh11 * t1.0;
            let dz = dh00 * p0.1 + dh10 * t0.1 + dh01 * p1.1 + dh11 * t1.1;

            let ddh00 = 12.0 * u - 6.0;
            let ddh10 = 6.0 * u - 4.0;
            let ddh01 = -12.0 * u + 6.0;
            let ddh11 = 6.0 * u - 2.0;

            let ddx = ddh00 * p0.0 + ddh10 * t0.0 + ddh01 * p1.0 + ddh11 * t1.0;
            let ddz = ddh00 * p0.1 + ddh10 * t0.1 + ddh01 * p1.1 + ddh11 * t1.1;

            if k > 0 {
                s_acc += ((x - prev_pt.0).powi(2) + (z - prev_pt.1).powi(2)).sqrt();
            }
            prev_pt = (x, z);

            let speed2 = dx * dx + dz * dz;
            let speed = speed2.sqrt().max(1e-4);
            let heading = dx.atan2(dz);
            // Signed QNX curvature (+ = right/clockwise): (dx' ? dz'' vs right vector)
            // In QNX (+x right at h=0, +z fwd): dheading/ds = (z'*x'' - x'*z'') / speed^3 wait check sign:
            // h = atan2(dx, dz) -> dh/du = (dz*ddx - dx*ddz) / (dx^2 + dz^2)
            let dh_du = (dz * ddx - dx * ddz) / speed2.max(1e-6);
            let kappa = dh_du / speed;

            raw.push(HermiteLutSample {
                s: s_acc,
                x,
                z,
                heading,
                curvature: kappa,
            });
        }

        Self::HermiteLut {
            length: s_acc.max(0.05),
            samples: raw,
        }
    }

    /// Build a smooth arc-length LUT directly from a real-world HD lane polyline (`[[x, z], ...]`).
    pub fn from_polyline(pts: &[[f32; 2]]) -> Self {
        if pts.len() < 2 {
            return Self::straight(
                pts.first().map(|p| p[0]).unwrap_or(0.0),
                pts.first().map(|p| p[1]).unwrap_or(0.0),
                0.0,
                0.5,
            );
        }

        // Deduplicate near-coincident vertices first
        let mut clean: Vec<[f32; 2]> = Vec::with_capacity(pts.len());
        for &p in pts {
            if let Some(prev) = clean.last() {
                if ((p[0] - prev[0]).powi(2) + (p[1] - prev[1]).powi(2)).sqrt() < 0.08 {
                    continue;
                }
            }
            clean.push(p);
        }
        if clean.len() < 2 {
            clean.push(*pts.last().unwrap());
        }

        let n = clean.len();
        let mut s_vals = vec![0.0f32; n];
        for i in 1..n {
            let ds = ((clean[i][0] - clean[i - 1][0]).powi(2)
                + (clean[i][1] - clean[i - 1][1]).powi(2))
            .sqrt();
            s_vals[i] = s_vals[i - 1] + ds.max(0.01);
        }

        let mut headings = vec![0.0f32; n];
        for i in 0..n {
            let i0 = i.saturating_sub(1);
            let i1 = (i + 1).min(n - 1);
            let dx = clean[i1][0] - clean[i0][0];
            let dz = clean[i1][1] - clean[i0][1];
            headings[i] = if dx.abs() + dz.abs() > 1e-5 {
                normalize_angle_rad(dx.atan2(dz))
            } else if i > 0 {
                headings[i - 1]
            } else {
                0.0
            };
        }

        let mut samples = Vec::with_capacity(n);
        for i in 0..n {
            let i0 = i.saturating_sub(2);
            let i1 = (i + 2).min(n - 1);
            let ds = (s_vals[i1] - s_vals[i0]).max(0.25);
            let dh = normalize_angle_rad(headings[i1] - headings[i0]);
            let kappa = (dh / ds).clamp(-0.18, 0.18);
            samples.push(HermiteLutSample {
                s: s_vals[i],
                x: clean[i][0],
                z: clean[i][1],
                heading: headings[i],
                curvature: kappa,
            });
        }

        Self::HermiteLut {
            length: (*s_vals.last().unwrap_or(&0.5)).max(0.05),
            samples,
        }
    }

    pub fn length(&self) -> f32 {
        match self {
            GeometryPiece::Straight { length, .. }
            | GeometryPiece::Arc { length, .. }
            | GeometryPiece::HermiteLut { length, .. } => *length,
        }
    }

    /// Evaluate exact world `(x, z, heading, curvature)` of lane 1 centerline at `s`.
    pub fn sample(&self, s: f32) -> (f32, f32, f32, f32) {
        match self {
            GeometryPiece::Straight {
                start_x,
                start_z,
                heading,
                length,
            } => {
                let sc = s.clamp(0.0, *length);
                let x = start_x + heading.sin() * sc;
                let z = start_z + heading.cos() * sc;
                (x, z, *heading, 0.0)
            }
            GeometryPiece::Arc {
                center_x,
                center_z,
                start_angle_rad,
                radius,
                delta_heading,
                length,
            } => {
                let sc = s.clamp(0.0, *length);
                let frac = sc / length.max(1e-4);
                let sign = if *delta_heading >= 0.0 { 1.0 } else { -1.0 };
                let sweep = delta_heading * frac;
                let alpha = start_angle_rad + sweep;
                let x = center_x + radius * alpha.sin();
                let z = center_z + radius * alpha.cos();
                // Tangent at alpha along +sweep direction:
                let heading = normalize_angle_rad(alpha + sign * FRAC_PI_2);
                let curvature = sign / radius.max(0.5);
                (x, z, heading, curvature)
            }
            GeometryPiece::HermiteLut { samples, length } => {
                let sc = s.clamp(0.0, *length);
                let idx = samples.partition_point(|smp| smp.s < sc);
                if idx == 0 {
                    let a = &samples[0];
                    return (a.x, a.z, a.heading, a.curvature);
                }
                if idx >= samples.len() {
                    let b = samples.last().unwrap();
                    return (b.x, b.z, b.heading, b.curvature);
                }
                let a = &samples[idx - 1];
                let b = &samples[idx];
                let span = (b.s - a.s).max(1e-5);
                let t = ((sc - a.s) / span).clamp(0.0, 1.0);
                let x = a.x + (b.x - a.x) * t;
                let z = a.z + (b.z - a.z) * t;
                let dh = normalize_angle_rad(b.heading - a.heading);
                let heading = normalize_angle_rad(a.heading + dh * t);
                let curvature = a.curvature + (b.curvature - a.curvature) * t;
                (x, z, heading, curvature)
            }
        }
    }

    /// Nearest longitudinal `s` along this piece (analytical for Straight/Arc, fine refinement for LUT).
    pub fn closest_s(&self, wx: f32, wz: f32) -> f32 {
        match self {
            GeometryPiece::Straight {
                start_x,
                start_z,
                heading,
                length,
            } => {
                let dx = wx - start_x;
                let dz = wz - start_z;
                let proj = dx * heading.sin() + dz * heading.cos();
                proj.clamp(0.0, *length)
            }
            GeometryPiece::Arc {
                center_x,
                center_z,
                start_angle_rad,
                radius,
                delta_heading,
                length,
            } => {
                let dx = wx - center_x;
                let dz = wz - center_z;
                let ang = dx.atan2(dz);
                let diff = normalize_angle_rad(ang - *start_angle_rad);
                let sign = if *delta_heading >= 0.0 { 1.0 } else { -1.0 };
                let sweep = delta_heading.abs();
                // Account cleanly for signed sweep direction
                let dir_diff = diff * sign;
                let s_raw = if dir_diff >= -0.25 && dir_diff <= sweep + 0.25 {
                    dir_diff.clamp(0.0, sweep) * radius
                } else {
                    // Check endpoints
                    let p0 = self.sample(0.0);
                    let p1 = self.sample(*length);
                    let d0 = (wx - p0.0).powi(2) + (wz - p0.1).powi(2);
                    let d1 = (wx - p1.0).powi(2) + (wz - p1.1).powi(2);
                    if d0 <= d1 {
                        0.0
                    } else {
                        *length
                    }
                };
                s_raw.clamp(0.0, *length)
            }
            GeometryPiece::HermiteLut { samples, length } => {
                let mut best_s = 0.0f32;
                let mut best_d2 = f32::MAX;
                for i in 0..samples.len() {
                    let d2 = (wx - samples[i].x).powi(2) + (wz - samples[i].z).powi(2);
                    if d2 < best_d2 {
                        best_d2 = d2;
                        best_s = samples[i].s;
                    }
                }
                // Refine locally via ternary/golden-like probe
                let mut lo = (best_s - 2.0).max(0.0);
                let mut hi = (best_s + 2.0).min(*length);
                for _ in 0..8 {
                    let m1 = lo + (hi - lo) * 0.3333;
                    let m2 = lo + (hi - lo) * 0.6667;
                    let p1 = self.sample(m1);
                    let p2 = self.sample(m2);
                    let d1 = (wx - p1.0).powi(2) + (wz - p1.1).powi(2);
                    let d2 = (wx - p2.0).powi(2) + (wz - p2.1).powi(2);
                    if d1 < d2 {
                        hi = m2;
                    } else {
                        lo = m1;
                    }
                }
                0.5 * (lo + hi)
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct RoadSegment {
    pub id: usize,
    pub from_node: usize,
    pub to_node: usize,
    pub pieces: Vec<GeometryPiece>,
    pub cumulative_s: Vec<f32>,
    pub total_length: f32,
    pub feature: RoadFeatureKind,
    pub is_two_way: bool,
    pub turn_hint: i32, // -1 left, 0 straight, +1 right
    pub speed_limit_mps: f32,
    pub bbox_min_x: f32,
    pub bbox_max_x: f32,
    pub bbox_min_z: f32,
    pub bbox_max_z: f32,
}

impl RoadSegment {
    pub fn new(
        id: usize,
        from_node: usize,
        to_node: usize,
        pieces: Vec<GeometryPiece>,
        feature: RoadFeatureKind,
        is_two_way: bool,
        turn_hint: i32,
        speed_limit_mps: f32,
    ) -> Self {
        let mut cumulative_s = Vec::with_capacity(pieces.len() + 1);
        cumulative_s.push(0.0);
        let mut total = 0.0f32;
        for p in &pieces {
            total += p.length();
            cumulative_s.push(total);
        }

        let mut min_x = f32::MAX;
        let mut max_x = f32::MIN;
        let mut min_z = f32::MAX;
        let mut max_z = f32::MIN;
        let n_smp = ((total / 8.0).ceil() as usize).max(4);
        for k in 0..=n_smp {
            let s = (k as f32 / n_smp as f32) * total;
            let (x, z, _, _) = Self::eval_raw(&pieces, &cumulative_s, total, s);
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_z = min_z.min(z);
            max_z = max_z.max(z);
        }
        let pad = 16.0;
        Self {
            id,
            from_node,
            to_node,
            pieces,
            cumulative_s,
            total_length: total.max(0.05),
            feature,
            is_two_way,
            turn_hint,
            speed_limit_mps,
            bbox_min_x: min_x - pad,
            bbox_max_x: max_x + pad,
            bbox_min_z: min_z - pad,
            bbox_max_z: max_z + pad,
        }
    }

    fn eval_raw(
        pieces: &[GeometryPiece],
        cum: &[f32],
        total: f32,
        s: f32,
    ) -> (f32, f32, f32, f32) {
        let sc = s.clamp(0.0, total);
        let idx = pieces
            .len()
            .saturating_sub(1)
            .min(cum.partition_point(|&v| v <= sc).saturating_sub(1));
        pieces[idx].sample(sc - cum[idx])
    }

    /// Evaluate lane `1` reference `(x, z, heading, curvature)` at `s`.
    #[inline]
    pub fn sample_ref(&self, s: f32) -> (f32, f32, f32, f32) {
        Self::eval_raw(&self.pieces, &self.cumulative_s, self.total_length, s)
    }

    /// Evaluate `(x, z, heading, curvature)` at longitudinal `s` and lateral offset `lat_m` from lane 1.
    #[inline]
    pub fn sample_at(&self, s: f32, lat_m: f32) -> (f32, f32, f32, f32) {
        let (rx, rz, h, k) = self.sample_ref(s);
        let (dx, dz) = right_vec(h);
        (rx + dx * lat_m, rz + dz * lat_m, h, k)
    }

    /// Evaluate center of specific lane `lane_id` (0=Left -3.5, 1=Center 0.0, 2=Right +3.5).
    #[inline]
    pub fn sample_lane(&self, s: f32, lane_id: usize) -> (f32, f32, f32, f32) {
        let lat = (lane_id as f32 - 1.0) * LANE_WIDTH_M;
        self.sample_at(s, lat)
    }

    /// Project world `(wx, wz)` onto `(s, lateral_from_lane1, distance_m)`.
    pub fn project_world(&self, wx: f32, wz: f32) -> (f32, f32, f32) {
        let mut best_s = 0.0f32;
        let mut best_d2 = f32::MAX;
        for (i, piece) in self.pieces.iter().enumerate() {
            let local_s = piece.closest_s(wx, wz);
            let (px, pz, _, _) = piece.sample(local_s);
            let d2 = (wx - px).powi(2) + (wz - pz).powi(2);
            if d2 < best_d2 {
                best_d2 = d2;
                best_s = self.cumulative_s[i] + local_s;
            }
        }
        let (rx, rz, h, _) = self.sample_ref(best_s);
        let (ux, uz) = right_vec(h);
        let lat = (wx - rx) * ux + (wz - rz) * uz;
        let dist = ((wx - rx).powi(2) + (wz - rz).powi(2)).sqrt();
        (best_s, lat, dist)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JunctionType {
    Signal4Way,
    Roundabout,
}

#[derive(Debug, Clone)]
pub struct JunctionMeta {
    pub id: usize,
    pub kind: JunctionType,
    pub center_x: f32,
    pub center_z: f32,
}

#[derive(Debug, Clone)]
pub struct RoadSnap {
    pub edge_id: usize,
    pub s: f32,
    pub lane_id: usize,
    pub lateral_offset: f32,
    pub world_pos: (f32, f32),
    pub heading: f32,
    pub dist_m: f32,
}

#[derive(Debug, Clone)]
pub struct PlannedRoute {
    pub edge_ids: Vec<usize>,
    pub start_s: f32,
    pub goal_edge_id: usize,
    pub goal_s: f32,
    pub goal_world: (f32, f32),
    pub total_distance_m: f32,
    pub polyline: Vec<(f32, f32)>,
    pub is_fallback: bool,
    pub reason: String,
}

#[derive(Debug, Clone, Copy)]
pub struct RouteCue {
    pub has_route: i32,
    pub ref_x: f32,
    pub ref_z: f32,
    pub ref_heading: f32,
    pub path_curvature: f32,
    pub next_curvature: f32,
    pub dist_to_goal: f32,
    pub turn_signal_hint: i32,
    pub dist_to_next_turn: f32,
    pub pre_turn_lane_hint: i32,
    pub in_intersection: i32,
    pub current_lane: i32,
    pub feature: RoadFeatureKind,
}

#[derive(Debug, Clone)]
struct GraphNode {
    x: f32,
    z: f32,
}

#[derive(Debug, Clone)]
pub struct WaymoVisualGeometry {
    pub scenario_id: String,
    pub road_lines: Vec<WaymoRoadLine>,
    pub crosswalks: Vec<WaymoCrosswalk>,
    pub stop_signs: Vec<WaymoStopSign>,
    pub lane_id_to_edge: HashMap<String, usize>,
    pub ego_start_pose: [f32; 4],
    pub ego_goal_pose: [f32; 3],
}

pub struct RoadNetwork {
    pub seed: u64,
    nodes: Vec<GraphNode>,
    pub edges: Vec<RoadSegment>,
    pub outgoing: Vec<Vec<usize>>,
    pub junctions: Vec<JunctionMeta>,
    pub grand_tour_edges: Vec<usize>,
    pub grand_tour_length_m: f32,
    pub unique_corridor_length_m: f32,
    pub bounds: (f32, f32, f32, f32),
    pub is_single_lane_hd: bool,
    pub waymo_visuals: Option<WaymoVisualGeometry>,
}

impl RoadNetwork {
    pub fn find_snap(&self, wx: f32, wz: f32) -> Option<RoadSnap> {
        let mut best: Option<RoadSnap> = None;
        let mut best_cost = f32::MAX;

        for edge in &self.edges {
            if wx < edge.bbox_min_x - 60.0
                || wx > edge.bbox_max_x + 60.0
                || wz < edge.bbox_min_z - 60.0
                || wz > edge.bbox_max_z + 60.0
            {
                continue;
            }
            let (s, lat, _) = edge.project_world(wx, wz);
            let (clamped_lat, lane_id) = if self.is_single_lane_hd {
                (0.0, 1)
            } else {
                let lid = if lat < -0.5 * LANE_WIDTH_M {
                    0
                } else if lat > 0.5 * LANE_WIDTH_M {
                    2
                } else {
                    1
                };
                (lat.clamp(-LANE_WIDTH_M, LANE_WIDTH_M), lid)
            };
            let (px, pz, h, _) = edge.sample_at(s, clamped_lat);
            let d = ((wx - px).powi(2) + (wz - pz).powi(2)).sqrt();
            // Slightly prefer physical road corridors over internal junction boxes on overlap
            let penalty = if edge.feature.is_physical_corridor() {
                0.0
            } else {
                2.0
            };
            if d + penalty < best_cost {
                best_cost = d + penalty;
                best = Some(RoadSnap {
                    edge_id: edge.id,
                    s,
                    lane_id,
                    lateral_offset: lat,
                    world_pos: (px, pz),
                    heading: h,
                    dist_m: d,
                });
            }
        }
        best
    }

    pub fn edge(&self, id: usize) -> &RoadSegment {
        &self.edges[id]
    }

    /// Shortest-path A* over directed road segments from `(start_edge, start_s)` to `(goal_edge, goal_s)`.
    pub fn plan_a_star(
        &self,
        start_edge: usize,
        start_s: f32,
        goal_edge: usize,
        goal_s: f32,
    ) -> Option<PlannedRoute> {
        let e_start = &self.edges[start_edge];
        let e_goal = &self.edges[goal_edge];

        // 1. Same segment forward
        if start_edge == goal_edge && goal_s >= start_s + 2.0 {
            let (gx, gz, _, _) = e_goal.sample_ref(goal_s);
            let dist = goal_s - start_s;
            let poly = self.sample_polyline(&[start_edge], start_s, goal_s, 8.0);
            return Some(PlannedRoute {
                edge_ids: vec![start_edge],
                start_s,
                goal_edge_id: goal_edge,
                goal_s,
                goal_world: (gx, gz),
                total_distance_m: dist,
                polyline: poly,
                is_fallback: false,
                reason: "Direct shortest path".to_string(),
            });
        }

        #[derive(Copy, Clone, PartialEq)]
        struct State {
            cost: f32,
            g: f32,
            edge_id: usize,
        }
        impl Eq for State {}
        impl Ord for State {
            fn cmp(&self, other: &Self) -> Ordering {
                other
                    .cost
                    .partial_cmp(&self.cost)
                    .unwrap_or(Ordering::Equal)
            }
        }
        impl PartialOrd for State {
            fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
                Some(self.cmp(other))
            }
        }

        let (gx, gz, _, _) = e_goal.sample_ref(goal_s);
        let heuristic = |eid: usize| -> f32 {
            let e = &self.edges[eid];
            let n = &self.nodes[e.to_node];
            ((gx - n.x).powi(2) + (gz - n.z).powi(2)).sqrt()
        };

        let n_edges = self.edges.len();
        let mut best_g = vec![f32::INFINITY; n_edges];
        let mut parent = vec![usize::MAX; n_edges];
        let mut heap = BinaryHeap::new();

        // Advance out of start_edge through its outgoing neighbors so ego never U-turns backward instantaneously
        let rem_start = (e_start.total_length - start_s).max(0.0);
        for &next_eid in &self.outgoing[e_start.to_node] {
            let ne = &self.edges[next_eid];
            let g0 = rem_start + if next_eid == goal_edge { goal_s } else { ne.total_length };
            best_g[next_eid] = g0;
            parent[next_eid] = start_edge;
            heap.push(State {
                cost: g0 + heuristic(next_eid),
                g: g0,
                edge_id: next_eid,
            });
        }

        let mut reached = false;
        while let Some(st) = heap.pop() {
            if st.edge_id == goal_edge {
                reached = true;
                break;
            }
            if st.g > best_g[st.edge_id] + 1e-3 {
                continue;
            }
            let cur_edge = &self.edges[st.edge_id];
            for &next_eid in &self.outgoing[cur_edge.to_node] {
                if next_eid == st.edge_id {
                    continue;
                }
                let ne = &self.edges[next_eid];
                let step_len = if next_eid == goal_edge {
                    goal_s
                } else {
                    ne.total_length
                };
                let ng = st.g + step_len;
                if ng < best_g[next_eid] {
                    best_g[next_eid] = ng;
                    parent[next_eid] = st.edge_id;
                    heap.push(State {
                        cost: ng + heuristic(next_eid),
                        g: ng,
                        edge_id: next_eid,
                    });
                }
            }
        }

        if reached {
            let mut rev_path = Vec::new();
            let mut cur = goal_edge;
            while cur != usize::MAX {
                rev_path.push(cur);
                if cur == start_edge {
                    break;
                }
                cur = parent[cur];
            }
            if rev_path.last() == Some(&start_edge) {
                rev_path.reverse();
                let total_dist = best_g[goal_edge];
                let poly = self.sample_polyline(&rev_path, start_s, goal_s, 8.0);
                return Some(PlannedRoute {
                    edge_ids: rev_path,
                    start_s,
                    goal_edge_id: goal_edge,
                    goal_s,
                    goal_world: (gx, gz),
                    total_distance_m: total_dist,
                    polyline: poly,
                    is_fallback: false,
                    reason: "A* shortest path (Point A -> B)".to_string(),
                });
            }
        }

        None
    }

    /// Track ego position cleanly along `active_edges`, returning updated `(route_idx, s_on_edge, RouteCue)`.
    pub fn compute_route_cue(
        &self,
        ego_x: f32,
        ego_z: f32,
        ego_heading: f32,
        active_edges: &[usize],
        route_cursor: usize,
        goal_s_tail: Option<f32>,
    ) -> (usize, f32, RouteCue) {
        if active_edges.is_empty() {
            return (
                0,
                0.0,
                RouteCue {
                    has_route: 0,
                    ref_x: ego_x,
                    ref_z: ego_z,
                    ref_heading: ego_heading,
                    path_curvature: 0.0,
                    next_curvature: 0.0,
                    dist_to_goal: 99999.0,
                    turn_signal_hint: 0,
                    dist_to_next_turn: 999.0,
                    pre_turn_lane_hint: 0,
                    in_intersection: 0,
                    current_lane: 1,
                    feature: RoadFeatureKind::Straight,
                },
            );
        }

        // Advance cursor smoothly along consecutive active edges (look at current & next 2 edges)
        let mut best_idx = route_cursor.min(active_edges.len() - 1);
        let mut best_s = 0.0f32;
        let mut best_lat = 0.0f32;
        let mut best_d = f32::MAX;

        let search_hi = (best_idx + 2).min(active_edges.len() - 1);
        for i in best_idx..=search_hi {
            let e = &self.edges[active_edges[i]];
            let (s, lat, d) = e.project_world(ego_x, ego_z);
            // Bias slightly toward staying on current edge until passing its end
            let penalty = if i == best_idx { 0.0 } else { 0.8 };
            if d + penalty < best_d {
                best_d = d + penalty;
                best_idx = i;
                best_s = s;
                best_lat = lat;
            }
        }

        let cur_edge = &self.edges[active_edges[best_idx]];
        let (ref_x, ref_z, ref_heading, k_here) = cur_edge.sample_ref(best_s);

        // Current discrete lane index [0, 1, 2] (Waymo single-lane HD maps stay pinned to lane 1 center)
        let current_lane = if self.is_single_lane_hd {
            1
        } else if best_lat < -0.5 * LANE_WIDTH_M {
            0
        } else if best_lat > 0.5 * LANE_WIDTH_M {
            2
        } else {
            1
        };

        // Helper to sample curvature delta_s meters ahead across edge chain
        let sample_ahead_curv = |delta_s: f32| -> (f32, &RoadSegment) {
            let mut rem = best_s + delta_s;
            for i in best_idx..active_edges.len() {
                let e = &self.edges[active_edges[i]];
                if rem <= e.total_length || i == active_edges.len() - 1 {
                    let (_, _, _, k) = e.sample_ref(rem.min(e.total_length));
                    return (k, e);
                }
                rem -= e.total_length;
            }
            let last = &self.edges[*active_edges.last().unwrap()];
            (last.sample_ref(last.total_length).3, last)
        };

        // Front-axle preview blend for Ackermann feedforward (kappa_preview)
        let (k_ahead_2, _) = sample_ahead_curv(2.2);
        let path_curvature = 0.78 * k_here + 0.22 * k_ahead_2;

        // Lookahead peak curvature [5m .. 25m] ahead for smooth predictive cornering speed reduction
        let mut next_curvature = 0.0f32;
        for d in [6.0, 10.0, 15.0, 20.0, 25.0] {
            let (kp, _) = sample_ahead_curv(d);
            if kp.abs() > next_curvature.abs() {
                next_curvature = kp;
            }
        }

        // Exact distance to goal along active route
        let dist_to_goal = if let Some(gs) = goal_s_tail {
            let last_i = active_edges.len() - 1;
            if best_idx == last_i {
                (gs - best_s).max(0.0)
            } else {
                let rem_cur = (cur_edge.total_length - best_s).max(0.0);
                let mid: f32 = active_edges[(best_idx + 1)..last_i]
                    .iter()
                    .map(|&eid| self.edges[eid].total_length)
                    .sum();
                (rem_cur + mid + gs).max(0.0)
            }
        } else {
            // Free cruise continuous tour: maintain smooth cruise horizon until tour wraps
            9999.0
        };

        // Upcoming intersection turn signals & pre-turn lane hint
        let mut turn_signal_hint = 0i32;
        let mut dist_to_next_turn = 999.0f32;
        let mut pre_turn_lane_hint = 0i32;
        let mut d_acc = (cur_edge.total_length - best_s).max(0.0);
        if cur_edge.turn_hint != 0 {
            turn_signal_hint = cur_edge.turn_hint;
            dist_to_next_turn = 0.0;
        } else {
            for &eid in &active_edges[(best_idx + 1)..] {
                if d_acc > 95.0 {
                    break;
                }
                let e = &self.edges[eid];
                if e.turn_hint != 0 {
                    turn_signal_hint = e.turn_hint;
                    dist_to_next_turn = d_acc;
                    pre_turn_lane_hint = e.turn_hint;
                    break;
                }
                d_acc += e.total_length;
            }
        }

        let cue = RouteCue {
            has_route: 1,
            ref_x,
            ref_z,
            ref_heading,
            path_curvature,
            next_curvature,
            dist_to_goal,
            turn_signal_hint,
            dist_to_next_turn,
            pre_turn_lane_hint,
            in_intersection: if cur_edge.feature.is_junction_box() { 1 } else { 0 },
            current_lane: current_lane as i32,
            feature: cur_edge.feature,
        };

        (best_idx, best_s, cue)
    }

    pub fn sample_polyline(
        &self,
        edge_ids: &[usize],
        start_s: f32,
        goal_s: f32,
        step_m: f32,
    ) -> Vec<(f32, f32)> {
        let mut pts = Vec::new();
        let last = edge_ids.len().saturating_sub(1);
        for (i, &eid) in edge_ids.iter().enumerate() {
            let e = &self.edges[eid];
            let s0 = if i == 0 { start_s.max(0.0) } else { 0.0 };
            let s1 = if i == last {
                goal_s.clamp(s0, e.total_length)
            } else {
                e.total_length
            };
            if s1 <= s0 {
                continue;
            }
            let steps = (((s1 - s0) / step_m.max(1.0)).ceil() as usize).max(2);
            for k in 0..=steps {
                let s = s0 + (k as f32 / steps as f32) * (s1 - s0);
                let (x, z, _, _) = e.sample_ref(s);
                pts.push((x, z));
            }
        }
        pts
    }
}

/// Deterministic ~10+ km Real-World Procedural Multi-District Road Generator.
pub struct MegaMapGenerator;

impl MegaMapGenerator {
    /// Generate a non-overlapping, G1-tangent-continuous multi-lane (~15 km corridors, >= 10.5 km Grand Tour) map.
    pub fn generate(seed: u64) -> RoadNetwork {
        let mut rng = DeterministicRng::new(seed);
        let mut nodes: Vec<GraphNode> = Vec::new();
        let mut edges: Vec<RoadSegment> = Vec::new();
        let mut junctions: Vec<JunctionMeta> = Vec::new();

        let add_node = |nodes: &mut Vec<GraphNode>, x: f32, z: f32| -> usize {
            let id = nodes.len();
            nodes.push(GraphNode { x, z });
            id
        };

        let mut add_edge = |nodes: &[GraphNode],
                            from_node: usize,
                            to_node: usize,
                            pieces: Vec<GeometryPiece>,
                            feature: RoadFeatureKind,
                            is_two_way: bool,
                            turn_hint: i32,
                            speed_limit_mps: f32|
         -> usize {
            let _ = nodes;
            let id = edges.len();
            edges.push(RoadSegment::new(
                id,
                from_node,
                to_node,
                pieces,
                feature,
                is_two_way,
                turn_hint,
                speed_limit_mps,
            ));
            id
        };

        // Grid coordinates (4 cols x 3 rows, 480m corridor separation -> zero non-junction overlap)
        const XS: [f32; 4] = [-720.0, -240.0, 240.0, 720.0];
        const ZS: [f32; 3] = [-480.0, 0.0, 480.0];

        // Choose 2 interior diagonal roundabouts deterministically per seed
        let parity = rng.gen_usize(2);
        let is_roundabout_cell = |col: usize, row: usize| -> bool {
            (col == 1 && row == parity) || (col == 2 && row == (1 - parity))
        };

        // Per-junction cardinal port node IDs: ports[col][row][dir] = (in_node, out_node)
        // dir: 0=North(+Z), 1=East(+X), 2=South(-Z), 3=West(-X)
        let mut ports = [[[(0usize, 0usize); 4]; 3]; 4];

        for col in 0..4 {
            for row in 0..3 {
                let cx = XS[col];
                let cz = ZS[row];
                let jid = junctions.len();
                let is_ro = is_roundabout_cell(col, row);
                junctions.push(JunctionMeta {
                    id: jid,
                    kind: if is_ro {
                        JunctionType::Roundabout
                    } else {
                        JunctionType::Signal4Way
                    },
                    center_x: cx,
                    center_z: cz,
                });

                let d_arm = if is_ro {
                    ROUNDABOUT_ARM_DIST_M
                } else {
                    INTERSECTION_HALF_SIZE_M
                };
                // Directions: 0:+Z(h=0), 1:+X(h=PI/2), 2:-Z(h=PI), 3:-X(h=-PI/2)
                let headings = [0.0f32, FRAC_PI_2, PI, -FRAC_PI_2];
                for (dir, &out_h) in headings.iter().enumerate() {
                    let (fx, fz) = (out_h.sin(), out_h.cos());
                    let (rx, rz) = right_vec(out_h);
                    // Outbound node sits +5.25m right of outbound radial axis
                    let ox = cx + fx * d_arm + rx * CARRIAGEWAY_OFFSET_M;
                    let oz = cz + fz * d_arm + rz * CARRIAGEWAY_OFFSET_M;
                    // Inbound node sits -5.25m left of outbound radial axis (right-hand incoming side)
                    let ix = cx + fx * d_arm - rx * CARRIAGEWAY_OFFSET_M;
                    let iz = cz + fz * d_arm - rz * CARRIAGEWAY_OFFSET_M;

                    let in_id = add_node(&mut nodes, ix, iz);
                    let out_id = add_node(&mut nodes, ox, oz);
                    ports[col][row][dir] = (in_id, out_id);
                }

                if !is_ro {
                    // Signalized 4-Way Intersection: exact Straight, RightArc(R=12.75), LeftArc(R=23.25)
                    let r_right = INTERSECTION_HALF_SIZE_M - CARRIAGEWAY_OFFSET_M;
                    let r_left = INTERSECTION_HALF_SIZE_M + CARRIAGEWAY_OFFSET_M;
                    for d_in in 0..4 {
                        let from_node = ports[col][row][d_in].0;
                        let approach_h = headings[(d_in + 2) % 4];
                        let np_in = (nodes[from_node].x, nodes[from_node].z);

                        // Straight across to opposite arm (d_out = (d_in + 2) % 4)
                        let to_str = ports[col][row][(d_in + 2) % 4].1;
                        add_edge(
                            &nodes,
                            from_node,
                            to_str,
                            vec![GeometryPiece::straight(
                                np_in.0,
                                np_in.1,
                                approach_h,
                                2.0 * INTERSECTION_HALF_SIZE_M,
                            )],
                            RoadFeatureKind::IntersectionStraight,
                            false,
                            0,
                            14.0,
                        );

                        // Right turn (+90 deg -> d_out = (d_in + 3) % 4)
                        let to_right = ports[col][row][(d_in + 3) % 4].1;
                        add_edge(
                            &nodes,
                            from_node,
                            to_right,
                            vec![GeometryPiece::arc(
                                np_in.0,
                                np_in.1,
                                approach_h,
                                r_right,
                                FRAC_PI_2,
                            )],
                            RoadFeatureKind::IntersectionTurn,
                            false,
                            1,
                            10.0,
                        );

                        // Left turn (-90 deg -> d_out = (d_in + 1) % 4)
                        let to_left = ports[col][row][(d_in + 1) % 4].1;
                        add_edge(
                            &nodes,
                            from_node,
                            to_left,
                            vec![GeometryPiece::arc(
                                np_in.0,
                                np_in.1,
                                approach_h,
                                r_left,
                                -FRAC_PI_2,
                            )],
                            RoadFeatureKind::IntersectionTurn,
                            false,
                            -1,
                            11.5,
                        );
                    }
                } else {
                    // Counter-Clockwise (CCW) Roundabout (8 ring nodes: Entry +30deg, Exit -30deg per side)
                    let mut ring_entry_nodes = [0usize; 4];
                    let mut ring_exit_nodes = [0usize; 4];
                    // Alpha angles measured from +Z clockwise; CCW traversal decreases alpha!
                    // Arms: 0=North(0), 1=East(+90), 2=South(+180), 3=West(-90)
                    let arm_alphas = [0.0f32, FRAC_PI_2, PI, -FRAC_PI_2];
                    let off_rad = 32.0f32.to_radians();

                    for d in 0..4 {
                        // Exit port occurs before cardinal heading (alpha + off_rad), Entry occurs after (alpha - off_rad)
                        let a_exit = arm_alphas[d] + off_rad;
                        let a_entry = arm_alphas[d] - off_rad;

                        let ex_x = cx + ROUNDABOUT_RING_RADIUS_M * a_exit.sin();
                        let ex_z = cz + ROUNDABOUT_RING_RADIUS_M * a_exit.cos();
                        let en_x = cx + ROUNDABOUT_RING_RADIUS_M * a_entry.sin();
                        let en_z = cz + ROUNDABOUT_RING_RADIUS_M * a_entry.cos();

                        ring_exit_nodes[d] = add_node(&mut nodes, ex_x, ex_z);
                        ring_entry_nodes[d] = add_node(&mut nodes, en_x, en_z);
                    }

                    for d in 0..4 {
                        let d_prev = (d + 3) % 4; // Next CCW around ring: d_exit -> d_entry -> d_prev_exit
                        // 1. Entry flare from arm_in to ring_entry[d]
                        let arm_in = ports[col][row][d].0;
                        let arm_in_h = headings[(d + 2) % 4];
                        let p_in = (nodes[arm_in].x, nodes[arm_in].z);
                        let re = ring_entry_nodes[d];
                        let p_re = (nodes[re].x, nodes[re].z);
                        let a_entry = arm_alphas[d] - off_rad;
                        let re_h = normalize_angle_rad(a_entry - FRAC_PI_2); // CCW tangent
                        add_edge(
                            &nodes,
                            arm_in,
                            re,
                            vec![GeometryPiece::cubic_hermite(p_in, arm_in_h, p_re, re_h)],
                            RoadFeatureKind::RoundaboutFlare,
                            false,
                            1,
                            10.0,
                        );

                        // 2. Exit flare from ring_exit[d] to arm_out
                        let rx_id = ring_exit_nodes[d];
                        let arm_out = ports[col][row][d].1;
                        let arm_out_h = headings[d];
                        let p_rx = (nodes[rx_id].x, nodes[rx_id].z);
                        let p_out = (nodes[arm_out].x, nodes[arm_out].z);
                        let a_exit = arm_alphas[d] + off_rad;
                        let rx_h = normalize_angle_rad(a_exit - FRAC_PI_2);
                        add_edge(
                            &nodes,
                            rx_id,
                            arm_out,
                            vec![GeometryPiece::cubic_hermite(p_rx, rx_h, p_out, arm_out_h)],
                            RoadFeatureKind::RoundaboutFlare,
                            false,
                            1,
                            11.0,
                        );

                        // 3. CCW Ring Arc from ring_exit[d] -> ring_entry[d] (64 deg sweep)
                        add_edge(
                            &nodes,
                            rx_id,
                            re,
                            vec![GeometryPiece::arc(
                                p_rx.0,
                                p_rx.1,
                                rx_h,
                                ROUNDABOUT_RING_RADIUS_M,
                                -2.0 * off_rad,
                            )],
                            RoadFeatureKind::RoundaboutRing,
                            false,
                            0,
                            10.0,
                        );

                        // 4. CCW Ring Arc from ring_entry[d] -> ring_exit[d_prev] (26 deg sweep)
                        let next_rx = ring_exit_nodes[d_prev];
                        let sweep_between = -(FRAC_PI_2 - 2.0 * off_rad);
                        add_edge(
                            &nodes,
                            re,
                            next_rx,
                            vec![GeometryPiece::arc(
                                p_re.0,
                                p_re.1,
                                re_h,
                                ROUNDABOUT_RING_RADIUS_M,
                                sweep_between,
                            )],
                            RoadFeatureKind::RoundaboutRing,
                            false,
                            -1,
                            10.0,
                        );
                    }
                }
            }
        }

        // Helper to connect two opposing cardinal ports cleanly with a straight bidirectional boulevard
        let mut connect_grid_corridor =
            |c1: usize, r1: usize, d1: usize, c2: usize, r2: usize, d2: usize, heading12: f32| {
                let p1_out = ports[c1][r1][d1].1;
                let p2_in = ports[c2][r2][d2].0;
                let len12 = ((nodes[p2_in].x - nodes[p1_out].x).powi(2)
                    + (nodes[p2_in].z - nodes[p1_out].z).powi(2))
                .sqrt();
                let e12 = add_edge(
                    &nodes,
                    p1_out,
                    p2_in,
                    vec![GeometryPiece::straight(
                        nodes[p1_out].x,
                        nodes[p1_out].z,
                        heading12,
                        len12,
                    )],
                    RoadFeatureKind::Straight,
                    true,
                    0,
                    14.0,
                );

                let p2_out = ports[c2][r2][d2].1;
                let p1_in = ports[c1][r1][d1].0;
                let h21 = normalize_angle_rad(heading12 + PI);
                let e21 = add_edge(
                    &nodes,
                    p2_out,
                    p1_in,
                    vec![GeometryPiece::straight(
                        nodes[p2_out].x,
                        nodes[p2_out].z,
                        h21,
                        len12,
                    )],
                    RoadFeatureKind::Straight,
                    true,
                    0,
                    14.0,
                );
                (e12, e21, len12)
            };

        // Store key straight edge IDs to construct a continuous >= 10.5 km Grand Tour
        let mut h_edges = [[(0usize, 0usize); 3]; 3]; // [row][col_seg]
        let mut v_edges = [[(0usize, 0usize); 2]; 4]; // [col][row_seg]
        let mut unique_corridor_len = 0.0f32;

        // Horizontal grid avenues (heading East = +PI/2)
        for row in 0..3 {
            for col in 0..3 {
                let (e_e, e_w, l) =
                    connect_grid_corridor(col, row, 1, col + 1, row, 3, FRAC_PI_2);
                h_edges[row][col] = (e_e, e_w);
                unique_corridor_len += l;
            }
        }
        // Vertical grid avenues (heading North = 0)
        for col in 0..4 {
            for row in 0..2 {
                let (e_n, e_s, l) = connect_grid_corridor(col, row, 0, col, row + 1, 2, 0.0);
                v_edges[col][row] = (e_n, e_s);
                unique_corridor_len += l;
            }
        }

        // Helper to build a sweeping bidirectional S-Arch Perimeter Highway between two same-facing outer ports!
        let mut add_scenic_s_arch = |col_a: usize,
                                     row_a: usize,
                                     dir_a: usize,
                                     col_b: usize,
                                     row_b: usize,
                                     dir_b: usize,
                                     start_h: f32,
                                     straight_out: f32,
                                     r_corner: f32,
                                     r_wave: f32,
                                     wave_deg: f32|
         -> (usize, usize, f32) {
            let p_a_out = ports[col_a][row_a][dir_a].1;
            let p_b_in = ports[col_b][row_b][dir_b].0;
            let (rx, rz) = right_vec(start_h);
            // Compute yellow spine endpoints (5.25m left of lane 1 outbound)
            let sp_ax = nodes[p_a_out].x - rx * CARRIAGEWAY_OFFSET_M;
            let sp_az = nodes[p_a_out].z - rz * CARRIAGEWAY_OFFSET_M;
            let sp_bx = nodes[p_b_in].x + rx * CARRIAGEWAY_OFFSET_M;
            let sp_bz = nodes[p_b_in].z + rz * CARRIAGEWAY_OFFSET_M;

            let span = ((sp_bx - sp_ax).powi(2) + (sp_bz - sp_az).powi(2)).sqrt();
            let theta = wave_deg.to_radians();
            let wave_dx = 4.0 * r_wave * theta.sin();
            let rem_span = (span - 2.0 * r_corner - wave_dx).max(30.0);
            let s_seg = rem_span / 3.0;

            let build_carriageway = |from_p: (f32, f32), h0: f32, is_fwd: bool| -> Vec<GeometryPiece> {
                let mut list = Vec::new();
                let mut cx = from_p.0;
                let mut cz = from_p.1;
                let mut ch = normalize_angle_rad(h0);

                let adv_str = |list: &mut Vec<GeometryPiece>,
                               cx: &mut f32,
                               cz: &mut f32,
                               ch: f32,
                               len: f32| {
                    let pc = GeometryPiece::straight(*cx, *cz, ch, len);
                    let end = pc.sample(pc.length());
                    *cx = end.0;
                    *cz = end.1;
                    list.push(pc);
                };
                let adv_arc = |list: &mut Vec<GeometryPiece>,
                               cx: &mut f32,
                               cz: &mut f32,
                               ch: &mut f32,
                               r_spine: f32,
                               d_ang: f32| {
                    // Carriageway sits +5.25m right of travel direction: right turn radius = r - 5.25, left = r + 5.25
                    let r_eff = if d_ang >= 0.0 {
                        r_spine - CARRIAGEWAY_OFFSET_M
                    } else {
                        r_spine + CARRIAGEWAY_OFFSET_M
                    };
                    let pc = GeometryPiece::arc(*cx, *cz, *ch, r_eff, d_ang);
                    let end = pc.sample(pc.length());
                    *cx = end.0;
                    *cz = end.1;
                    *ch = end.2;
                    list.push(pc);
                };

                let turn_c = if is_fwd { FRAC_PI_2 } else { -FRAC_PI_2 };
                let s1 = if is_fwd { -theta } else { theta };
                adv_str(&mut list, &mut cx, &mut cz, ch, straight_out);
                adv_arc(&mut list, &mut cx, &mut cz, &mut ch, r_corner, turn_c);
                adv_str(&mut list, &mut cx, &mut cz, ch, s_seg);
                adv_arc(&mut list, &mut cx, &mut cz, &mut ch, r_wave, s1);
                adv_arc(&mut list, &mut cx, &mut cz, &mut ch, r_wave, -s1);
                adv_str(&mut list, &mut cx, &mut cz, ch, s_seg);
                adv_arc(&mut list, &mut cx, &mut cz, &mut ch, r_wave, -s1);
                adv_arc(&mut list, &mut cx, &mut cz, &mut ch, r_wave, s1);
                adv_str(&mut list, &mut cx, &mut cz, ch, s_seg);
                adv_arc(&mut list, &mut cx, &mut cz, &mut ch, r_corner, turn_c);
                adv_str(&mut list, &mut cx, &mut cz, ch, straight_out);
                list
            };

            let fwd_pieces =
                build_carriageway((nodes[p_a_out].x, nodes[p_a_out].z), start_h, true);
            let fwd_len: f32 = fwd_pieces.iter().map(|p| p.length()).sum();
            let e_fwd = add_edge(
                &nodes,
                p_a_out,
                p_b_in,
                fwd_pieces,
                RoadFeatureKind::CurveArch,
                true,
                0,
                16.5,
            );

            let p_b_out = ports[col_b][row_b][dir_b].1;
            let p_a_in = ports[col_a][row_a][dir_a].0;
            let rev_pieces =
                build_carriageway((nodes[p_b_out].x, nodes[p_b_out].z), start_h, false);
            let e_rev = add_edge(
                &nodes,
                p_b_out,
                p_a_in,
                rev_pieces,
                RoadFeatureKind::CurveArch,
                true,
                0,
                16.5,
            );
            (e_fwd, e_rev, fwd_len)
        };

        // Vary S-arch geometry safely per seed
        let north_deg = rng.gen_range(26.0, 34.0);
        let south_deg = rng.gen_range(26.0, 34.0);
        let east_deg = rng.gen_range(20.0, 28.0);
        let west_deg = rng.gen_range(20.0, 28.0);

        // 1. North Scenic S-Arch Freeway: (col=0,row=2 North) -> (col=3,row=2 North)
        let (n_fwd, _n_rev, n_len) =
            add_scenic_s_arch(0, 2, 0, 3, 2, 0, 0.0, 55.0, 115.0, 125.0, north_deg);
        // 2. East Express Arch: (col=3,row=2 East) -> (col=3,row=0 East)
        let (e_fwd, _e_rev, e_len) =
            add_scenic_s_arch(3, 2, 1, 3, 0, 1, FRAC_PI_2, 50.0, 110.0, 115.0, east_deg);
        // 3. South Coastal S-Arch: (col=3,row=0 South) -> (col=0,row=0 South)
        let (s_fwd, _s_rev, s_len) =
            add_scenic_s_arch(3, 0, 2, 0, 0, 2, PI, 55.0, 115.0, 125.0, south_deg);
        // 4. West Ridge Arch: (col=0,row=0 West) -> (col=0,row=2 West)
        let (w_fwd, _w_rev, w_len) =
            add_scenic_s_arch(0, 0, 3, 0, 2, 3, -FRAC_PI_2, 50.0, 110.0, 115.0, west_deg);

        // Highway On/Off Diagonal Connector Ramp bypasses in safe outer quadrants (no overlaps!)
        // E.g. NW Outer Merge Bypass or South-Central Express Connector? Already all 4 scenic loops
        // + interior grid form ~15 km of non-overlapping connected 6-lane roadways!
        unique_corridor_len += n_len + e_len + s_len + w_len;

        // Build outgoing adjacency list
        let mut outgoing = vec![Vec::new(); nodes.len()];
        for e in &edges {
            outgoing[e.from_node].push(e.id);
        }

        let mut net = RoadNetwork {
            seed,
            nodes,
            edges,
            outgoing,
            junctions,
            grand_tour_edges: Vec::new(),
            grand_tour_length_m: 0.0,
            unique_corridor_length_m: unique_corridor_len,
            bounds: (-1050.0, 1050.0, -820.0, 820.0),
            is_single_lane_hd: false,
            waymo_visuals: None,
        };

        // Construct a single deterministic closed >= 10,800 m (11+ km) Grand Tour traversing:
        // Grid Straights, Intersections (turns + straight), Roundabouts, & ALL 4 Perimeter S-Arches!
        let find_edge = |from_n: usize, to_n: usize| -> usize {
            net.outgoing[from_n]
                .iter()
                .copied()
                .find(|&eid| net.edges[eid].to_node == to_n)
                .expect("expected valid grand tour transition")
        };

        let append_via_junction = |net: &RoadNetwork, tour: &mut Vec<usize>, in_edge: usize, next_edge: usize| {
            let from_j = net.edges[in_edge].to_node;
            let to_j = net.edges[next_edge].from_node;
            // Shortest sequence across junction connectors (1 edge for 4-way, 2-4 edges inside roundabout ring)
            let path = net.shortest_junction_bridge(from_j, to_j);
            tour.extend(path);
            tour.push(next_edge);
        };

        // Grand Tour Sequence (closed loop returning seamlessly onto start edge v_edges[0][0].0):
        // Phase A (West->North Scenic + Top Grid):
        //   (0,0)->(0,1)->(0,2) [V] -> North S-Arch -> (3,2) -> West to (2,2)->(1,2)->(0,2) [H]
        //   Wait let's route cleanly through ALL rows & cols + ALL 4 arches so tour >= 11 km:
        let backbone = [
            v_edges[0][0].0, // (0,0)->(0,1) N
            v_edges[0][1].0, // (0,1)->(0,2) N
            n_fwd,           // North S-Arch: (0,2)N -> (3,2)N -> Southbound into (3,2)
            v_edges[3][1].1, // (3,2)->(3,1) S
            h_edges[1][2].1, // (3,1)->(2,1) W (enters central roundabout/intersection!)
            h_edges[1][1].1, // (2,1)->(1,1) W (enters central roundabout/intersection!)
            h_edges[1][0].1, // (1,1)->(0,1) W
            v_edges[0][0].1, // (0,1)->(0,0) S
            h_edges[0][0].0, // (0,0)->(1,0) E
            h_edges[0][1].0, // (1,0)->(2,0) E
            v_edges[2][0].0, // (2,0)->(2,1) N
            v_edges[2][1].0, // (2,1)->(2,2) N
            h_edges[2][1].1, // (2,2)->(1,2) W
            v_edges[1][1].1, // (1,2)->(1,1) S
            v_edges[1][0].1, // (1,1)->(1,0) S
            h_edges[0][1].0, // (1,0)->(2,0) E
            h_edges[0][2].0, // (2,0)->(3,0) E
            s_fwd,           // South S-Arch: (3,0)S -> (0,0)S -> Northbound into (0,0)
            w_fwd,           // West Arch: (0,0)W -> (0,2)W -> Eastbound into (0,2)
            h_edges[2][0].0, // (0,2)->(1,2) E
            h_edges[2][1].0, // (1,2)->(2,2) E
            h_edges[2][2].0, // (2,2)->(3,2) E
            e_fwd,           // East Arch: (3,2)E -> (3,0)E -> Westbound into (3,0)
            h_edges[0][2].1, // (3,0)->(2,0) W
            h_edges[0][1].1, // (2,0)->(1,0) W
            h_edges[0][0].1, // (1,0)->(0,0) W (turns North onto start v_edges[0][0].0!)
        ];
        let _ = find_edge;
        let mut tour = Vec::new();
        tour.push(backbone[0]);
        for &nxt in &backbone[1..] {
            let prev = *tour.last().unwrap();
            append_via_junction(&net, &mut tour, prev, nxt);
        }
        // Close final bridge back to tour[0] cleanly
        let last_edge = *tour.last().unwrap();
        let closing_bridge =
            net.shortest_junction_bridge(net.edges[last_edge].to_node, net.edges[tour[0]].from_node);
        tour.extend(closing_bridge);

        let tour_len: f32 = tour.iter().map(|&eid| net.edges[eid].total_length).sum();
        net.grand_tour_edges = tour;
        net.grand_tour_length_m = tour_len;

        // Compute exact tight world bounding box
        let mut bx0 = f32::MAX;
        let mut bx1 = f32::MIN;
        let mut bz0 = f32::MAX;
        let mut bz1 = f32::MIN;
        for e in &net.edges {
            bx0 = bx0.min(e.bbox_min_x);
            bx1 = bx1.max(e.bbox_max_x);
            bz0 = bz0.min(e.bbox_min_z);
            bz1 = bz1.max(e.bbox_max_z);
        }
        net.bounds = (bx0 - 30.0, bx1 + 30.0, bz0 - 30.0, bz1 + 30.0);
        net
    }
}

impl RoadNetwork {
    fn shortest_junction_bridge(&self, start_node: usize, target_node: usize) -> Vec<usize> {
        if start_node == target_node {
            return Vec::new();
        }
        let mut queue = std::collections::VecDeque::new();
        let mut prev_edge: HashMap<usize, (usize, usize)> = HashMap::new(); // node -> (prev_node, edge_id)
        queue.push_back(start_node);
        let mut found = false;

        while let Some(u) = queue.pop_front() {
            if u == target_node {
                found = true;
                break;
            }
            for &eid in &self.outgoing[u] {
                let e = &self.edges[eid];
                if e.feature.is_physical_corridor() {
                    continue; // Stay inside local junction/roundabout ring
                }
                let v = e.to_node;
                if v != start_node && !prev_edge.contains_key(&v) {
                    prev_edge.insert(v, (u, eid));
                    queue.push_back(v);
                }
            }
        }

        if !found {
            return Vec::new();
        }
        let mut res = Vec::new();
        let mut cur = target_node;
        while cur != start_node {
            let (pnode, eid) = prev_edge[&cur];
            res.push(eid);
            cur = pnode;
        }
        res.reverse();
        res
    }

    /// Build a connected, A*-routable single-lane HD `RoadNetwork` directly from an exported ScenarioNet Waymo scene.
    pub fn from_waymo_scenario(sc: &WaymoScenario) -> Self {
        let mut nodes: Vec<GraphNode> = Vec::new();
        let mut edges: Vec<RoadSegment> = Vec::new();
        let mut lane_id_to_edge: HashMap<String, usize> = HashMap::new();

        // Spatial deduplicating node allocator so connected lanes share exact graph vertices
        let get_or_add_node = |nodes: &mut Vec<GraphNode>, x: f32, z: f32| -> usize {
            for (i, n) in nodes.iter().enumerate() {
                if ((n.x - x).powi(2) + (n.z - z).powi(2)).sqrt() <= 1.25 {
                    return i;
                }
            }
            let id = nodes.len();
            nodes.push(GraphNode { x, z });
            id
        };

        let drivable: Vec<&crate::scenario_loader::WaymoLane> = sc
            .lanes
            .iter()
            .filter(|l| !l.is_bike_lane && l.polyline.len() >= 2)
            .collect();

        // First allocate endpoints & edges for each drivable Waymo lane
        let mut lane_ports: Vec<(usize, usize)> = Vec::with_capacity(drivable.len());
        let mut raw_pieces: Vec<GeometryPiece> = Vec::with_capacity(drivable.len());
        for lane in &drivable {
            let piece = GeometryPiece::from_polyline(&lane.polyline);
            let p0 = piece.sample(0.0);
            let p1 = piece.sample(piece.length());
            let fn_id = get_or_add_node(&mut nodes, p0.0, p0.1);
            let tn_id = get_or_add_node(&mut nodes, p1.0, p1.1);
            lane_ports.push((fn_id, tn_id));
            raw_pieces.push(piece);
        }

        // Unify topological `exit_lanes` -> `entry_lanes` connections if endpoints drifted > 1.25 m
        let mut id_to_raw_idx: HashMap<&str, usize> = HashMap::new();
        for (idx, lane) in drivable.iter().enumerate() {
            id_to_raw_idx.insert(lane.id.as_str(), idx);
        }
        for (i, lane) in drivable.iter().enumerate() {
            let from_end_node = lane_ports[i].1;
            let p_end = raw_pieces[i].sample(raw_pieces[i].length());
            for exit_id in &lane.exit_lanes {
                if let Some(&j) = id_to_raw_idx.get(exit_id.as_str()) {
                    let p_start = raw_pieces[j].sample(0.0);
                    let gap = ((p_end.0 - p_start.0).powi(2) + (p_end.1 - p_start.1).powi(2)).sqrt();
                    if gap < 4.5 && lane_ports[j].0 != from_end_node {
                        lane_ports[j].0 = from_end_node;
                    }
                }
            }
        }

        let mut unique_len = 0.0f32;
        for (i, (lane, piece)) in drivable.into_iter().zip(raw_pieces.into_iter()).enumerate() {
            let (fn_id, tn_id) = lane_ports[i];
            let eid = edges.len();
            unique_len += piece.length();
            lane_id_to_edge.insert(lane.id.clone(), eid);
            edges.push(RoadSegment::new(
                eid,
                fn_id,
                tn_id,
                vec![piece],
                RoadFeatureKind::WaymoLane,
                false,
                0,
                lane.speed_limit_mps.clamp(6.0, 22.0),
            ));
        }

        // Also build primary Waymo SDC reference corridor if valid so initial drive follows exact scene corridor smoothly
        let ref_piece = GeometryPiece::from_polyline(&sc.ego.reference_polyline);
        let p_ref0 = ref_piece.sample(0.0);
        let p_ref1 = ref_piece.sample(ref_piece.length());
        let r_from = get_or_add_node(&mut nodes, p_ref0.0, p_ref0.1);
        let r_to = get_or_add_node(&mut nodes, p_ref1.0, p_ref1.1);
        let ref_eid = edges.len();
        edges.push(RoadSegment::new(
            ref_eid,
            r_from,
            r_to,
            vec![ref_piece],
            RoadFeatureKind::WaymoLane,
            false,
            0,
            14.5,
        ));

        let mut outgoing = vec![Vec::new(); nodes.len()];
        for e in &edges {
            outgoing[e.from_node].push(e.id);
        }

        // Build primary cruise sequence along Waymo HD graph starting near ego's start pose, falling back to ref_eid
        let mut tour_edges = Vec::new();
        let mut best_start_eid = ref_eid;
        let mut best_score = f32::MAX;
        for e in &edges {
            if e.id == ref_eid {
                continue;
            }
            let (s, lat, dist) = e.project_world(sc.ego.start_pose[0], sc.ego.start_pose[1]);
            let (_, _, eh, _) = e.sample_ref(s);
            let dh = normalize_angle_rad(eh - sc.ego.start_pose[2]).abs();
            if lat.abs() < 2.2 && dh < 0.45 && dist + dh * 4.0 < best_score {
                best_score = dist + dh * 4.0;
                best_start_eid = e.id;
            }
        }

        if best_start_eid != ref_eid {
            // Walk downstream toward ego.goal_pose along connected HD edges
            tour_edges.push(best_start_eid);
            let mut visited = std::collections::HashSet::new();
            visited.insert(best_start_eid);
            for _ in 0..8 {
                let last_e = &edges[*tour_edges.last().unwrap()];
                let gx = sc.ego.goal_pose[0];
                let gz = sc.ego.goal_pose[1];
                let (_, _, end_h, _) = last_e.sample_ref(last_e.total_length);
                let mut next_best: Option<(usize, f32)> = None;
                for &cand in &outgoing[last_e.to_node] {
                    if cand == ref_eid || visited.contains(&cand) {
                        continue;
                    }
                    let ce = &edges[cand];
                    let (_, _, sh, _) = ce.sample_ref(0.0);
                    let dh = normalize_angle_rad(sh - end_h).abs();
                    if dh > 1.25 {
                        continue;
                    }
                    let (emx, emz, _, _) = ce.sample_ref(ce.total_length * 0.5);
                    let goal_d = ((emx - gx).powi(2) + (emz - gz).powi(2)).sqrt();
                    let cost = goal_d + dh * 30.0;
                    if next_best.map(|b| cost < b.1).unwrap_or(true) {
                        next_best = Some((cand, cost));
                    }
                }
                if let Some((nid, _)) = next_best {
                    visited.insert(nid);
                    tour_edges.push(nid);
                } else {
                    break;
                }
            }
        }

        let tour_len: f32 = tour_edges.iter().map(|&id| edges[id].total_length).sum();
        if tour_edges.is_empty() || tour_len < edges[ref_eid].total_length * 0.65 {
            tour_edges.clear();
            tour_edges.push(ref_eid);
        }
        let tour_len: f32 = tour_edges.iter().map(|&id| edges[id].total_length).sum();

        let mut bx0 = f32::MAX;
        let mut bx1 = f32::MIN;
        let mut bz0 = f32::MAX;
        let mut bz1 = f32::MIN;
        for e in &edges {
            bx0 = bx0.min(e.bbox_min_x);
            bx1 = bx1.max(e.bbox_max_x);
            bz0 = bz0.min(e.bbox_min_z);
            bz1 = bz1.max(e.bbox_max_z);
        }

        RoadNetwork {
            seed: 0,
            nodes,
            edges,
            outgoing,
            junctions: Vec::new(),
            grand_tour_edges: tour_edges,
            grand_tour_length_m: tour_len.max(10.0),
            unique_corridor_length_m: unique_len.max(tour_len),
            bounds: (bx0 - 20.0, bx1 + 20.0, bz0 - 20.0, bz1 + 20.0),
            is_single_lane_hd: true,
            waymo_visuals: Some(WaymoVisualGeometry {
                scenario_id: sc.scenario_id.clone(),
                road_lines: sc.road_lines.clone(),
                crosswalks: sc.crosswalks.clone(),
                stop_signs: sc.stop_signs.clone(),
                lane_id_to_edge,
                ego_start_pose: sc.ego.start_pose,
                ego_goal_pose: sc.ego.goal_pose,
            }),
        }
    }
}
