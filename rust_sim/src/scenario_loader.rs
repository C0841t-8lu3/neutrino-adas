//! Offline-exported ScenarioNet (Waymo / nuScenes / nuPlan) JSON loader and
//! converter into `RoadNetwork` HD lane graphs + dynamic replay data.

use serde::Deserialize;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

#[derive(Debug, Clone, Deserialize)]
pub struct WaymoScenario {
    pub scenario_id: String,
    pub dataset: String,
    pub length: usize,
    pub dt_s: f32,
    pub origin_xy: [f32; 2],
    pub ego: WaymoEgo,
    pub lanes: Vec<WaymoLane>,
    pub road_lines: Vec<WaymoRoadLine>,
    pub crosswalks: Vec<WaymoCrosswalk>,
    pub stop_signs: Vec<WaymoStopSign>,
    pub traffic_lights: Vec<WaymoTrafficLight>,
    pub tracks: Vec<WaymoTrack>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct WaymoEgo {
    pub id: String,
    pub length_m: f32,
    pub width_m: f32,
    pub start_pose: [f32; 4], // [x, z, qnx_heading, speed]
    pub goal_pose: [f32; 3],  // [x, z, qnx_heading]
    pub reference_polyline: Vec<[f32; 2]>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct WaymoLane {
    pub id: String,
    pub is_bike_lane: bool,
    pub polyline: Vec<[f32; 2]>,
    pub width_m: f32,
    pub speed_limit_mps: f32,
    pub entry_lanes: Vec<String>,
    pub exit_lanes: Vec<String>,
    pub left_neighbor: Vec<String>,
    pub right_neighbor: Vec<String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct WaymoRoadLine {
    pub id: String,
    pub line_type: String,
    pub polyline: Vec<[f32; 2]>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct WaymoCrosswalk {
    pub id: String,
    pub polygon: Vec<[f32; 2]>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct WaymoStopSign {
    pub id: String,
    pub pos: [f32; 2],
    pub lanes: Vec<String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct WaymoTrafficLight {
    pub id: String,
    pub stop_point: [f32; 2],
    pub lane_id: String,
    pub states: Vec<String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct WaymoTrack {
    pub id: String,
    pub obj_type: String,
    pub length_m: f32,
    pub width_m: f32,
    pub frames: Vec<WaymoTrackFrame>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct WaymoTrackFrame {
    pub t: usize,
    pub x: f32,
    pub z: f32,
    pub h: f32,
    pub spd: f32,
}

impl WaymoScenario {
    pub fn duration_s(&self) -> f32 {
        (self.length.saturating_sub(1) as f32) * self.dt_s.max(0.05)
    }

    pub fn interpolate_track(&self, track: &WaymoTrack, t_s: f32) -> Option<(f32, f32, f32, f32)> {
        if track.frames.is_empty() {
            return None;
        }
        let dt = self.dt_s.max(0.05);
        let t_idx_f = (t_s / dt).clamp(0.0, (self.length.saturating_sub(1)) as f32);
        let t_lo = t_idx_f.floor() as usize;
        let t_hi = t_idx_f.ceil() as usize;

        let p0 = track.frames.partition_point(|f| f.t < t_lo);
        let f0 = track.frames.get(p0)?;
        if f0.t != t_lo {
            // Track not valid yet at t_lo (check immediate boundary within 1 frame)
            return None;
        }
        if t_hi == t_lo {
            return Some((f0.x, f0.z, f0.h, f0.spd));
        }
        if let Some(f1) = track.frames.get(p0 + 1) {
            if f1.t == t_hi {
                let alpha = (t_idx_f - t_lo as f32).clamp(0.0, 1.0);
                let x = f0.x + (f1.x - f0.x) * alpha;
                let z = f0.z + (f1.z - f0.z) * alpha;
                let mut dh = f1.h - f0.h;
                while dh > std::f32::consts::PI {
                    dh -= std::f32::consts::TAU;
                }
                while dh <= -std::f32::consts::PI {
                    dh += std::f32::consts::TAU;
                }
                let h = crate::coordinate_mapper::normalize_angle_rad(f0.h + dh * alpha);
                let spd = f0.spd + (f1.spd - f0.spd) * alpha;
                return Some((x, z, h, spd));
            }
        }
        Some((f0.x, f0.z, f0.h, f0.spd))
    }
}

pub fn load_scenarios_from_dir(dir: &Path) -> std::io::Result<Vec<WaymoScenario>> {
    let mut paths = Vec::new();
    for entry in std::fs::read_dir(dir)? {
        let entry = entry?;
        let p = entry.path();
        if p.is_file() && p.extension().and_then(|s| s.to_str()) == Some("json") {
            paths.push(p);
        }
    }
    paths.sort();

    let mut out = Vec::with_capacity(paths.len());
    for p in paths {
        let f = File::open(&p)?;
        let sc: WaymoScenario = serde_json::from_reader(BufReader::new(f)).map_err(|e| {
            std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("Failed parsing scenario {}: {}", p.display(), e),
            )
        })?;
        out.push(sc);
    }
    Ok(out)
}
