//! Perception scanner formatting NPCs, trucks, pedestrians, and red/yellow traffic-light
//! stop lines within 85 m in QNX coordinates, with toggleable seeded noise & LOS occlusion.

use crate::coordinate_mapper::{normalize_angle_rad, project_relative, DeterministicRng};
use crate::qnx_client::TelemetryObstacle;
use crate::traffic::{LightColor, TrafficDirector};

#[derive(Debug, Clone)]
struct Candidate {
    obs: TelemetryObstacle,
    dist: f32,
    azimuth: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct SignalHudInfo {
    pub color: LightColor,
    pub distance_m: f32,
    pub countdown_s: f32,
}

pub struct PerceptionEngine {
    pub max_range_m: f32,
    pub noise_enabled: bool,
    pub max_obstacles: usize,
    rng: DeterministicRng,
    pub last_signal_hud: Option<SignalHudInfo>,
    pub last_stop_line: Option<TelemetryObstacle>,
}

impl PerceptionEngine {
    pub fn new(max_range_m: f32, noise_enabled: bool, seed: u64) -> Self {
        Self {
            max_range_m,
            noise_enabled,
            max_obstacles: 22,
            rng: DeterministicRng::new(seed ^ 0xA115_BEEF),
            last_signal_hud: None,
            last_stop_line: None,
        }
    }

    pub fn reseed(&mut self, seed: u64) {
        self.rng.reseed(seed ^ 0xA115_BEEF);
    }

    pub fn scan(
        &mut self,
        director: &TrafficDirector,
        ego_x: f32,
        ego_z: f32,
        ego_heading: f32,
        ego_speed: f32,
    ) -> Vec<TelemetryObstacle> {
        self.last_signal_hud = None;
        self.last_stop_line = None;
        let mut cands: Vec<Candidate> = Vec::with_capacity(32);

        let sin_h = ego_heading.sin();
        let cos_h = ego_heading.cos();

        // 1. Scan NPC Vehicles (Cars/SUVs type=0, Trucks type=1)
        for v in &director.vehicles {
            let dx = v.x - ego_x;
            let dz = v.z - ego_z;
            let dist = (dx * dx + dz * dz).sqrt();
            if dist > self.max_range_m {
                continue;
            }
            let vx = v.speed * v.heading.sin();
            let vz = v.speed * v.heading.cos();
            let speed_along_ego = vx * sin_h + vz * cos_h;
            cands.push(Candidate {
                obs: TelemetryObstacle {
                    x: v.x,
                    z: v.z,
                    speed: speed_along_ego,
                    vx,
                    vz,
                    obs_type: v.class.qnx_type(),
                },
                dist,
                azimuth: dx.atan2(dz),
            });
        }

        // 2. Scan Pedestrians & Cyclists (type=2 VRU)
        for p in &director.pedestrians {
            let dx = p.x - ego_x;
            let dz = p.z - ego_z;
            let dist = (dx * dx + dz * dz).sqrt();
            if dist > self.max_range_m {
                continue;
            }
            let speed_along_ego = p.vx * sin_h + p.vz * cos_h;
            cands.push(Candidate {
                obs: TelemetryObstacle {
                    x: p.x,
                    z: p.z,
                    speed: speed_along_ego,
                    vx: p.vx,
                    vz: p.vz,
                    obs_type: 2,
                },
                dist,
                azimuth: dx.atan2(dz),
            });
        }
        for cyc in &director.cyclists {
            let dx = cyc.x - ego_x;
            let dz = cyc.z - ego_z;
            let dist = (dx * dx + dz * dz).sqrt();
            if dist > self.max_range_m {
                continue;
            }
            let speed_along_ego = cyc.vx * sin_h + cyc.vz * cos_h;
            cands.push(Candidate {
                obs: TelemetryObstacle {
                    x: cyc.x,
                    z: cyc.z,
                    speed: speed_along_ego,
                    vx: cyc.vx,
                    vz: cyc.vz,
                    obs_type: 2,
                },
                dist,
                azimuth: dx.atan2(dz),
            });
        }

        // 3. Match forward Traffic Light approach along ego's travel corridor (< 75 m)
        let mut best_hud_fwd = f32::INFINITY;
        let mut best_stop_fwd = f32::INFINITY;
        let mut best_stop: Option<Candidate> = None;

        for (app, color, countdown) in director.iter_signals() {
            let (fwd, lat) = project_relative(ego_x, ego_z, ego_heading, app.stop_x, app.stop_z);
            if fwd < 1.0 || fwd > 75.0 || lat.abs() > 8.5 {
                continue;
            }
            let dh = normalize_angle_rad(app.heading - ego_heading).abs();
            if dh > 45.0f32.to_radians() {
                continue;
            }
            if fwd < best_hud_fwd {
                best_hud_fwd = fwd;
                self.last_signal_hud = Some(SignalHudInfo {
                    color,
                    distance_m: fwd,
                    countdown_s: countdown,
                });
            }

            if matches!(color, LightColor::Red | LightColor::Yellow) {
                // Dilemma-zone filter for late Yellow lights
                if color == LightColor::Yellow {
                    let stop_dist = (ego_speed * ego_speed) / (2.0 * 3.5);
                    if fwd < stop_dist + 2.5 {
                        continue;
                    }
                }
                if fwd < best_stop_fwd {
                    best_stop_fwd = fwd;
                    let dx = app.stop_x - ego_x;
                    let dz = app.stop_z - ego_z;
                    let dist = (dx * dx + dz * dz).sqrt();
                    // Project stop bar cleanly onto ego center axis ahead so ForwardRadar sees exact distance regardless of lane 0/1/2
                    let bar_x = ego_x + fwd * sin_h;
                    let bar_z = ego_z + fwd * cos_h;
                    best_stop = Some(Candidate {
                        obs: TelemetryObstacle {
                            x: bar_x,
                            z: bar_z,
                            speed: 0.0,
                            vx: 0.0,
                            vz: 0.0,
                            obs_type: 3,
                        },
                        dist,
                        azimuth: ego_heading,
                    });
                }
            }
        }

        if let Some(stop_cand) = best_stop {
            self.last_stop_line = Some(stop_cand.obs);
            cands.push(stop_cand);
        }

        cands.sort_by(|a, b| a.dist.partial_cmp(&b.dist).unwrap_or(std::cmp::Ordering::Equal));

        if !self.noise_enabled {
            return cands
                .into_iter()
                .take(self.max_obstacles)
                .map(|c| c.obs)
                .collect();
        }

        // Seeded LOS occlusion + Gaussian sensor noise when enabled ('N')
        let mut visible: Vec<Candidate> = Vec::with_capacity(cands.len());
        for cand in cands {
            if cand.obs.obs_type == 3 {
                visible.push(cand);
                continue;
            }
            let occluded = visible.iter().any(|c| {
                c.obs.obs_type != 3
                    && (cand.dist - c.dist) > 2.5
                    && normalize_angle_rad(cand.azimuth - c.azimuth).abs() < 0.055
            });
            if !occluded {
                visible.push(cand);
            }
        }

        let mut out = Vec::with_capacity(self.max_obstacles.min(visible.len()));
        for mut c in visible.into_iter().take(self.max_obstacles) {
            if c.obs.obs_type != 3 {
                let sigma_p = 0.03 + 0.004 * c.dist;
                let sigma_v = 0.12;
                c.obs.x += self.rng.gauss(0.0, sigma_p);
                c.obs.z += self.rng.gauss(0.0, sigma_p);
                c.obs.speed += self.rng.gauss(0.0, sigma_v);
                c.obs.vx += self.rng.gauss(0.0, sigma_v);
                c.obs.vz += self.rng.gauss(0.0, sigma_v);
            }
            out.push(c.obs);
        }
        out
    }
}
