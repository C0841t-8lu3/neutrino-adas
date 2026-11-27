//! Ring-buffer telemetry recorder, pro-driver accuracy tracker, and JSON/CSV exporter.

use crate::qnx_client::{AdasOutput, RttStats};
use serde::Serialize;
use std::collections::VecDeque;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;

#[derive(Debug, Clone, Serialize)]
pub struct StepRecord {
    pub tick: u64,
    pub sim_time_s: f64,
    pub speed_kmh: f32,
    pub lateral_error_m: f32,
    pub throttle: f32,
    pub brake: f32,
    pub steer_cmd_rad: f32,
    pub drive_mode: i32,
    pub drive_mode_name: String,
    pub drive_profile: i32,
    pub lead_distance_m: f32,
    pub ttc_s: f32,
    pub left_bsm: bool,
    pub right_bsm: bool,
    pub aeb_active: bool,
    pub watchdog_ok: bool,
    pub collision: bool,
    pub odometer_km: f64,
    pub rtt_us: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct SummaryReport {
    pub seed: u64,
    pub total_ticks: u64,
    pub odometer_km: f64,
    pub drive_time_s: f64,
    pub mean_speed_kmh: f64,
    pub max_speed_kmh: f32,
    pub rms_lateral_error_m: f64,
    pub max_abs_lateral_error_m: f32,
    pub min_ttc_s: f32,
    pub aeb_events: u32,
    pub bsm_events: u32,
    pub watchdog_trips: u32,
    pub collision_ticks: u32,
    pub qnx_rtt_p50_us: f64,
    pub qnx_rtt_p95_us: f64,
    pub qnx_rtt_max_us: f64,
}

#[derive(Debug, Clone, Serialize)]
struct JsonExport<'a> {
    summary: SummaryReport,
    steps: &'a VecDeque<StepRecord>,
}

pub struct TelemetryRecorder {
    capacity: usize,
    pub seed: u64,
    pub total_ticks: u64,
    pub cumulative_odometer_m: f64,
    pub cumulative_drive_time_s: f64,
    pub episode_odometer_m: f64,
    pub current_route_length_m: f64,
    pub episodes_completed: u32,
    pub rollovers: u32,

    pub max_speed_kmh: f32,
    speed_sum_kmh: f64,
    lat_err_sq_sum: f64,
    lat_err_samples: u64,
    pub max_abs_lat_err_m: f32,
    pub min_ttc_s: f32,
    pub aeb_events: u32,
    pub bsm_events: u32,
    pub watchdog_trips: u32,
    pub collision_ticks: u32,
    prev_aeb: bool,
    prev_bsm: bool,
    prev_wd: bool,

    pub records: VecDeque<StepRecord>,
}

impl TelemetryRecorder {
    pub fn new(seed: u64, route_length_m: f64, capacity: usize) -> Self {
        Self {
            capacity,
            seed,
            total_ticks: 0,
            cumulative_odometer_m: 0.0,
            cumulative_drive_time_s: 0.0,
            episode_odometer_m: 0.0,
            current_route_length_m: route_length_m.max(50.0),
            episodes_completed: 0,
            rollovers: 0,
            max_speed_kmh: 0.0,
            speed_sum_kmh: 0.0,
            lat_err_sq_sum: 0.0,
            lat_err_samples: 0,
            max_abs_lat_err_m: 0.0,
            min_ttc_s: 999.0,
            aeb_events: 0,
            bsm_events: 0,
            watchdog_trips: 0,
            collision_ticks: 0,
            prev_aeb: false,
            prev_bsm: false,
            prev_wd: true,
            records: VecDeque::with_capacity(capacity),
        }
    }

    pub fn record_step(
        &mut self,
        tick: u64,
        dt_s: f32,
        dist_step_m: f32,
        speed_mps: f32,
        lateral_error_m: f32,
        lane_change_active: bool,
        adas: &AdasOutput,
        collision: bool,
    ) {
        let speed_kmh = speed_mps * 3.6;
        self.total_ticks += 1;
        self.cumulative_drive_time_s += dt_s as f64;
        self.cumulative_odometer_m += dist_step_m.max(0.0) as f64;
        self.episode_odometer_m += dist_step_m.max(0.0) as f64;
        self.max_speed_kmh = self.max_speed_kmh.max(speed_kmh);
        self.speed_sum_kmh += speed_kmh as f64;

        // Track lane-centering RMS during normal lane keeping (exclude active 3.5m lane-change transitions)
        if !lane_change_active && speed_mps > 1.5 {
            let e = lateral_error_m as f64;
            self.lat_err_sq_sum += e * e;
            self.lat_err_samples += 1;
            self.max_abs_lat_err_m = self.max_abs_lat_err_m.max(lateral_error_m.abs());
        }

        if adas.time_to_collision > 0.0 && adas.time_to_collision < self.min_ttc_s {
            self.min_ttc_s = adas.time_to_collision;
        }

        let aeb = adas.drive_mode == 5;
        let bsm = adas.left_mirror_blind_spot || adas.right_mirror_blind_spot;
        if aeb && !self.prev_aeb {
            self.aeb_events += 1;
        }
        if bsm && !self.prev_bsm {
            self.bsm_events += 1;
        }
        if !adas.watchdog_ok && self.prev_wd {
            self.watchdog_trips += 1;
        }
        self.prev_aeb = aeb;
        self.prev_bsm = bsm;
        self.prev_wd = adas.watchdog_ok;

        if collision {
            self.collision_ticks += 1;
        }

        if self.records.len() >= self.capacity {
            self.records.pop_front();
        }
        self.records.push_back(StepRecord {
            tick,
            sim_time_s: (self.cumulative_drive_time_s * 1000.0).round() / 1000.0,
            speed_kmh,
            lateral_error_m,
            throttle: adas.throttle,
            brake: adas.brake,
            steer_cmd_rad: adas.steer_cmd,
            drive_mode: adas.drive_mode,
            drive_mode_name: adas.mode_name().to_string(),
            drive_profile: adas.drive_profile,
            lead_distance_m: adas.lead_distance,
            ttc_s: adas.time_to_collision,
            left_bsm: adas.left_mirror_blind_spot,
            right_bsm: adas.right_mirror_blind_spot,
            aeb_active: aeb,
            watchdog_ok: adas.watchdog_ok,
            collision,
            odometer_km: self.cumulative_odometer_m / 1000.0,
            rtt_us: adas.rtt_us,
        });
    }

    pub fn on_new_map(&mut self, seed: u64, route_length_m: f64, completed: bool) {
        self.seed = seed;
        self.current_route_length_m = route_length_m.max(50.0);
        self.episode_odometer_m = 0.0;
        self.rollovers += 1;
        if completed {
            self.episodes_completed += 1;
        }
    }

    pub fn rms_lateral_error_m(&self) -> f64 {
        if self.lat_err_samples == 0 {
            0.0
        } else {
            (self.lat_err_sq_sum / self.lat_err_samples as f64).sqrt()
        }
    }

    pub fn formatted_drive_time(&self) -> String {
        let secs = self.cumulative_drive_time_s.max(0.0) as u64;
        let mins = secs / 60;
        let rem = secs % 60;
        format!("{:02}:{:02}", mins, rem)
    }

    pub fn summary(&self, rtt: RttStats) -> SummaryReport {
        let mean_speed = if self.total_ticks > 0 {
            self.speed_sum_kmh / self.total_ticks as f64
        } else {
            0.0
        };
        SummaryReport {
            seed: self.seed,
            total_ticks: self.total_ticks,
            odometer_km: self.cumulative_odometer_m / 1000.0,
            drive_time_s: self.cumulative_drive_time_s,
            mean_speed_kmh: mean_speed,
            max_speed_kmh: self.max_speed_kmh,
            rms_lateral_error_m: self.rms_lateral_error_m(),
            max_abs_lateral_error_m: self.max_abs_lat_err_m,
            min_ttc_s: self.min_ttc_s,
            aeb_events: self.aeb_events,
            bsm_events: self.bsm_events,
            watchdog_trips: self.watchdog_trips,
            collision_ticks: self.collision_ticks,
            qnx_rtt_p50_us: rtt.p50_us,
            qnx_rtt_p95_us: rtt.p95_us,
            qnx_rtt_max_us: rtt.max_us,
        }
    }

    pub fn export_all(&self, prefix: &str, rtt: RttStats) -> std::io::Result<(PathBuf, PathBuf)> {
        let json_path = PathBuf::from(format!("{}.json", prefix));
        let csv_path = PathBuf::from(format!("{}.csv", prefix));

        let payload = JsonExport {
            summary: self.summary(rtt),
            steps: &self.records,
        };
        let jf = File::create(&json_path)?;
        serde_json::to_writer_pretty(BufWriter::new(jf), &payload)?;

        let cf = File::create(&csv_path)?;
        let mut cw = BufWriter::new(cf);
        writeln!(
            cw,
            "tick,sim_time_s,speed_kmh,lateral_error_m,throttle,brake,steer_cmd_rad,drive_mode,lead_distance_m,ttc_s,left_bsm,right_bsm,aeb_active,watchdog_ok,collision,odometer_km,rtt_us"
        )?;
        for r in &self.records {
            writeln!(
                cw,
                "{},{:.3},{:.2},{:.4},{:.3},{:.3},{:.4},{},{:.2},{:.2},{},{},{},{},{},{:.4},{:.1}",
                r.tick,
                r.sim_time_s,
                r.speed_kmh,
                r.lateral_error_m,
                r.throttle,
                r.brake,
                r.steer_cmd_rad,
                r.drive_mode,
                r.lead_distance_m,
                r.ttc_s,
                r.left_bsm as u8,
                r.right_bsm as u8,
                r.aeb_active as u8,
                r.watchdog_ok as u8,
                r.collision as u8,
                r.odometer_km,
                r.rtt_us
            )?;
        }
        Ok((json_path, csv_path))
    }
}
