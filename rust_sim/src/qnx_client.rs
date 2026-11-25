//! Persistent TCP_NODELAY QNX 8.0 Safety Kernel client with reusable zero-churn formatting,
//! microsecond p50/p95 RTT measurement, and 450 ms Watchdog Safe Fail-Stop enforcement.

use serde::Deserialize;
use std::collections::VecDeque;
use std::fmt::Write as _;
use std::io::{BufRead, BufReader, Write};
use std::net::{SocketAddr, TcpStream, ToSocketAddrs};
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Deserialize)]
pub struct AdasOutput {
    #[serde(default)]
    pub throttle: f32,
    #[serde(default)]
    pub brake: f32,
    #[serde(default)]
    pub steer_cmd: f32,
    #[serde(default)]
    pub left_turn_signal: bool,
    #[serde(default)]
    pub right_turn_signal: bool,
    #[serde(default)]
    pub hazard_lights: bool,
    #[serde(default)]
    pub horn_active: bool,
    #[serde(default)]
    pub brake_lights: bool,
    #[serde(default)]
    pub left_mirror_blind_spot: bool,
    #[serde(default)]
    pub right_mirror_blind_spot: bool,
    #[serde(default = "default_mode")]
    pub drive_mode: i32,
    #[serde(default = "default_profile")]
    pub drive_profile: i32,
    #[serde(default)]
    pub lateral_nudge: f32,
    #[serde(default = "default_lead")]
    pub lead_distance: f32,
    #[serde(default)]
    pub lead_rel_speed: f32,
    #[serde(default = "default_lead")]
    pub time_to_collision: f32,
    #[serde(default)]
    pub ack_seq: u64,
    #[serde(default = "default_true")]
    pub watchdog_ok: bool,
    #[serde(default)]
    pub status_message: String,
    #[serde(skip)]
    pub rtt_us: f64,
}

fn default_mode() -> i32 {
    1
}
fn default_profile() -> i32 {
    1
}
fn default_lead() -> f32 {
    999.0
}
fn default_true() -> bool {
    true
}

impl Default for AdasOutput {
    fn default() -> Self {
        Self {
            throttle: 0.0,
            brake: 0.0,
            steer_cmd: 0.0,
            left_turn_signal: false,
            right_turn_signal: false,
            hazard_lights: false,
            horn_active: false,
            brake_lights: false,
            left_mirror_blind_spot: false,
            right_mirror_blind_spot: false,
            drive_mode: 1,
            drive_profile: 1,
            lateral_nudge: 0.0,
            lead_distance: 999.0,
            lead_rel_speed: 0.0,
            time_to_collision: 999.0,
            ack_seq: 0,
            watchdog_ok: true,
            status_message: "INITIALIZING".to_string(),
            rtt_us: 0.0,
        }
    }
}

impl AdasOutput {
    pub fn mode_name(&self) -> &'static str {
        match self.drive_mode {
            0 => "MANUAL",
            1 => "CRUISING",
            2 => "ACC_FOLLOWING",
            3 => "LANE_CHANGING",
            4 => "TURNING_INTERSECTION",
            5 => "AEB_EMERGENCY",
            6 => "STOPPED",
            _ => "UNKNOWN",
        }
    }

    pub fn fail_stop(reason: &str, ack_seq: u64, rtt_us: f64) -> Self {
        Self {
            throttle: 0.0,
            brake: 1.0,
            steer_cmd: 0.0,
            left_turn_signal: false,
            right_turn_signal: false,
            hazard_lights: true,
            horn_active: true,
            brake_lights: true,
            left_mirror_blind_spot: false,
            right_mirror_blind_spot: false,
            drive_mode: 5,
            drive_profile: 1,
            lateral_nudge: 0.0,
            lead_distance: 999.0,
            lead_rel_speed: 0.0,
            time_to_collision: 999.0,
            ack_seq,
            watchdog_ok: false,
            status_message: reason.to_string(),
            rtt_us,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct TelemetryObstacle {
    pub x: f32,
    pub z: f32,
    pub speed: f32,
    pub vx: f32,
    pub vz: f32,
    pub obs_type: i32, // 0=car, 1=truck, 2=ped, 3=stop_line
}

#[derive(Debug, Clone)]
pub struct TelemetryPacket<'a> {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub speed: f32,
    pub heading: f32,
    pub steer_angle: f32,
    pub current_lane: i32,
    pub target_lane: i32,
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
    pub dt: f32,
    pub seq: u64,
    pub drive_profile: i32,
    pub obs: &'a [TelemetryObstacle],
}

#[derive(Debug, Clone, Copy, Default)]
pub struct RttStats {
    pub samples: usize,
    pub p50_us: f64,
    pub p95_us: f64,
    pub max_us: f64,
}

pub struct QnxTcpClient {
    host: String,
    port: u16,
    watchdog_timeout: Duration,
    reader: Option<BufReader<TcpStream>>,
    tx_buf: String,
    rx_buf: String,
    rtt_samples_us: VecDeque<f64>,
    last_connect_attempt: Option<Instant>,
    pub connected: bool,
}

impl QnxTcpClient {
    pub fn new(host: impl Into<String>, port: u16, watchdog_timeout_ms: u64) -> Self {
        Self {
            host: host.into(),
            port,
            watchdog_timeout: Duration::from_millis(watchdog_timeout_ms.max(20)),
            reader: None,
            tx_buf: String::with_capacity(4096),
            rx_buf: String::with_capacity(1024),
            rtt_samples_us: VecDeque::with_capacity(2048),
            last_connect_attempt: None,
            connected: false,
        }
    }

    pub fn connect(&mut self) -> bool {
        self.disconnect();
        self.last_connect_attempt = Some(Instant::now());

        let addr_str = format!("{}:{}", self.host, self.port);
        let addr: SocketAddr = match addr_str.to_socket_addrs().ok().and_then(|mut it| it.next()) {
            Some(a) => a,
            None => return false,
        };

        let stream = match TcpStream::connect_timeout(&addr, Duration::from_millis(350)) {
            Ok(s) => s,
            Err(_) => return false,
        };

        let _ = stream.set_nodelay(true);
        let _ = stream.set_read_timeout(Some(self.watchdog_timeout));
        let _ = stream.set_write_timeout(Some(self.watchdog_timeout));

        self.reader = Some(BufReader::new(stream));
        self.connected = true;
        true
    }

    pub fn disconnect(&mut self) {
        self.reader = None;
        self.connected = false;
    }

    pub fn format_telemetry_into_buf(&mut self, pkt: &TelemetryPacket<'_>) {
        self.tx_buf.clear();
        let _ = write!(
            &mut self.tx_buf,
            "{{\"x\":{:.3},\"y\":{:.3},\"z\":{:.3},\"speed\":{:.3},\"heading\":{:.4},\
             \"steer_angle\":{:.4},\"current_lane\":{},\"target_lane\":{},\"has_route\":{},\
             \"ref_x\":{:.3},\"ref_z\":{:.3},\"ref_heading\":{:.4},\"path_curvature\":{:.5},\
             \"next_curvature\":{:.5},\"dist_to_goal\":{:.1},\"turn_signal_hint\":{},\
             \"dist_to_next_turn\":{:.1},\"pre_turn_lane_hint\":{},\"in_intersection\":{},\
             \"dt\":{:.4},\"seq\":{},\"drive_profile\":{},\"obs\":[",
            pkt.x,
            pkt.y,
            pkt.z,
            pkt.speed,
            pkt.heading,
            pkt.steer_angle,
            pkt.current_lane,
            pkt.target_lane,
            pkt.has_route,
            pkt.ref_x,
            pkt.ref_z,
            pkt.ref_heading,
            pkt.path_curvature,
            pkt.next_curvature,
            pkt.dist_to_goal,
            pkt.turn_signal_hint,
            pkt.dist_to_next_turn,
            pkt.pre_turn_lane_hint,
            pkt.in_intersection,
            pkt.dt.clamp(0.002, 0.095),
            pkt.seq,
            pkt.drive_profile,
        );

        for (idx, o) in pkt.obs.iter().take(24).enumerate() {
            if idx > 0 {
                self.tx_buf.push(',');
            }
            let _ = write!(
                &mut self.tx_buf,
                "{{\"x\":{:.2},\"z\":{:.2},\"speed\":{:.2},\"vx\":{:.2},\"vz\":{:.2},\"type\":{}}}",
                o.x, o.z, o.speed, o.vx, o.vz, o.obs_type
            );
        }
        self.tx_buf.push_str("]}\n");
    }

    pub fn send_telemetry(&mut self, pkt: &TelemetryPacket<'_>) -> AdasOutput {
        if !self.connected {
            let should_retry = self
                .last_connect_attempt
                .map(|t| t.elapsed() >= Duration::from_secs(1))
                .unwrap_or(true);
            if !should_retry || !self.connect() {
                return AdasOutput::fail_stop(
                    "WATCHDOG SAFE FAIL-STOP: QNX 8.0 RTOS Unreachable",
                    pkt.seq,
                    0.0,
                );
            }
        }

        self.format_telemetry_into_buf(pkt);
        let t0 = Instant::now();

        let io_res: Result<usize, std::io::Error> = (|| {
            let reader = self
                .reader
                .as_mut()
                .ok_or_else(|| std::io::Error::other("no reader"))?;
            reader.get_mut().write_all(self.tx_buf.as_bytes())?;
            reader.get_mut().flush()?;
            self.rx_buf.clear();
            reader.read_line(&mut self.rx_buf)
        })();

        let elapsed_us = t0.elapsed().as_secs_f64() * 1_000_000.0;

        match io_res {
            Ok(0) => {
                self.disconnect();
                AdasOutput::fail_stop("WATCHDOG SAFE FAIL-STOP: QNX Link Closed", pkt.seq, elapsed_us)
            }
            Ok(_) => match serde_json::from_str::<AdasOutput>(self.rx_buf.trim()) {
                Ok(mut out) => {
                    out.rtt_us = elapsed_us;
                    if self.rtt_samples_us.len() >= 2000 {
                        self.rtt_samples_us.pop_front();
                    }
                    self.rtt_samples_us.push_back(elapsed_us);
                    out
                }
                Err(_) => {
                    self.disconnect();
                    AdasOutput::fail_stop(
                        "WATCHDOG SAFE FAIL-STOP: Malformed QNX Response",
                        pkt.seq,
                        elapsed_us,
                    )
                }
            },
            Err(err) => {
                self.disconnect();
                AdasOutput::fail_stop(
                    &format!("WATCHDOG SAFE FAIL-STOP ({}ms): {}", self.watchdog_timeout.as_millis(), err),
                    pkt.seq,
                    elapsed_us,
                )
            }
        }
    }

    pub fn rtt_stats(&self) -> RttStats {
        if self.rtt_samples_us.is_empty() {
            return RttStats::default();
        }
        let mut sorted: Vec<f64> = self.rtt_samples_us.iter().copied().collect();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let n = sorted.len();
        RttStats {
            samples: n,
            p50_us: sorted[((0.50 * (n.saturating_sub(1)) as f64) as usize).min(n - 1)],
            p95_us: sorted[((0.95 * (n.saturating_sub(1)) as f64) as usize).min(n - 1)],
            max_us: sorted[n - 1],
        }
    }
}
