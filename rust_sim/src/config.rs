//! Runtime configuration and CLI parser for the native Rust Bevy 3D QNX 8.0 ADAS simulator.

use std::fmt;

pub const DT_60HZ: f32 = 1.0 / 60.0;
pub const WHEELBASE_M: f32 = 2.75;
pub const LANE_WIDTH_M: f32 = 3.5;
pub const CARRIAGEWAY_LANES: usize = 3;
pub const MU_DRY: f32 = 0.95;
pub const MU_WET: f32 = 0.55;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewMode {
    View2D,
    View3D,
}

impl ViewMode {
    pub fn toggle(self) -> Self {
        match self {
            ViewMode::View2D => ViewMode::View3D,
            ViewMode::View3D => ViewMode::View2D,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            ViewMode::View2D => "2D FSD TOP-DOWN",
            ViewMode::View3D => "3D CHASE-CAM",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapMode {
    Mega10km,
    Waymo,
}

impl MapMode {
    pub fn label(self) -> &'static str {
        match self {
            MapMode::Mega10km => "MEGA-10KM",
            MapMode::Waymo => "WAYMO-SCENARIONET",
        }
    }

    pub fn from_str_name(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "mega10km" | "mega" | "procedural" => Some(MapMode::Mega10km),
            "waymo" | "scenarionet" | "scenario" => Some(MapMode::Waymo),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveProfile {
    Chill = 0,
    Standard = 1,
    Hurry = 2,
}

impl DriveProfile {
    pub fn id(self) -> i32 {
        self as i32
    }

    pub fn label(self) -> &'static str {
        match self {
            DriveProfile::Chill => "CHILL",
            DriveProfile::Standard => "STANDARD",
            DriveProfile::Hurry => "HURRY",
        }
    }

    pub fn cycle(self) -> Self {
        match self {
            DriveProfile::Chill => DriveProfile::Standard,
            DriveProfile::Standard => DriveProfile::Hurry,
            DriveProfile::Hurry => DriveProfile::Chill,
        }
    }

    pub fn from_str_name(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "chill" => Some(DriveProfile::Chill),
            "standard" => Some(DriveProfile::Standard),
            "hurry" => Some(DriveProfile::Hurry),
            _ => None,
        }
    }
}

impl fmt::Display for DriveProfile {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.label())
    }
}

#[derive(Debug, Clone)]
pub struct SimConfig {
    pub view: ViewMode,
    pub map_mode: MapMode,
    pub seed: u64,
    pub traffic_density: f32,
    pub qnx_host: String,
    pub qnx_port: u16,
    pub qnx_watchdog_timeout_ms: u64,
    pub profile: DriveProfile,
    pub noise: bool,
    pub wet_surface: bool,
    pub detection_radius_m: f32,
    pub report_prefix: String,
    pub scenarios_dir: String,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            view: ViewMode::View3D,
            map_mode: MapMode::Mega10km,
            seed: 1000,
            traffic_density: 0.20,
            qnx_host: "192.168.122.62".to_string(),
            qnx_port: 9090,
            qnx_watchdog_timeout_ms: 450,
            profile: DriveProfile::Standard,
            noise: false,
            wet_surface: false,
            detection_radius_m: 85.0,
            report_prefix: "qnx_rust_scorecard".to_string(),
            scenarios_dir: "assets/scenarios".to_string(),
        }
    }
}

impl SimConfig {
    pub fn current_friction(&self) -> f32 {
        if self.wet_surface {
            MU_WET
        } else {
            MU_DRY
        }
    }

    pub fn surface_label(&self) -> &'static str {
        if self.wet_surface {
            "WET"
        } else {
            "DRY"
        }
    }

    pub fn toggle_wet(&mut self) -> bool {
        self.wet_surface = !self.wet_surface;
        self.wet_surface
    }

    pub fn help_text() -> &'static str {
        "QNX 8.0 Native 60 FPS Bevy 3D ADAS Simulator\n\
         \n\
         USAGE:\n\
           qnx_rust_sim [OPTIONS]\n\
         \n\
         OPTIONS:\n\
           --map <mega10km|waymo>   Road environment mode (default: mega10km)\n\
           --view <3d|2d>           Initial view mode (default: 3d; press V to toggle live)\n\
           --seed <N>               Deterministic map/scenario seed (default: 1000)\n\
           --traffic-density <F>    NPC traffic intensity 0.0..0.5 in mega10km (default: 0.20)\n\
           --qnx-host <IP>          QNX 8.0 target IP address (default: 192.168.122.62)\n\
           --qnx-port <PORT>        QNX 8.0 TCP port (default: 9090)\n\
           --profile <chill|standard|hurry>  FSD driving profile (default: standard)\n\
           --noise                  Enable seeded radar Gaussian noise & LOS occlusion\n\
           --wet                    Start on low-friction wet asphalt (mu=0.55)\n\
           --scenarios-dir <PATH>   Directory of exported ScenarioNet JSONs (default: assets/scenarios)\n\
           --report-prefix <NAME>   Export filename prefix (default: qnx_rust_scorecard)\n\
           -h, --help               Print help message and exit\n"
    }

    /// Parse CLI flags cleanly; returns `Ok(None)` when `--help` was requested.
    pub fn from_args(args: &[String]) -> Result<Option<Self>, String> {
        let mut cfg = Self::default();
        let mut i = 0;
        while i < args.len() {
            match args[i].as_str() {
                "-h" | "--help" => {
                    return Ok(None);
                }
                "--view" => {
                    i += 1;
                    let val = args.get(i).ok_or("--view requires a value (3d or 2d)")?;
                    cfg.view = match val.to_ascii_lowercase().as_str() {
                        "2d" => ViewMode::View2D,
                        "3d" => ViewMode::View3D,
                        _ => return Err(format!("Unsupported --view '{}'; expected '3d' or '2d'", val)),
                    };
                }
                "--map" => {
                    i += 1;
                    let val = args.get(i).ok_or("--map requires a value (mega10km or waymo)")?;
                    cfg.map_mode = MapMode::from_str_name(val).ok_or_else(|| {
                        format!("Unsupported --map '{}'; expected 'mega10km' or 'waymo'", val)
                    })?;
                }
                "--seed" => {
                    i += 1;
                    let val = args.get(i).ok_or("--seed requires an integer")?;
                    cfg.seed = val
                        .parse::<u64>()
                        .map_err(|_| format!("Invalid --seed '{}'", val))?;
                }
                "--traffic-density" => {
                    i += 1;
                    let val = args.get(i).ok_or("--traffic-density requires a float")?;
                    let parsed = val
                        .parse::<f32>()
                        .map_err(|_| format!("Invalid --traffic-density '{}'", val))?;
                    cfg.traffic_density = parsed.clamp(0.0, 0.5);
                }
                "--qnx-host" => {
                    i += 1;
                    cfg.qnx_host = args
                        .get(i)
                        .ok_or("--qnx-host requires an IP or hostname")?
                        .clone();
                }
                "--qnx-port" => {
                    i += 1;
                    let val = args.get(i).ok_or("--qnx-port requires a port")?;
                    cfg.qnx_port = val
                        .parse::<u16>()
                        .map_err(|_| format!("Invalid --qnx-port '{}'", val))?;
                }
                "--profile" => {
                    i += 1;
                    let val = args.get(i).ok_or("--profile requires chill|standard|hurry")?;
                    cfg.profile = DriveProfile::from_str_name(val)
                        .ok_or_else(|| format!("Invalid --profile '{}'", val))?;
                }
                "--noise" => cfg.noise = true,
                "--wet" => cfg.wet_surface = true,
                "--scenarios-dir" => {
                    i += 1;
                    cfg.scenarios_dir = args
                        .get(i)
                        .ok_or("--scenarios-dir requires a directory path")?
                        .clone();
                }
                "--report-prefix" => {
                    i += 1;
                    cfg.report_prefix = args
                        .get(i)
                        .ok_or("--report-prefix requires a string")?
                        .clone();
                }
                other => {
                    return Err(format!("Unknown argument '{}'. Run with --help.", other));
                }
            }
            i += 1;
        }
        Ok(Some(cfg))
    }
}
