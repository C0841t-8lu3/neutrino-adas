// bridge/src/main.rs
// ============================================================================
// QNX 8.0 NEUTRINO RTOS - AUTONOMOUS VEHICLE 3D SIMULATION HOST
// High-performance asynchronous bridge connecting Three.js WebGPU Frontend
// to QNX SDP 8.0 ADAS L3 Safety Kernel daemon running on QNX Neutrino RTOS
// ============================================================================

mod config;
mod asset_server;
mod ws_bridge;
mod window_launcher;

use config::BridgeConfig;
use asset_server::AssetServer;
use ws_bridge::QnxWsBridge;
use window_launcher::DesktopWindowLauncher;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

fn main() {
    let config = BridgeConfig::from_env_and_args();

    println!("==================================================================");
    println!("  QNX 8.0 NEUTRINO RTOS - AUTONOMOUS VEHICLE 3D SIMULATOR APP     ");
    println!("  Core: QNX SDP 8.0 ADAS L3 Safety Kernel ({}:{})", config.qnx_host, config.qnx_port);
    println!("  Desktop Host: Rust Native Desktop Application                   ");
    println!("==================================================================");

    let running = Arc::new(AtomicBool::new(true));

    let root_dir = DesktopWindowLauncher::find_slop_dir();
    if !root_dir.exists() {
        eprintln!("[Rust Host] ERROR: slop-newyork directory not found at {:?}", root_dir);
        return;
    }
    println!("[Rust Host] Serving 3D City Assets from: {:?}", root_dir);

    // Start Asset Server
    let asset_server = AssetServer::new(root_dir, config.http_port);
    let http_thread = asset_server.start(running.clone());

    // Start WebSocket Bridge to QNX
    let ws_bridge = QnxWsBridge::new(config.ws_port, config.qnx_host.clone(), config.qnx_port);
    let ws_thread = ws_bridge.start(running.clone());

    // Brief pause to allow listeners to bind
    thread::sleep(Duration::from_millis(500));

    // Open Desktop Window in Standalone App Mode
    let app_url = if config.is_benchmark {
        format!("http://127.0.0.1:{}/?benchmark=1&duration=25", config.http_port)
    } else {
        format!("http://127.0.0.1:{}/", config.http_port)
    };

    let mut app_process = if !config.server_only {
        DesktopWindowLauncher::launch(&app_url)
    } else {
        println!("[Rust Host] Running in Server-Only Mode (HTTP {}, WS {}).", config.http_port, config.ws_port);
        None
    };

    println!("[Rust Host] Autonomous Car Simulator Window Active. Close window to exit.");

    // Monitor Desktop App Process
    if let Some(ref mut child) = app_process {
        let _ = child.wait();
        println!("[Rust Host] Desktop Window closed by user. Shutting down QNX link...");
    } else {
        // Fallback wait loop
        while running.load(Ordering::Relaxed) {
            thread::sleep(Duration::from_millis(500));
        }
    }

    running.store(false, Ordering::Relaxed);
    let _ = http_thread.join();
    let _ = ws_thread.join();
    println!("[Rust Host] QNX Autonomous Vehicle Host terminated cleanly.");
}
