use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::path::{Path, PathBuf};
use std::process::{Child, Command};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};
use tiny_http::{Header, Response, Server};
use tungstenite::accept;
use tungstenite::Message;

const HTTP_PORT: u16 = 8088;
const WS_PORT: u16 = 8089;
const QNX_HOST: &str = "192.168.122.62";
const QNX_PORT: u16 = 9090;

fn get_mime_type(path: &Path) -> &'static str {
    match path.extension().and_then(|s| s.to_str()).unwrap_or("") {
        "html" => "text/html; charset=utf-8",
        "js" | "mjs" => "application/javascript; charset=utf-8",
        "css" => "text/css; charset=utf-8",
        "json" => "application/json",
        "png" => "image/png",
        "jpg" | "jpeg" => "image/jpeg",
        "svg" => "image/svg+xml",
        "wasm" => "application/wasm",
        "woff2" => "font/woff2",
        "woff" => "font/woff",
        "gz" => "application/gzip",
        "bin" => "application/octet-stream",
        _ => "application/octet-stream",
    }
}

// 1. Static Asset HTTP Server for Slop New York 3D Simulator
fn start_asset_server(root_dir: PathBuf, running: Arc<AtomicBool>) {
    let server = match Server::http(format!("127.0.0.1:{}", HTTP_PORT)) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("[Rust Host] Failed to bind HTTP server on port {}: {}", HTTP_PORT, e);
            return;
        }
    };
    println!("[Rust Host] Simulation Asset Server running at http://127.0.0.1:{}/", HTTP_PORT);

    while running.load(Ordering::Relaxed) {
        let request = match server.recv_timeout(Duration::from_millis(200)) {
            Ok(Some(req)) => req,
            Ok(None) => continue,
            Err(_) => continue,
        };

        let raw_url = request.url().trim_start_matches('/');
        let clean_path = if raw_url.is_empty() || raw_url.starts_with('?') {
            "index.html"
        } else {
            raw_url.split('?').next().unwrap_or("index.html")
        };

        let file_path = root_dir.join(clean_path);
        let mime = get_mime_type(&file_path);

        if file_path.exists() && file_path.is_file() {
            if let Ok(file) = File::open(&file_path) {
                let mut response = Response::from_file(file);
                if let Ok(header) = Header::from_bytes(&b"Content-Type"[..], mime.as_bytes()) {
                    response.add_header(header);
                }
                // Allow cross-origin module loading and caching
                if let Ok(cors) = Header::from_bytes(&b"Access-Control-Allow-Origin"[..], &b"*"[..]) {
                    response.add_header(cors);
                }
                let _ = request.respond(response);
            } else {
                let _ = request.respond(Response::from_string("File open error").with_status_code(500));
            }
        } else {
            let _ = request.respond(Response::from_string("404 Not Found").with_status_code(404));
        }
    }
}

// 2. High-Speed WebSocket <-> QNX RTOS TCP Bridge
fn handle_ws_client(stream: TcpStream, running: Arc<AtomicBool>) {
    let _ = stream.set_nodelay(true);
    let mut ws = match accept(stream) {
        Ok(w) => w,
        Err(e) => {
            eprintln!("[Rust Bridge] WebSocket handshake failed: {}", e);
            return;
        }
    };
    println!("[Rust Bridge] 3D Simulation Client Connected to QNX RTOS Bridge!");

    // Connect to QNX Neutrino RTOS target on VM
    let qnx_stream = match TcpStream::connect(format!("{}:{}", QNX_HOST, QNX_PORT)) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("[Rust Bridge] ERROR: Could not connect to QNX RTOS VM at {}:{}: {}", QNX_HOST, QNX_PORT, e);
            let err_json = format!(
                r#"{{"throttle":0.0,"brake":0.0,"steer_cmd":0.0,"left_turn_signal":false,"right_turn_signal":false,"hazard_lights":false,"horn_active":false,"brake_lights":false,"left_mirror_blind_spot":false,"right_mirror_blind_spot":false,"drive_mode":0,"lead_distance":999.0,"lead_rel_speed":0.0,"time_to_collision":999.0,"status_message":"QNX Target Unreachable ({}:{})"}}"#,
                QNX_HOST, QNX_PORT
            );
            let _ = ws.send(Message::Text(err_json));
            return;
        }
    };
    let _ = qnx_stream.set_nodelay(true);
    let _ = qnx_stream.set_read_timeout(Some(Duration::from_millis(200)));

    let mut qnx_writer = qnx_stream.try_clone().expect("Failed to clone QNX socket");
    let mut qnx_reader = BufReader::new(qnx_stream);

    let mut frame_count = 0u64;
    let mut last_log = Instant::now();

    while running.load(Ordering::Relaxed) {
        // Read telemetry packet from 3D Frontend
        let msg = match ws.read() {
            Ok(m) => m,
            Err(tungstenite::Error::ConnectionClosed) => {
                println!("[Rust Bridge] 3D Simulation Client Disconnected.");
                break;
            }
            Err(_) => break,
        };

        if let Message::Text(telemetry_json) = msg {
            let start = Instant::now();

            // Forward telemetry line to QNX RTOS
            let mut line = telemetry_json.trim().to_string();
            line.push('\n');
            if let Err(e) = qnx_writer.write_all(line.as_bytes()) {
                eprintln!("[Rust Bridge] Failed to send to QNX: {}", e);
                break;
            }
            let _ = qnx_writer.flush();

            // Receive real-time actuator command from QNX RTOS
            let mut qnx_response = String::new();
            match qnx_reader.read_line(&mut qnx_response) {
                Ok(0) => {
                    eprintln!("[Rust Bridge] QNX Server closed connection.");
                    break;
                }
                Ok(_) => {
                    let rtt_us = start.elapsed().as_micros();
                    // Send back to frontend WebSocket
                    let _ = ws.send(Message::Text(qnx_response.trim().to_string()));

                    frame_count += 1;
                    if last_log.elapsed() >= Duration::from_secs(2) {
                        println!(
                            "[QNX RTOS ACTIVE] 60Hz Telemetry Link Healthy | RTT: {} us | Packets: {}",
                            rtt_us, frame_count
                        );
                        last_log = Instant::now();
                    }
                }
                Err(e) => {
                    eprintln!("[Rust Bridge] Timeout or error reading from QNX: {}", e);
                    break;
                }
            }
        }
    }
}

fn start_ws_bridge(running: Arc<AtomicBool>) {
    let listener = match TcpListener::bind(format!("127.0.0.1:{}", WS_PORT)) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("[Rust Bridge] Failed to bind WebSocket bridge on port {}: {}", WS_PORT, e);
            return;
        }
    };
    let _ = listener.set_nonblocking(true);
    println!("[Rust Bridge] WebSocket Telemetry Bridge listening on ws://127.0.0.1:{}/", WS_PORT);

    while running.load(Ordering::Relaxed) {
        match listener.accept() {
            Ok((stream, _)) => {
                let running_clone = running.clone();
                thread::spawn(move || {
                    handle_ws_client(stream, running_clone);
                });
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(20));
            }
            Err(e) => {
                eprintln!("[Rust Bridge] Accept error: {}", e);
                break;
            }
        }
    }
}

// 3. Launch Standalone Native Desktop Window
fn launch_desktop_window(url: &str) -> Option<Child> {
    println!("[Rust Host] Launching Standalone Desktop Simulation Window...");

    let chrome_paths = [
        "/usr/bin/google-chrome",
        "/usr/bin/google-chrome-stable",
        "/usr/bin/chromium",
        "/usr/bin/chromium-browser",
    ];

    for path in &chrome_paths {
        if Path::new(path).exists() {
            println!("[Rust Host] Using Native App Shell: {}", path);
            match Command::new(path)
                .args([
                    &format!("--app={}", url),
                    "--user-data-dir=/tmp/qnx_chrome_profile",
                    "--window-size=1600,950",
                    "--window-position=50,50",
                    "--enable-features=Vulkan,WebGPU,DefaultANGLEVulkan",
                    "--enable-unsafe-webgpu",
                    "--disable-infobars",
                    "--disable-session-crashed-bubble",
                    "--no-first-run",
                    "--no-default-browser-check",
                    "--remote-debugging-port=9222",
                    "--autoplay-policy=no-user-gesture-required",
                ])
                .spawn()
            {
                Ok(child) => return Some(child),
                Err(e) => eprintln!("[Rust Host] Failed to spawn {}: {}", path, e),
            }
        }
    }

    eprintln!("[Rust Host] Chrome/Chromium not found. Opening with xdg-open...");
    let _ = Command::new("xdg-open").arg(url).spawn();
    None
}

fn find_slop_dir() -> PathBuf {
    // 1. Try relative to current working directory
    let local = PathBuf::from("slop-newyork");
    if local.exists() && local.is_dir() {
        return local;
    }

    // 2. Try relative to executable directory
    if let Ok(mut exe_dir) = std::env::current_exe() {
        exe_dir.pop();
        let cand1 = exe_dir.join("slop-newyork");
        if cand1.exists() {
            return cand1;
        }
        // If in target/release or target/debug, traverse up
        if let Some(target_dir) = exe_dir.parent() {
            if let Some(proj_dir) = target_dir.parent() {
                let cand2 = proj_dir.join("slop-newyork");
                if cand2.exists() {
                    return cand2;
                }
            }
        }
    }

    // 3. Fallback absolute path
    PathBuf::from("/home/aauel/qnxprojects/helloworld/slop-newyork")
}

fn main() {
    println!("==================================================================");
    println!("  QNX 8.0 NEUTRINO RTOS - AUTONOMOUS VEHICLE 3D SIMULATOR APP     ");
    println!("  Core: QNX SDP 8.0 ADAS L3 Safety Kernel ({}:{})", QNX_HOST, QNX_PORT);
    println!("  Desktop Host: Rust Native Desktop Application                   ");
    println!("==================================================================");

    let running = Arc::new(AtomicBool::new(true));

    let root_dir = find_slop_dir();
    if !root_dir.exists() {
        eprintln!("[Rust Host] ERROR: slop-newyork directory not found at {:?}", root_dir);
        return;
    }
    println!("[Rust Host] Serving 3D City Assets from: {:?}", root_dir);

    // Start Asset Server
    let r1 = running.clone();
    let root_clone = root_dir.clone();
    let http_thread = thread::spawn(move || {
        start_asset_server(root_clone, r1);
    });

    // Start WebSocket Bridge to QNX
    let r2 = running.clone();
    let ws_thread = thread::spawn(move || {
        start_ws_bridge(r2);
    });

    // Brief pause to allow listeners to bind
    thread::sleep(Duration::from_millis(500));

    // Check CLI arguments
    let args: Vec<String> = std::env::args().collect();
    let server_only = args.iter().any(|a| a == "--server-only");
    let is_benchmark = args.iter().any(|a| a == "--benchmark");

    // Open Desktop Window in Standalone App Mode
    let app_url = if is_benchmark {
        format!("http://127.0.0.1:{}/?benchmark=1&duration=25", HTTP_PORT)
    } else {
        format!("http://127.0.0.1:{}/", HTTP_PORT)
    };

    let mut app_process = if !server_only {
        launch_desktop_window(&app_url)
    } else {
        println!("[Rust Host] Running in Server-Only Mode (HTTP {}, WS {}).", HTTP_PORT, WS_PORT);
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
