// bridge/src/ws_bridge.rs
// ============================================================================
// High-Speed WebSocket <-> QNX RTOS TCP Bridge
// Bridges 60 Hz JSON telemetry frames from Three.js frontend to QNX L3 Kernel
// ============================================================================

use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};
use tungstenite::accept;
use tungstenite::Message;

pub struct QnxWsBridge {
    ws_port: u16,
    qnx_host: String,
    qnx_port: u16,
}

impl QnxWsBridge {
    pub fn new(ws_port: u16, qnx_host: String, qnx_port: u16) -> Self {
        Self {
            ws_port,
            qnx_host,
            qnx_port,
        }
    }

    pub fn start(&self, running: Arc<AtomicBool>) -> thread::JoinHandle<()> {
        let ws_port = self.ws_port;
        let qnx_host = self.qnx_host.clone();
        let qnx_port = self.qnx_port;

        thread::spawn(move || {
            let listener = match TcpListener::bind(format!("127.0.0.1:{}", ws_port)) {
                Ok(l) => l,
                Err(e) => {
                    eprintln!("[QnxWsBridge] Failed to bind WebSocket bridge on port {}: {}", ws_port, e);
                    return;
                }
            };
            let _ = listener.set_nonblocking(true);
            println!("[QnxWsBridge] WebSocket Telemetry Bridge listening on ws://127.0.0.1:{}/", ws_port);

            while running.load(Ordering::Relaxed) {
                match listener.accept() {
                    Ok((stream, _)) => {
                        let running_clone = running.clone();
                        let host_clone = qnx_host.clone();
                        thread::spawn(move || {
                            Self::handle_client(stream, &host_clone, qnx_port, running_clone);
                        });
                    }
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        thread::sleep(Duration::from_millis(20));
                    }
                    Err(e) => {
                        eprintln!("[QnxWsBridge] Accept error: {}", e);
                        break;
                    }
                }
            }
        })
    }

    fn handle_client(stream: TcpStream, qnx_host: &str, qnx_port: u16, running: Arc<AtomicBool>) {
        let _ = stream.set_nodelay(true);
        let mut ws = match accept(stream) {
            Ok(w) => w,
            Err(e) => {
                eprintln!("[QnxWsBridge] WebSocket handshake failed: {}", e);
                return;
            }
        };
        let _ = ws.get_ref().set_nodelay(true);
        println!("[QnxWsBridge] 3D Simulation Client Connected to QNX RTOS Bridge!");

        // Connect to QNX Neutrino RTOS target on VM
        let qnx_stream = match TcpStream::connect(format!("{}:{}", qnx_host, qnx_port)) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("[QnxWsBridge] ERROR: Could not connect to QNX RTOS VM at {}:{}: {}", qnx_host, qnx_port, e);
                let err_json = format!(
                    r#"{{"throttle":0.0,"brake":0.0,"steer_cmd":0.0,"left_turn_signal":false,"right_turn_signal":false,"hazard_lights":false,"horn_active":false,"brake_lights":false,"left_mirror_blind_spot":false,"right_mirror_blind_spot":false,"drive_mode":0,"lead_distance":999.0,"lead_rel_speed":0.0,"time_to_collision":999.0,"status_message":"QNX Target Unreachable ({}:{})","bridge_rtt_us":999999}}"#,
                    qnx_host, qnx_port
                );
                let _ = ws.send(Message::Text(err_json));
                return;
            }
        };
        let _ = qnx_stream.set_nodelay(true);
        let _ = qnx_stream.set_read_timeout(Some(Duration::from_millis(200)));

        let mut qnx_writer = qnx_stream.try_clone().expect("Failed to clone QNX socket");
        let mut qnx_reader = BufReader::new(qnx_stream);

        // Pre-allocate zero-GC reuse buffers for 60 Hz relay
        let mut qnx_send_buf: Vec<u8> = Vec::with_capacity(4096);
        let mut qnx_response = String::with_capacity(4096);
        let mut ws_payload = String::with_capacity(4096);

        // Rolling RTT latency tracking (microseconds)
        let mut rtt_samples: Vec<u64> = Vec::with_capacity(1000);
        let mut frame_count = 0u64;
        let mut last_log = Instant::now();

        while running.load(Ordering::Relaxed) {
            // Read telemetry packet from 3D Frontend
            let msg = match ws.read() {
                Ok(m) => m,
                Err(tungstenite::Error::ConnectionClosed) => {
                    println!("[QnxWsBridge] 3D Simulation Client Disconnected.");
                    break;
                }
                Err(_) => break,
            };

            if let Message::Text(telemetry_json) = msg {
                let start = Instant::now();

                // Forward telemetry line to QNX RTOS using pre-allocated buffer
                qnx_send_buf.clear();
                qnx_send_buf.extend_from_slice(telemetry_json.trim().as_bytes());
                qnx_send_buf.push(b'\n');

                if let Err(e) = qnx_writer.write_all(&qnx_send_buf) {
                    eprintln!("[QnxWsBridge] Failed to send to QNX: {}", e);
                    break;
                }
                let _ = qnx_writer.flush();

                // Receive real-time actuator command from QNX RTOS
                qnx_response.clear();
                match qnx_reader.read_line(&mut qnx_response) {
                    Ok(0) => {
                        eprintln!("[QnxWsBridge] QNX Server closed connection.");
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
                        eprintln!("[QnxWsBridge] Timeout or error reading from QNX: {}", e);
                        break;
                    }
                }
            }
        }
    }
}
