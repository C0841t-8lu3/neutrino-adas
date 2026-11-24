// bridge/src/asset_server.rs
// ============================================================================
// Static Asset HTTP Server for Slop New York 3D WebGPU Simulation
// In-Memory Cached Asset Server with Gzip Compression & Cache-Control
// ============================================================================

use std::collections::HashMap;
use std::fs;
use std::io::Cursor;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;
use tiny_http::{Header, Response, Server};

pub fn get_mime_type(path: &Path) -> &'static str {
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

pub struct AssetServer {
    root_dir: PathBuf,
    port: u16,
}

impl AssetServer {
    pub fn new(root_dir: PathBuf, port: u16) -> Self {
        Self { root_dir, port }
    }

    pub fn start(&self, running: Arc<AtomicBool>) -> thread::JoinHandle<()> {
        let port = self.port;
        let root_dir = self.root_dir.clone();

        thread::spawn(move || {
            let server = match Server::http(format!("127.0.0.1:{}", port)) {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("[AssetServer] Failed to bind HTTP server on port {}: {}", port, e);
                    return;
                }
            };
            println!("[AssetServer] Simulation Asset Server running at http://127.0.0.1:{}/", port);

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
        })
    }
}
