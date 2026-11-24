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
use std::sync::{Arc, RwLock};
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

#[derive(Clone)]
struct CachedAsset {
    data: Arc<Vec<u8>>,
    mime: &'static str,
    is_gz: bool,
    cache_control: &'static str,
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
        let cache: Arc<RwLock<HashMap<String, CachedAsset>>> = Arc::new(RwLock::new(HashMap::new()));

        thread::spawn(move || {
            let server = match Server::http(format!("127.0.0.1:{}", port)) {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("[AssetServer] Failed to bind HTTP server on port {}: {}", port, e);
                    return;
                }
            };
            println!("[AssetServer] High-Performance In-Memory Asset Server running at http://127.0.0.1:{}/", port);

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

                let cached = {
                    let cache_read = cache.read().unwrap();
                    cache_read.get(clean_path).cloned()
                };

                let asset = match cached {
                    Some(a) => a,
                    None => {
                        if file_path.exists() && file_path.is_file() {
                            match fs::read(&file_path) {
                                Ok(bytes) => {
                                    let is_gz = clean_path.ends_with(".gz");
                                    let cache_control = if clean_path.ends_with(".html") {
                                        "no-cache"
                                    } else {
                                        "public, max-age=31536000, immutable"
                                    };
                                    let new_asset = CachedAsset {
                                        data: Arc::new(bytes),
                                        mime,
                                        is_gz,
                                        cache_control,
                                    };
                                    let mut cache_write = cache.write().unwrap();
                                    cache_write.insert(clean_path.to_string(), new_asset.clone());
                                    new_asset
                                }
                                Err(_) => {
                                    let _ = request.respond(Response::from_string("File read error").with_status_code(500));
                                    continue;
                                }
                            }
                        } else {
                            let _ = request.respond(Response::from_string("404 Not Found").with_status_code(404));
                            continue;
                        }
                    }
                } else {
                    let _ = request.respond(Response::from_string("404 Not Found").with_status_code(404));
                }
            }
        })
    }
}
