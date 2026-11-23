// bridge/src/window_launcher.rs
// ============================================================================
// Native Desktop Window Launcher and Path Discovery
// Launches Chrome/Chromium in standalone app window mode with WebGPU enabled
// ============================================================================

use std::path::{Path, PathBuf};
use std::process::{Child, Command};

pub struct DesktopWindowLauncher;

impl DesktopWindowLauncher {
    pub fn find_slop_dir() -> PathBuf {
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

    pub fn launch(url: &str) -> Option<Child> {
        println!("[DesktopWindowLauncher] Launching Standalone Desktop Simulation Window...");

        let chrome_paths = [
            "/usr/bin/google-chrome",
            "/usr/bin/google-chrome-stable",
            "/usr/bin/chromium",
            "/usr/bin/chromium-browser",
        ];

        for path in &chrome_paths {
            if Path::new(path).exists() {
                println!("[DesktopWindowLauncher] Using Native App Shell: {}", path);
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
                    Err(e) => eprintln!("[DesktopWindowLauncher] Failed to spawn {}: {}", path, e),
                }
            }
        }

        eprintln!("[DesktopWindowLauncher] Chrome/Chromium not found. Opening with xdg-open...");
        let _ = Command::new("xdg-open").arg(url).spawn();
        None
    }
}
