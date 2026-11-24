// bridge/src/config.rs
// ============================================================================
// Configuration settings for Rust Native Desktop Host & Telemetry Bridge
// Supports defaults, environment variables, and CLI argument overrides
// ============================================================================

use std::env;

#[derive(Debug, Clone)]
pub struct BridgeConfig {
    pub http_port: u16,
    pub ws_port: u16,
    pub qnx_host: String,
    pub qnx_port: u16,
    pub server_only: bool,
    pub is_benchmark: bool,
}

impl Default for BridgeConfig {
    fn default() -> Self {
        Self {
            http_port: 8088,
            ws_port: 8089,
            qnx_host: "192.168.122.62".to_string(),
            qnx_port: 9090,
            server_only: false,
            is_benchmark: false,
        }
    }
}

impl BridgeConfig {
    pub fn from_env_and_args() -> Self {
        let mut config = Self::default();

        // 1. Environment variable overrides
        if let Ok(host) = env::var("QNX_HOST") {
            if !host.trim().is_empty() {
                config.qnx_host = host.trim().to_string();
            }
        }
        if let Ok(port_str) = env::var("QNX_PORT") {
            if let Ok(port) = port_str.trim().parse::<u16>() {
                config.qnx_port = port;
            }
        }
        if let Ok(port_str) = env::var("HTTP_PORT") {
            if let Ok(port) = port_str.trim().parse::<u16>() {
                config.http_port = port;
            }
        }
        if let Ok(port_str) = env::var("WS_PORT") {
            if let Ok(port) = port_str.trim().parse::<u16>() {
                config.ws_port = port;
            }
        }

        // 2. Command-line argument overrides
        let args: Vec<String> = env::args().collect();
        let mut i = 1;
        while i < args.len() {
            match args[i].as_str() {
                "--server-only" => {
                    config.server_only = true;
                }
                "--benchmark" => {
                    config.is_benchmark = true;
                }
                "--qnx-host" if i + 1 < args.len() => {
                    i += 1;
                    config.qnx_host = args[i].clone();
                }
                "--qnx-port" if i + 1 < args.len() => {
                    i += 1;
                    if let Ok(p) = args[i].parse::<u16>() {
                        config.qnx_port = p;
                    }
                }
                "--http-port" if i + 1 < args.len() => {
                    i += 1;
                    if let Ok(p) = args[i].parse::<u16>() {
                        config.http_port = p;
                    }
                }
                "--ws-port" if i + 1 < args.len() => {
                    i += 1;
                    if let Ok(p) = args[i].parse::<u16>() {
                        config.ws_port = p;
                    }
                }
                _ => {}
            }
            i += 1;
        }

        config
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bridge_config_default() {
        let cfg = BridgeConfig::default();
        assert_eq!(cfg.http_port, 8088);
        assert_eq!(cfg.ws_port, 8089);
        assert_eq!(cfg.qnx_port, 9090);
        assert_eq!(cfg.qnx_host, "192.168.122.62");
        assert_eq!(cfg.server_only, false);
        assert_eq!(cfg.is_benchmark, false);
    }
}
