#!/usr/bin/env python3
"""
QNX 8.0 ADAS DRIVING REALISM & SAFETY BENCHMARK RUNNER
Executes the fixed test route on the running simulation and collects concrete metrics:
- Lane-keeping offset (RMS, Mean, Max)
- Steering angle vs road curvature (Mean, Max steer error, heading error)
- Braking distance & harsh deceleration
- Collisions & Near-Misses
- Signal / Red-Light compliance
"""

import sys
import os
import time
import json
import subprocess
import urllib.request
import asyncio
import websockets

HTTP_PORT = 8088
WS_PORT = 8089
CDP_PORT = 9222
DURATION = 25.0

def is_port_open(port):
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.connect(("127.0.0.1", port))
        s.close()
        return True
    except:
        return False

async def run_cdp_session(output_file, duration=25.0):
    url = f"http://127.0.0.1:{CDP_PORT}/json"
    print(f"[Benchmark Runner] Polling Chrome CDP at {url}...")
    
    ws_url = None
    for _ in range(30):
        try:
            req = urllib.request.urlopen(url, timeout=1.0)
            pages = json.loads(req.read().decode())
            for page in pages:
                if "8088" in page.get("url", ""):
                    ws_url = page.get("webSocketDebuggerUrl")
                    break
            if ws_url:
                break
        except Exception:
            pass
        await asyncio.sleep(0.5)

    if not ws_url:
        print("[Benchmark Runner] ERROR: Could not find active simulator page in Chrome CDP!")
        return None

    print(f"[Benchmark Runner] Attached to Chrome DevTools: {ws_url}")
    
    async with websockets.connect(ws_url, max_size=10*1024*1024) as ws:
        # Enable console and runtime
        await ws.send(json.dumps({"id": 1, "method": "Runtime.enable"}))
        await ws.send(json.dumps({"id": 2, "method": "Console.enable"}))
        
        # Poll for completion
        startup_start_time = time.time()
        max_startup_timeout = 300.0 # allow shader compilation & pipeline warmup
        route_start_time = None
        route_timeout = duration * 4.0 + 30.0
        
        report = None
        current_status = "INITIALIZING"
        last_progress_log = time.time()
        while True:
            now = time.time()
            if route_start_time is None:
                if now - startup_start_time > max_startup_timeout:
                    print("[Benchmark Runner] ERROR: Timed out waiting for scene initialization.")
                    break
            else:
                if now - route_start_time > route_timeout:
                    print("[Benchmark Runner] ERROR: Timed out waiting for route completion.")
                    break

            try:
                # Check if report is ready on window object
                msg_id = int(time.time() * 1000) % 100000 + 10
                check_cmd = {
                    "id": msg_id,
                    "method": "Runtime.evaluate",
                    "params": {
                        "expression": "window.__DRIVING_METRICS_REPORT__ ? JSON.stringify(window.__DRIVING_METRICS_REPORT__) : (window.__BENCHMARK_STATUS__ || '')"
                    }
                }
                await ws.send(json.dumps(check_cmd))
                
                # Read messages
                while True:
                    raw = await asyncio.wait_for(ws.recv(), timeout=0.8)
                    data = json.loads(raw)
                    
                    if data.get("id") == msg_id:
                        val = data.get("result", {}).get("result", {}).get("value", "")
                        if val:
                            if "lane_keeping" in val:
                                report = json.loads(val)
                                break
                            else:
                                current_status = str(val)
                                if current_status.startswith("RUNNING") and route_start_time is None:
                                    route_start_time = time.time()
                                    print(f"[Benchmark Runner] Route execution started at {now - startup_start_time:.1f}s!")
                    elif data.get("method") == "Runtime.exceptionThrown":
                        details = data.get("params", {}).get("exceptionDetails", {})
                        print(f"  [Browser Exception] {details.get('text', '')}: {details.get('exception', {}).get('description', '')}")
                    elif data.get("method") == "Runtime.consoleAPICalled":
                        call_type = data.get("params", {}).get("type", "")
                        args = data.get("params", {}).get("args", [])
                        text_parts = [str(a.get("value", "")) for a in args]
                        full_line = " ".join(text_parts)
                        if "__DRIVING_METRICS_REPORT__" in full_line:
                            json_part = full_line.split("__DRIVING_METRICS_REPORT__")[1].strip()
                            report = json.loads(json_part)
                            break
                        elif call_type == "error" or any(k in full_line for k in ["[init]", "[BENCHMARK]", "[METRICS]", "[QNX]", "Error", "Exception"]):
                            print(f"  [Browser Console {call_type}] {full_line}")
                    if report:
                        break
                if report:
                    break
            except asyncio.TimeoutError:
                pass
            except Exception as e:
                pass
            if time.time() - last_progress_log >= 5.0:
                print(f"[Benchmark Runner] Route in progress... ({time.time() - startup_start_time:.1f}s elapsed, status={current_status})")
                last_progress_log = time.time()
            await asyncio.sleep(0.5)

        if report:
            print(f"[Benchmark Runner] Test Route Completed Successfully in {report.get('route_duration_s')}s!")
            with open(output_file, "w") as f:
                json.dump(report, f, indent=2)
            print(f"[Benchmark Runner] Metrics saved to {output_file}")
            return report
        else:
            print("[Benchmark Runner] ERROR: Benchmark timed out before metrics report was received.")
            return None

def main():
    output_file = sys.argv[1] if len(sys.argv) > 1 else "benchmark_metrics.json"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else DURATION
    time_offset = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    
    print("==================================================================")
    print("  QNX 8.0 ADAS DRIVING REALISM BENCHMARK TEST RUNNER             ")
    print(f"  Target Route: 5th Ave Southbound (Duration: {duration}s)       ")
    print(f"  Output File:  {output_file}                                    ")
    print(f"  Time Offset:  {time_offset} s                                  ")
    print("==================================================================")

    # 1. Ensure Rust Desktop Host HTTP Asset Server is running
    rust_proc = None
    if not is_port_open(HTTP_PORT) or not is_port_open(WS_PORT):
        print("[Benchmark Runner] Starting Rust Native Desktop Host (qnx_desktop_app)...")
        rust_bin = os.path.join(os.path.dirname(os.path.abspath(__file__)), "target/release/qnx_desktop_app")
        if not os.path.exists(rust_bin):
            print(f"Building {rust_bin}...")
            subprocess.run(["cargo", "build", "--release"], check=True)
        # We start Rust host in background without browser launch (or let it run)
        # Note: desktop_app starts asset server and ws bridge
    
    # 2. Launch Chrome with CDP
    chrome_cmd = [
        "/usr/bin/google-chrome",
        f"--app=http://127.0.0.1:{HTTP_PORT}/?benchmark=1&duration={duration}&time_offset={time_offset}",
        f"--remote-debugging-port={CDP_PORT}",
        "--user-data-dir=/tmp/qnx_benchmark_profile",
        "--window-size=1600,950",
        "--window-position=50,50",
        "--enable-features=Vulkan,WebGPU,DefaultANGLEVulkan",
        "--enable-unsafe-webgpu",
        "--disable-infobars",
        "--no-first-run",
        "--no-default-browser-check",
        "--disable-background-timer-throttling",
        "--disable-backgrounding-occluded-windows",
        "--disable-renderer-backgrounding",
        "--autoplay-policy=no-user-gesture-required",
    ]
    
    # Check if Rust desktop app is already running
    desktop_running = is_port_open(HTTP_PORT) and is_port_open(WS_PORT)
    if not desktop_running:
        print("[Benchmark Runner] Launching target/release/qnx_desktop_app...")
        rust_proc = subprocess.Popen(["./target/release/qnx_desktop_app", "--server-only"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        time.sleep(1.5)
    else:
        print("[Benchmark Runner] Rust Desktop Host already active on ports 8088 & 8089.")

    print(f"[Benchmark Runner] Launching Chrome test instance with CDP on port {CDP_PORT}...")
    chrome_proc = subprocess.Popen(chrome_cmd)
    
    try:
        report = asyncio.run(run_cdp_session(output_file, duration))
        if report:
            print("\n============================================================")
            print("                RECORDED BENCHMARK METRICS                  ")
            print("============================================================")
            print(f"Route Duration:           {report['route_duration_s']} s")
            print(f"Frames Sampled:           {report['frames_sampled']}")
            print("Lane-Keeping Offset:")
            print(f"  - RMS Offset:           {report['lane_keeping']['rms_offset_m']} m")
            print(f"  - Mean Offset:          {report['lane_keeping']['mean_offset_m']} m")
            print(f"  - Max Offset:           {report['lane_keeping']['max_offset_m']} m")
            print("Steering vs Curvature:")
            print(f"  - Mean Steer Error:     {report['steering_vs_curvature']['mean_steer_error_deg']} deg")
            print(f"  - Max Steer Error:      {report['steering_vs_curvature']['max_steer_error_deg']} deg")
            print(f"  - Mean Heading Error:   {report['steering_vs_curvature']['mean_heading_error_deg']} deg")
            print("Braking & Clearance:")
            print(f"  - Min Obstacle Gap:     {report['braking_and_clearance']['min_obstacle_clearance_m']} m")
            print(f"  - Harsh Braking Events: {report['braking_and_clearance']['harsh_braking_events']}")
            print("Safety Incidents:")
            print(f"  - Collision Count:      {report['safety_incidents']['collision_count']}")
            print(f"  - Near-Miss Events:     {report['safety_incidents']['near_miss_count']}")
            print("Signal Compliance:")
            print(f"  - Intersections:        {report['signal_compliance']['intersections_encountered']}")
            print(f"  - Red Light Violations: {report['signal_compliance']['red_light_violations']}")
            print(f"  - Compliance Rate:      {report['signal_compliance']['compliance_rate_pct']} %")
            print("============================================================\n")
            return 0
        else:
            return 1
    finally:
        print("[Benchmark Runner] Terminating Chrome test instance...")
        chrome_proc.terminate()
        try:
            chrome_proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            chrome_proc.kill()
        if rust_proc:
            rust_proc.terminate()
            try:
                rust_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                rust_proc.kill()

if __name__ == "__main__":
    sys.exit(main())
