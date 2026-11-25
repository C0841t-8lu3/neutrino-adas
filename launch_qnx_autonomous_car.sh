#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# QNX 8.0 NEUTRINO RTOS <-> NATIVE 60 FPS BEVY 3D ADAS SIMULATOR LAUNCHER
# Map   : --map mega10km (default ~10+ km procedural) | --map waymo (ScenarioNet HD)
# View  : --view 3d (default Chase-Cam) | --view 2d (Top-Down FSD), press [V] to toggle live
# Optimized for Intel Core i3 + Intel HD Integrated Graphics
# ==============================================================================

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
QNX_VM_IP="${QNX_VM_IP:-192.168.122.62}"
QNX_PORT="${QNX_PORT:-9090}"
QEMU_DIR="/home/aauel/qnxprojects/targets/qemu-qnx800-x86_64-1"
QNX_BINARY="$PROJECT_DIR/nto/x86_64/o/helloworld"
RUST_BIN="$PROJECT_DIR/target/release/qnx_rust_sim"

# Fast-path help printout without touching QNX VM or opening Bevy window
for arg in "$@"; do
    if [[ "$arg" == "-h" || "$arg" == "--help" ]]; then
        if [ -x "$RUST_BIN" ]; then
            exec "$RUST_BIN" --help
        fi
        break
    fi
done

echo -e "\033[1;36m==================================================================\033[0m"
echo -e "\033[1;36m  QNX 8.0 NEUTRINO RTOS <-> NATIVE BEVY 3D ADAS SIMULATOR         \033[0m"
echo -e "\033[1;36m  Real MetaDrive glTF Assets + Waymo ScenarioNet + ~10km MegaMap  \033[0m"
echo -e "\033[1;36m==================================================================\033[0m"

is_qnx_port_open() {
    timeout 1 bash -c "</dev/tcp/$QNX_VM_IP/$QNX_PORT" 2>/dev/null
}

# ------------------------------------------------------------------------------
# 1. Verify / Boot QNX 8.0 VM and Start ADAS Safety Kernel Daemon on Port 9090
# ------------------------------------------------------------------------------
SSH_OPTS="-o BatchMode=yes -o LogLevel=ERROR -o ConnectTimeout=2"

echo -e "\n\033[1;33m[1/3] Checking QNX 8.0 RTOS Target ($QNX_VM_IP:$QNX_PORT)...\033[0m"
if ! is_qnx_port_open; then
    if ! ssh $SSH_OPTS qnxuser@"$QNX_VM_IP" "uname -s" >/dev/null 2>&1; then
        if [ -d "$QEMU_DIR" ] && [ -f "$QEMU_DIR/output/ifs.bin" ]; then
            echo -e "\033[1;33m[INFO] QNX VM is offline. Booting QEMU virtual machine in background...\033[0m"
            rm -f "$QEMU_DIR/output/qemu.pid"
            (cd "$QEMU_DIR" && nohup qemu-system-x86_64 -smp 2 --enable-kvm --cpu host -m 1G \
                -drive file=output/disk-qemu.vmdk,if=ide,id=drv0 \
                -netdev bridge,br=virbr0,id=net0 \
                -device virtio-net-pci,netdev=net0,mac=52:54:00:89:c0:37 \
                -pidfile output/qemu.pid -nographic -kernel output/ifs.bin \
                -object rng-random,filename=/dev/urandom,id=rng0 \
                -device virtio-rng-pci,rng=rng0 > /tmp/qemu_qnx.log 2>&1 &)

            echo -n "Waiting for QNX Neutrino RTOS to boot"
            for _ in {1..20}; do
                if ssh $SSH_OPTS qnxuser@"$QNX_VM_IP" "uname -s" >/dev/null 2>&1; then
                    echo -e " \033[1;32m[READY]\033[0m"
                    break
                fi
                echo -n "."
                sleep 1
            done
            echo ""
        fi
    fi

    if ! ssh $SSH_OPTS qnxuser@"$QNX_VM_IP" "uname -s" >/dev/null 2>&1; then
        echo -e "\033[1;31m[ERROR] Cannot reach QNX VM at $QNX_VM_IP via SSH or TCP:$QNX_PORT.\033[0m"
        exit 1
    fi

    if [ ! -f "$QNX_BINARY" ]; then
        echo -e "\033[1;31m[ERROR] QNX binary not found at $QNX_BINARY. Run 'make' first.\033[0m"
        exit 1
    fi
    echo "Deploying and starting QNX ADAS Safety Kernel daemon on $QNX_VM_IP:$QNX_PORT..."
    ssh $SSH_OPTS qnxuser@"$QNX_VM_IP" 'PIDS=$(pidin -f aN | awk "/qnx_adas_server/{print \$1}" | uniq); for p in $PIDS; do kill -9 "$p" 2>/dev/null || true; done; rm -f /tmp/qnx_adas_server' >/dev/null 2>&1 || true
    scp $SSH_OPTS -q "$QNX_BINARY" qnxuser@"$QNX_VM_IP":/tmp/qnx_adas_server
    ssh $SSH_OPTS -f -n qnxuser@"$QNX_VM_IP" \
        "chmod +x /tmp/qnx_adas_server && nohup /tmp/qnx_adas_server --server --port $QNX_PORT </dev/null >/tmp/qnx_server.log 2>&1 &"

    for _ in {1..15}; do
        if is_qnx_port_open; then
            break
        fi
        sleep 0.4
    done
fi

if ! is_qnx_port_open; then
    echo -e "\033[1;31m[ERROR] QNX ADAS Safety Kernel is not listening on $QNX_VM_IP:$QNX_PORT.\033[0m"
    exit 1
fi
echo -e "\033[1;32m[OK]\033[0m QNX 8.0 ADAS Safety Kernel listening on $QNX_VM_IP:$QNX_PORT."

# ------------------------------------------------------------------------------
# 2. Ensure Native Bevy Release Simulator Binary Is Built
# ------------------------------------------------------------------------------
echo -e "\n\033[1;33m[2/3] Checking Native Bevy 3D Release Binary ($RUST_BIN)...\033[0m"
cd "$PROJECT_DIR"
if [ ! -x "$RUST_BIN" ] || [ -n "$(find "$PROJECT_DIR/rust_sim/src" "$PROJECT_DIR/Cargo.toml" -newer "$RUST_BIN" 2>/dev/null)" ]; then
    echo "Building release binary via cargo build --release..."
    cargo build --release
fi
echo -e "\033[1;32m[OK]\033[0m Native Bevy binary ready."

# ------------------------------------------------------------------------------
# 3. Launch Native Bevy 3D Simulator <-> QNX 8.0 Safety Kernel
# ------------------------------------------------------------------------------
echo -e "\n\033[1;33m[3/3] Launching Native Bevy 3D QNX ADAS Simulator...\033[0m"
exec "$RUST_BIN" "$@"
