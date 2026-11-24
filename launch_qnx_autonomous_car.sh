#!/usr/bin/env bash
set -e

# ==============================================================================
# QNX 8.0 NEUTRINO RTOS - AUTONOMOUS VEHICLE 3D SIMULATION LAUNCHER
# Integrates:
# - QNX SDP 8.0 ADAS L3 Safety Kernel Daemon on QEMU Target (192.168.122.62:9090)
# - Rust Native Desktop Application Host & Telemetry Bridge
# - Slop New York Photorealistic 3D Simulation with Three.js TSL Shaders
# - Real-Time Side-View Mirrors with BSM LEDs & Accident Prevention Supervisor
# ==============================================================================

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUST_APP_DIR="$PROJECT_DIR"
QNX_VM_IP="192.168.122.62"
QNX_PORT="9090"
QEMU_DIR="/home/aauel/qnxprojects/targets/qemu-qnx800-x86_64-1"

echo -e "\033[1;36m==================================================================\033[0m"
echo -e "\033[1;36m  QNX 8.0 NEUTRINO RTOS - AUTONOMOUS CAR 3D SIMULATION SYSTEM     \033[0m"
echo -e "\033[1;36m==================================================================\033[0m"

# 1. Check QNX VM Connectivity (Auto-boot if offline)
SSH_OPTS="-o BatchMode=yes -o LogLevel=ERROR -o ConnectTimeout=2"

echo -e "\n\033[1;33m[1/3] Checking QNX 8.0 RTOS Target ($QNX_VM_IP)...\033[0m"
if ! ssh $SSH_OPTS qnxuser@"$QNX_VM_IP" "uname -s" >/dev/null 2>&1; then
    if [ -d "$QEMU_DIR" ] && [ -f "$QEMU_DIR/output/ifs.bin" ]; then
        echo -e "\033[1;33m[INFO] QNX VM is offline. Automatically booting QEMU virtual machine in background...\033[0m"
        rm -f "$QEMU_DIR/output/qemu.pid"
        (cd "$QEMU_DIR" && nohup qemu-system-x86_64 -smp 2 --enable-kvm --cpu host -m 1G \
            -drive file=output/disk-qemu.vmdk,if=ide,id=drv0 \
            -netdev bridge,br=virbr0,id=net0 \
            -device virtio-net-pci,netdev=net0,mac=52:54:00:89:c0:37 \
            -pidfile output/qemu.pid -nographic -kernel output/ifs.bin \
            -object rng-random,filename=/dev/urandom,id=rng0 \
            -device virtio-rng-pci,rng=rng0 > /tmp/qemu_qnx.log 2>&1 &)

        echo -n "Waiting for QNX Neutrino RTOS to boot"
        for i in {1..20}; do
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
    echo -e "\033[1;31m[ERROR] Cannot reach QNX VM at $QNX_VM_IP via SSH.\033[0m"
    echo "Please ensure the QNX 8.0 QEMU virtual machine is running."
    exit 1
fi
echo -e "\033[1;32m[OK]\033[0m QNX 8.0 Neutrino VM is online."

# 2. Deploy & Ensure QNX ADAS Server Daemon is Running on VM
echo -e "\n\033[1;33m[2/3] Verifying QNX ADAS L3 Kernel Daemon on VM...\033[0m"
if ! ssh $SSH_OPTS qnxuser@"$QNX_VM_IP" "pidin | grep qnx_adas" >/dev/null 2>&1; then
    echo "Starting QNX ADAS Server Daemon on port $QNX_PORT..."
    scp -o BatchMode=yes "$PROJECT_DIR/nto/x86_64/o/helloworld" qnxuser@"$QNX_VM_IP":/tmp/qnx_adas_server
    ssh -o BatchMode=yes qnxuser@"$QNX_VM_IP" "chmod +x /tmp/qnx_adas_server && nohup /tmp/qnx_adas_server --server --port $QNX_PORT > /tmp/qnx_server.log 2>&1 & sleep 1"
fi
echo -e "\033[1;32m[OK]\033[0m QNX ADAS L3 Server running on $QNX_VM_IP:$QNX_PORT."

# 3. Launch Rust Desktop App Host
echo -e "\n\033[1;33m[3/3] Launching Rust Native Desktop Host Application...\033[0m"
cd "$RUST_APP_DIR"
exec ./target/release/qnx_desktop_app
