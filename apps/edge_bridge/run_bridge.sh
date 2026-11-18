#!/bin/bash
# Script chạy Hardware Bridge trên Jetson Nano
TARGET_IP=${1:-"192.168.1.8"}

echo "============================================================"
echo "  🤖 KHỞI ĐỘNG JETBOT HARDWARE BRIDGE (CẢM BIẾN & ĐỘNG CƠ)"
echo "  Kết nối tới Laptop: ws://$TARGET_IP:8000/ws/robot"
echo "============================================================"

# Tắt tiến trình cũ để tránh xung đột port và nghẽn CPU
pkill -9 -f run.py 2>/dev/null
pkill -9 -f jetbot_bridge.py 2>/dev/null
sleep 1

export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra-egl:/usr/lib/aarch64-linux-gnu/tegra:$LD_LIBRARY_PATH
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
exec python3 "$DIR/jetbot_bridge.py" "$TARGET_IP"
