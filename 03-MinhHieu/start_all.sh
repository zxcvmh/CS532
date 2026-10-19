#!/bin/bash
# Script khởi động toàn bộ hệ thống Autonomous JetBot (Edge Standalone trên Jetson Nano)
pkill -9 -f run.py 2>/dev/null
pkill -9 -f jetbot_bridge 2>/dev/null
sleep 1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/tegra-egl:/usr/lib/aarch64-linux-gnu/tegra:$LD_LIBRARY_PATH"
export AUTO_START_BRIDGE="0"

echo "[1/2] Đang khởi động Web Server Backend & Bộ não Thuật toán (cổng 8000)..."
cd "$SCRIPT_DIR/backend"
nohup python3 run.py --host 0.0.0.0 --port 8000 > "$SCRIPT_DIR/backend/server.log" 2>&1 &
sleep 2

echo "[2/2] Đang kết nối Cảm biến Phần cứng (Camera CSI, LiDAR D500, TensorRT YOLO)..."
nohup python3 "$SCRIPT_DIR/jetbot_bridge.py" 127.0.0.1 > "$SCRIPT_DIR/bridge.log" 2>&1 &

echo "============================================================"
echo "  🎉 TOÀN BỘ HỆ THỐNG ĐÃ KHỞI ĐỘNG TRỰC TIẾP TRÊN JETBOT!"
echo "  - Web Dashboard:  http://192.168.1.11:8000"
echo "  - Log Web Server: $SCRIPT_DIR/backend/server.log"
echo "  - Log Cảm biến:   $SCRIPT_DIR/bridge.log"
echo "  - Dừng hệ thống:  bash $SCRIPT_DIR/stop_all.sh"
echo "============================================================"
