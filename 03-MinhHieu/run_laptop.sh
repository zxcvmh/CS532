#!/bin/bash
# Script chạy Dashboard Backend trên Laptop (CS532)
cd "$(dirname "$0")/backend" || exit 1
export AUTO_START_BRIDGE=0

MY_IP=$(hostname -I | awk '{print $1}')

echo "============================================================"
echo "  🚀 KHỞI ĐỘNG JETBOT DASHBOARD SERVER TRÊN LAPTOP"
echo "  IP Laptop của bạn: $MY_IP"
echo "  Dashboard URL:     http://localhost:8000"
echo "============================================================"
echo "👉 Trên JetBot hãy chạy: ./run_bridge.sh $MY_IP"
echo "============================================================"

exec .venv/bin/python3 run.py --host 0.0.0.0 --port 8000
