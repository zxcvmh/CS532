#!/bin/bash
# Script dừng an toàn toàn bộ hệ thống Autonomous JetBot
echo "Đang dừng toàn bộ tiến trình JetBot & ngắt động cơ..."
pkill -9 -f run.py 2>/dev/null
pkill -9 -f jetbot_bridge 2>/dev/null
sleep 1
echo "✅ Đã dừng an toàn tất cả động cơ và dịch vụ Web."
