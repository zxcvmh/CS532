#!/usr/bin/env python3
"""
Simple runner for JetBot Dashboard Backend.
Usage:
    python3 run.py [--host 0.0.0.0] [--port 8000]
"""

import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

def check_dependencies():
    missing = []
    for pkg in ["uvicorn", "fastapi", "websockets", "pydantic"]:
        try:
            __import__(pkg)
        except ImportError:
            missing.append(pkg)
    return missing

def main():
    missing = check_dependencies()
    if missing:
        print("=" * 60)
        print("[!] THIẾU THƯ VIỆN ĐỂ CHẠY BACKEND:")
        print(f"    Các gói còn thiếu: {', '.join(missing)}")
        print("\n👉 Hãy chạy lệnh sau trên terminal JetBot để cài đặt:")
        print(f"    pip3 install --user {' '.join(missing)}")
        print("\n(Nếu Jetson Nano dùng Python 3.6 và báo lỗi phiên bản, hãy chạy:)")
        print('    pip3 install --user "uvicorn<0.17" "fastapi<0.70" "pydantic<2.0" "websockets<11.0"')
        print("=" * 60)
        sys.exit(1)

    import uvicorn
    host = "0.0.0.0"
    port = 8000

    for i, arg in enumerate(sys.argv):
        if arg == "--host" and i + 1 < len(sys.argv):
            host = sys.argv[i + 1]
        elif arg == "--port" and i + 1 < len(sys.argv):
            port = int(sys.argv[i + 1])

    print("=" * 60)
    print(f"  JETBOT DASHBOARD SERVER")
    print(f"  URL: http://{host}:{port}")
    print("=" * 60)

    uvicorn.run("app.main:app", host=host, port=port, reload=False)

if __name__ == "__main__":
    main()
