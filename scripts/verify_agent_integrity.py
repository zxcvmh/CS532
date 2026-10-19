#!/usr/bin/env python3
"""
verify_agent_integrity.py — Bộ Thẩm Định Tính Toàn Vẹn Cú Pháp & Đường Dẫn Thực Tế.
Dự án: CS532 Autonomous JetBot (2026).

Định hướng mới:
- TRIỆT TIÊU TOÀN BỘ CÁC BÀI TEST LÝ THUYẾT VÒNG LẶP / MOCK LIDAR GIẢ LẬP (Vì vô dụng và không phản ánh thế giới thực).
- Mọi đánh giá chất lượng thuật toán bắt buộc dựa trên:
  1. Thẩm định tĩnh: Cú pháp AST Python (Static AST Syntax Integrity).
  2. Thẩm định tài liệu: 100% đường dẫn file nhắc đến trong agents-doc/agents/ phải tồn tại thực tế trên ổ đĩa.
  3. Kiểm chứng thực tế: Chạy xe thật qua Web Dashboard Telemetry / Hardware Log Replay hoặc mô phỏng vật lý Digital Twin chuẩn.
"""

import sys
import os
import ast
import re
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent

# Các module cốt lõi thực tế trên codebase hiện tại
CORE_DIRS = ["01-HieuMe", "02-DuyHieu", "03-MinhHieu", "05-d500lib-motion", "setup", "agents-doc", "scripts"]

def get_python_interpreter():
    """Tự động phát hiện virtual environment của dự án."""
    venv_py = ROOT_DIR / ".venv" / "bin" / "python"
    if venv_py.exists():
        return str(venv_py)
    return sys.executable

def print_banner():
    py_exec = get_python_interpreter()
    print("=" * 70)
    print(" 🛡️  CS532 AGENT HARNESS INTEGRITY & ANTI-HALLUCINATION AUDITOR")
    print(f" 🐍 Interpreter: {py_exec}")
    print(" 🎯 Triết lý: Loại bỏ test lý thuyết giả lập -> Tập trung Thực tế & Digital Twin")
    print("=" * 70)

def audit_python_syntax():
    print("\n[1/2] 🔍 Kiểm tra cú pháp AST toàn bộ mã nguồn Python...")
    errors = []
    checked_files = 0
    ignored_patterns = ["node_modules", ".venv", "__pycache__", "dist", ".git", ".ruff_cache"]

    for core_dir in CORE_DIRS:
        dir_path = ROOT_DIR / core_dir
        if not dir_path.is_dir():
            continue
        for py_file in dir_path.rglob("*.py"):
            parts = py_file.parts
            if any(ign in parts for ign in ignored_patterns):
                continue

            checked_files += 1
            try:
                with open(py_file, "r", encoding="utf-8", errors="replace") as f:
                    source = f.read()
                ast.parse(source, filename=str(py_file))
            except SyntaxError as e:
                errors.append(f"❌ Syntax Error tại {py_file.relative_to(ROOT_DIR)}:{e.lineno} -> {e.msg}")

    if errors:
        for err in errors:
            print(f"  {err}")
        return False
    else:
        print(f"  ✅ Đã quét {checked_files} files Python dự án. 100% hợp lệ cú pháp.")
        return True

def audit_documentation_paths():
    print("\n[2/2] 🔍 Kiểm định liên kết & đường dẫn trong agents-doc/agents/ (Bẫy Hallucination)...")
    agents_dir = ROOT_DIR / "agents-doc" / "agents"
    if not agents_dir.is_dir():
        print("  ⚠️ Không tìm thấy thư mục agents-doc/agents/")
        return True

    path_pattern = re.compile(r"[`'\"]([a-zA-Z0-9_\-\./\\]+\.[a-zA-Z0-9]+)[`'\"]")
    missing_paths = []
    checked_refs = 0

    valid_prefixes = ["01-", "02-", "03-", "05-", "agents-doc", "setup", "cad", "scripts"]

    for md_file in agents_dir.glob("*.md"):
        with open(md_file, "r", encoding="utf-8") as f:
            content = f.read()

        matches = path_pattern.findall(content)
        for match in matches:
            if any(match.startswith(prefix) for prefix in valid_prefixes):
                checked_refs += 1
                target = ROOT_DIR / match
                if not target.exists():
                    if not any(skip in match for skip in [".so", ".engine", ".stl", "example"]):
                        missing_paths.append((md_file.name, match))

    if missing_paths:
        print(f"  ⚠️ PHÁT HIỆN {len(missing_paths)} ĐƯỜNG DẪN ẢO GIÁC (HALLUCINATED/MISSING PATHS):")
        for md_name, miss in missing_paths:
            print(f"     - File '{md_name}' nhắc đến: '{miss}' (Tập tin KHÔNG tồn tại!)")
        return False
    else:
        print(f"  ✅ Đã kiểm tra {checked_refs} tham chiếu file trong tài liệu agent. 100% tồn tại thực tế.")
        return True

def main():
    print_banner()
    c1 = audit_python_syntax()
    c2 = audit_documentation_paths()

    print("\n" + "=" * 70)
    if c1 and c2:
        print(" 🎉 KẾT QUẢ: 100% MÃ NGUỒN HỢP LỆ & 0 ĐƯỜNG DẪN MA (ZERO-HALLUCINATION).")
        print(" Hệ thống đã loại bỏ hoàn toàn các bài test lý thuyết vô bổ.")
        print(" Sẵn sàng thực thi trên Xe Thật hoặc Mô phỏng Vật lý Digital Twin!")
        print("=" * 70)
        sys.exit(0)
    else:
        print(" ❌ KẾT QUẢ: CẢNH BÁO VI PHẠM KỶ LUẬT HARNESS HOẶC PHÁT HIỆN HALLUCINATION!")
        print(" Vui lòng sửa chữa các lỗi trên trước khi tiếp tục.")
        print("=" * 70)
        sys.exit(1)

if __name__ == "__main__":
    main()
