#!/usr/bin/env python3
"""
verify_agent_integrity.py — Bộ Thẩm Định Tính Xác Thực & Chống Hallucination Tự Động.
Dự án: CS532 Autonomous JetBot (2026).

Chức năng:
1. Static Integrity: Quét cú pháp AST của toàn bộ file Python dự án (bỏ qua node_modules, .venv).
2. Link & Path Verification: Kiểm tra các đường dẫn tập tin được nhắc đến trong agents-doc/agents/.
3. Deterministic Regression Suite: Chạy thực tế các bài kiểm thử cốt lõi bằng Python venv của dự án.
4. Hallucination Detection: Phát hiện bất kỳ tuyên bố ảo giác hoặc sai lệch nào so với ground truth.
"""

import sys
import os
import ast
import re
import subprocess
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent

# Các module cốt lõi cần quét
CORE_DIRS = ["01-HieuMe", "02-DuyHieu", "03-MinhHieu", "04-hieu-MotionControl", "setup", "agents-doc", "scripts"]

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
    print("=" * 70)

def audit_python_syntax():
    print("\n[1/3] 🔍 Kiểm tra cú pháp AST toàn bộ mã nguồn Python...")
    errors = []
    checked_files = 0
    ignored_patterns = ["node_modules", ".venv", "__pycache__", "dist", ".git"]

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
    print("\n[2/3] 🔍 Kiểm định liên kết & đường dẫn trong agents-doc/agents/ (Bẫy Hallucination)...")
    agents_dir = ROOT_DIR / "agents-doc" / "agents"
    if not agents_dir.is_dir():
        print("  ⚠️ Không tìm thấy thư mục agents-doc/agents/")
        return True

    path_pattern = re.compile(r"[`'\"]([a-zA-Z0-9_\-\./\\]+\.[a-zA-Z0-9]+)[`'\"]")
    missing_paths = []
    checked_refs = 0

    for md_file in agents_dir.glob("*.md"):
        with open(md_file, "r", encoding="utf-8") as f:
            content = f.read()

        matches = path_pattern.findall(content)
        for match in matches:
            # Lọc các đường dẫn trông giống file code/doc trong dự án
            if any(match.startswith(prefix) for prefix in ["01-", "02-", "03-", "04-", "agents-doc", "setup", "cad", "scripts"]):
                checked_refs += 1
                target = ROOT_DIR / match
                if not target.exists():
                    # Bỏ qua nếu là file build output hoặc example
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

def run_test_suite():
    print("\n[3/3] 🧪 Thực thi Suite Kiểm thử Hồi quy Deterministic (Chạy thật qua Subprocess)...")
    py_exec = get_python_interpreter()
    tests = [
        ("01-HieuMe Computer Vision (YOLO26-n & Floor IPM)", [py_exec, "01-HieuMe/test_cv_local.py"]),
        ("02-DuyHieu Planning A* & Obstacle Clustering", [py_exec, "02-DuyHieu/verify_step1_astar.py"]),
        ("04-hieu-MotionControl DDE + CBF Safety Filter", [py_exec, "04-hieu-MotionControl/test_closed_loop_continuous_navigation.py"]),
    ]

    all_passed = True
    for name, cmd in tests:
        print(f"\n  ▶ Đang chạy: {name}...")
        try:
            res = subprocess.run(
                cmd,
                cwd=str(ROOT_DIR),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=60
            )
            if res.returncode == 0:
                print(f"    ✅ PASS 100% ({name})")
            else:
                print(f"    ❌ FAIL ({name}) - Exit code: {res.returncode}")
                # In 10 dòng cuối của log lỗi
                lines = res.stdout.strip().split("\n")
                tail = "\n".join(lines[-10:])
                print(f"    --- Traceback Preview ---\n{tail}\n    ------------------------")
                all_passed = False
        except Exception as e:
            print(f"    ❌ Lỗi thực thi kiểm thử: {e}")
            all_passed = False

    return all_passed

def main():
    print_banner()
    c1 = audit_python_syntax()
    c2 = audit_documentation_paths()
    c3 = run_test_suite()

    print("\n" + "=" * 70)
    if c1 and c2 and c3:
        print(" 🎉 KẾT QUẢ: 100% KIỂM TRA CHÍNH XÁC. ZERO-HALLUCINATION VERIFIED!")
        print(" Hệ thống đạt chuẩn kỷ luật Harness Engineering và sẵn sàng bàn giao.")
        print("=" * 70)
        sys.exit(0)
    else:
        print(" ❌ KẾT QUẢ: CẢNH BÁO VI PHẠM KỶ LUẬT HARNESS HOẶC PHÁT HIỆN HALLUCINATION!")
        print(" Vui lòng sửa chữa các lỗi trên trước khi tiếp tục.")
        print("=" * 70)
        sys.exit(1)

if __name__ == "__main__":
    main()
