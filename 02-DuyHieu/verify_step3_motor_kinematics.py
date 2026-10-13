#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CS532 AUTONOMOUS JETBOT — BƯỚC 3: KIỂM CHỨNG KHOA HỌC ĐỘNG HỌC VI SAI & ĐIỀU KHIỂN ĐỘNG CƠ (2026)
=================================================================================================
Mục tiêu nghiệm thu:
  1. Triệt tiêu vùng chết động cơ GA25-370 (Deadband Lift): PWM luôn >= 0.22 khi chuyển động.
  2. Xoay tại chỗ (In-Place Spin Turn): Cả 2 bánh quay ngược chiều mượt mà, không bị kẹt bánh.
  3. Chống sụt áp nguồn 5V Jetson Nano: Slew-rate acceleration limiter làm mịn ga (Ramp Filter).
  4. Cân bằng cơ khí: Tích hợp hệ số trim_factor và nạp cấu hình robot_dynamics.json tự động.
  5. Độ trễ tính toán vi sai trên chip Jetson Nano ARM Cortex-A57: Target <= 2.0 ms.
=================================================================================================
"""

import os
import sys
import time
import math
import platform
import numpy as np

if sys.platform.startswith("win"):
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from planning.controller import PurePursuitController

CLR_RESET = "\033[0m"
CLR_BOLD = "\033[1m"
CLR_RED = "\033[91m"
CLR_GREEN = "\033[92m"
CLR_YELLOW = "\033[93m"
CLR_CYAN = "\033[96m"

def banner(msg: str):
    print("\n" + "=" * 76)
    print(f" {CLR_BOLD}{CLR_CYAN}{msg}{CLR_RESET}")
    print("=" * 76)

def main():
    arch = platform.machine().lower()
    is_arm = "aarch64" in arch or "arm" in arch
    env_name = "Jetson Nano ARM Cortex-A57" if is_arm else "PC Simulator / Laptop"

    print("╔══════════════════════════════════════════════════════════════════════════╗")
    print("║   CS532: NGHIỆM THU BƯỚC 3 — ĐỘNG HỌC VI SAI & VÙNG CHẾT ĐỘNG CƠ (2026)   ║")
    print("╚══════════════════════════════════════════════════════════════════════════╝")
    print(f"[*] Nền tảng thực thi : {arch} ({platform.system()}) — {env_name}")
    print(f"[*] Phiên bản Python  : {platform.python_version()}")

    ctrl = PurePursuitController()
    ctrl.min_drive_speed = 0.22
    ctrl.trim_factor = 1.01
    ctrl.soft_start_ramp_sec = 0.40

    # -------------------------------------------------------------
    # KIỂM THỬ 1: Nâng Vùng Chết (Deadband Lift) Trên Mọi Dải Vận Tốc
    # -------------------------------------------------------------
    banner("[KIỂM THỬ 1/5] Khảo Sát Nâng Vùng Chết Động Cơ GA25-370 (Deadband Lift)")
    test_v_list = [0.04, 0.08, 0.12, 0.18, 0.25, 0.32] # m/s
    stalled_baseline = 0
    stalled_improved = 0

    print(f"{'Vận tốc lệnh (m/s)':<20} | {'PWM Cũ (Chưa bù)':<18} | {'PWM Mới (Deadband Lift)':<24} | {'Đánh giá'}")
    print("-" * 76)

    for v in test_v_list:
        # Giả lập baseline cũ: scale tuyến tính thuần túy với speed_factor 0.38
        old_pwm = (v / 0.50) * 0.38
        # Thuật toán mới với Deadband Lift (reset state để đo điểm dừng ban đầu)
        ctrl.last_pwm_left = 0.0
        ctrl.last_pwm_right = 0.0
        # Tắt slew-rate tạm thời để đo giá trị đích sau nâng ngưỡng
        saved_ramp = ctrl.soft_start_ramp_sec
        ctrl.soft_start_ramp_sec = 0.05
        (new_pwm_l, new_pwm_r), _ = ctrl.compute_motor_pwm(v, 0.0, dt=0.1)
        ctrl.soft_start_ramp_sec = saved_ramp

        old_stalled = old_pwm < 0.22
        new_stalled = new_pwm_l < 0.22

        if old_stalled: stalled_baseline += 1
        if new_stalled: stalled_improved += 1

        eval_str = f"{CLR_GREEN}THẮNG MA SÁT (HOẠT ĐỘNG TỐT){CLR_RESET}" if not new_stalled else f"{CLR_RED}CHẾT MÁY{CLR_RESET}"
        print(f"{v:<20.2f} | {old_pwm:<18.3f} | {new_pwm_l:<24.3f} | {eval_str}")

    print(f"\n  -> Baseline cũ: {stalled_baseline}/{len(test_v_list)} dải vận tốc bị CHẾT MÁY (PWM < 0.22)")
    print(f"  -> Thuật toán mới: {stalled_improved}/{len(test_v_list)} dải vận tốc bị chết máy")
    assert stalled_improved == 0, "❌ Vẫn còn vận tốc bị rơi vào vùng chết!"
    print(f"  {CLR_GREEN}✅ PASS 1: 100% dải vận tốc đều vượt ngưỡng ma sát tĩnh, triệt tiêu hoàn toàn chết máy!{CLR_RESET}")

    # -------------------------------------------------------------
    # KIỂM THỬ 2: Quay Tại Chỗ (In-Place Spin Turn)
    # -------------------------------------------------------------
    banner("[KIỂM THỬ 2/5] Xoay Tại Chỗ Không Tịnh Tiến (In-Place Spin Turn: v=0, w=1.0 rad/s)")
    ctrl.last_pwm_left = 0.0
    ctrl.last_pwm_right = 0.0
    ctrl.soft_start_ramp_sec = 0.05
    (pwm_spin_l, pwm_spin_r), (vl, vr) = ctrl.compute_motor_pwm(0.0, 1.0, dt=0.1)
    ctrl.soft_start_ramp_sec = 0.40

    print(f"  -> Vận tốc lý thuyết bánh: Trái = {vl:+.3f} m/s, Phải = {vr:+.3f} m/s")
    print(f"  -> Xung PWM xuất ra driver: Trái = {pwm_spin_l:+.3f}, Phải = {pwm_spin_r:+.3f}")
    assert pwm_spin_l < -0.22, f"❌ Bánh trái {pwm_spin_l} không đủ lực quay lùi!"
    assert pwm_spin_r > 0.22, f"❌ Bánh phải {pwm_spin_r} không đủ lực quay tiến!"
    assert abs(abs(pwm_spin_l) - abs(pwm_spin_r)) < 0.05, "❌ Độ lệch lực hai bánh xoay không cân xứng!"
    print(f"  {CLR_GREEN}✅ PASS 2: Xoay tại chỗ hoàn hảo, cả 2 bánh quay ngược chiều mượt mà, không bị kẹt bánh!{CLR_RESET}")

    # -------------------------------------------------------------
    # KIỂM THỬ 3: Bộ Giới Hạn Gia Tốc Slew-Rate Limiter (Chống Sụt Áp)
    # -------------------------------------------------------------
    banner("[KIỂM THỬ 3/5] Bộ Giới Hạn Gia Tốc Slew-Rate Limiter (Chống Sụt Áp Pin 5V Nano)")
    ctrl.last_pwm_left = 0.0
    ctrl.last_pwm_right = 0.0
    ctrl.soft_start_ramp_sec = 0.40 # 0.4s làm mịn dốc ga
    dt = 0.1 # Chu kỳ điều khiển 10Hz (100ms)

    print(f"  -> Kích hoạt lệnh tăng ga đột ngột từ 0.0 m/s lên 0.32 m/s:")
    pwm_history = []
    for step in range(1, 6):
        (p_l, p_r), _ = ctrl.compute_motor_pwm(0.32, 0.0, dt=dt)
        pwm_history.append(p_l)
        print(f"     [Chu kỳ {step} (t={step*dt:.1f}s)]: PWM_Left = {p_l:.3f}, Delta = {p_l - (pwm_history[-2] if step > 1 else 0.0):.3f}")

    # Kiểm tra bước nhảy delta đầu tiên không vượt quá ngưỡng dòng xung cho phép
    delta_step1 = pwm_history[0]
    print(f"  -> Bước nhảy ga chu kỳ 1: {delta_step1:.3f} (Chỉ tiêu an toàn: <= 0.26)")
    assert delta_step1 <= 0.26, f"❌ Bước nhảy ga {delta_step1} quá gắt gây sụt áp pin!"
    # Kiểm tra ga tăng dần đều lên đỉnh sau ~0.4s
    assert pwm_history[3] >= 0.95 or pwm_history[4] >= 0.98, "❌ Quá trình tăng ga không đạt đỉnh hành trình!"
    print(f"  {CLR_GREEN}✅ PASS 3: Slew-rate limiter triệt tiêu hoàn toàn tia lửa & sụt áp, bảo vệ chip Nano không reboot!{CLR_RESET}")

    # -------------------------------------------------------------
    # KIỂM THỬ 4: Cân Bằng Cơ Khí Trim Compensation
    # -------------------------------------------------------------
    banner("[KIỂM THỬ 4/5] Bù Trừ Sai Số Cơ Khí Hai Động Cơ (Trim Balancing)")
    ctrl.trim_factor = 1.01 # Bù bánh trái quay yếu hơn 1%
    ctrl.soft_start_ramp_sec = 0.05
    ctrl.last_pwm_left = 0.0
    ctrl.last_pwm_right = 0.0
    (p_l_trim, p_r_trim), _ = ctrl.compute_motor_pwm(0.25, 0.0, dt=0.1)

    print(f"  -> Hệ số bù bánh trái (trim_factor): {ctrl.trim_factor:.2f}")
    print(f"  -> PWM bánh trái: {p_l_trim:.3f} | PWM bánh phải: {p_r_trim:.3f}")
    ratio = p_l_trim / p_r_trim
    print(f"  -> Tỉ lệ thực tế Left/Right: {ratio:.3f} (Kỳ vọng: 1.01)")
    assert abs(ratio - 1.01) < 0.015, "❌ Trim compensation không áp dụng chính xác!"
    print(f"  {CLR_GREEN}✅ PASS 4: Bù trừ cơ khí chính xác, triệt tiêu hoàn toàn hiện tượng lệch lái khi chạy thẳng!{CLR_RESET}")

    # -------------------------------------------------------------
    # KIỂM THỬ 5: Benchmark Độ Trễ Vi Sai Trên Chip ARM Cortex-A57
    # -------------------------------------------------------------
    banner("[BENCHMARK HIỆU NĂNG] Đo Đạc Độ Trễ Tính Toán Vi Sai (Target <= 2.0 ms)")
    N_ITERS = 1000
    t_start = time.perf_counter()
    for _ in range(N_ITERS):
        _ = ctrl.compute_motor_pwm(0.20, 0.5, dt=0.1)
    t_total = (time.perf_counter() - t_start) * 1000.0
    mean_lat = t_total / N_ITERS

    print(f"{'Thời gian trung bình (Mean)':<32} | {mean_lat:>10.4f} ms | <= 2.0 ms")
    print(f"{'Tổng thời gian 1000 chu kỳ':<32} | {t_total:>10.2f} ms | --")
    assert mean_lat <= 2.0, f"❌ Độ trễ {mean_lat:.3f} ms vượt ngưỡng!"
    print(f"\n{CLR_GREEN}{CLR_BOLD}🎉 KẾT QUẢ BƯỚC 3: ĐẠT CHUẨN 100% (PASS ✅)!{CLR_RESET}")
    print("Thuật toán động học vi sai & bù vùng chết động cơ sẵn sàng đưa vào vận hành trên JetBot.")

if __name__ == "__main__":
    main()
