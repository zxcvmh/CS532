### TK-INT-02: Legacy tests still point at dead `04-hieu-MotionControl` paths
- Role: integration
- Status: DRAFT
- Problem: `motion/test_*.py` reference `04-hieu-MotionControl/` (no such folder).
- Depends on: TK-SLAM-01

---

## SLAM findings from the TK-SLAM-01 audit (2026-11-15)

Simulated 2×1 m arena, real `MotionAdapter` code, in-place 360° turn (threshold ≤ 5 cm / 3°):
ideal scan @5.24 rad/s 0.0°/1.6 cm ✅ · distorted scan −22.8°/8.5 cm ❌ · distorted + anchor off
(= production) −37.0°/0.5 cm ❌ · ideal scan @0.35 rad/s −10.0°/16.6 cm ❌ · slow + distorted
+4.7°/36.5 cm ❌. Repro scripts (auditor scratchpad, to be turned into a locked test by the Supervisor):
`repro_slam.py`, `repro4.py`.
