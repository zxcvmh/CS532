"""
Evaluation Benchmark Suite for JetBot Imitation Learning Policy
================================================================
CS532 Autonomous JetBot — Module 07-imitation-learning
Based on UIT ITCourses RL-101 (Bài 10: Học bắt chước - Imitation Learning)

Standard Robotics Navigation Evaluation Protocol:
  1. Benchmark Suites (50 total episodes):
     - Suite A: Omnidirectional 360-Degree Target Reaching (16 episodes)
     - Suite B: Fine-Grained Ca Nước (Small Obstacle) Bypass (16 episodes)
     - Suite C: Multi-Obstacle Slalom & Clutter Corridors (10 episodes)
     - Suite D: Local Lookahead Waypoint Tracking (8 episodes)

  2. Quantitative Evaluation Metrics:
     - SR  : Success Rate (%)
     - CR  : Collision Rate (%)
     - SPL : Success weighted by Path Length (%) [Anderson et al., 2018]
             SPL = (1/N) * sum_i (S_i * L_i / max(P_i, L_i))
     - TTG : Mean Time to Goal (seconds)
     - Jerk: Mean Angular Jerk (rad/s^2) - measures steering oscillation/smoothness
     - Clr : Obstacle Minimum Safety Clearance (meters)
     - Lat : Inference Latency (milliseconds per step)
"""

import os
import sys
import time
import math
import json
import random
import argparse
from typing import List, Tuple, Dict, Any, Optional
import numpy as np

# Ensure UTF-8 output on Windows consoles
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")


# Add project root and module to path
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from config import (
    CONTROL_HZ,
    GOAL_TOLERANCE_M,
    ROBOT_RADIUS_M,
    LIDAR_MAX_RANGE_M,
    MAX_LINEAR_SPEED,
    MAX_ANGULAR_SPEED
)
from mock_expert_simulator import SimulationEnvironment
from il_policy_runner import ILPolicyCoordinatorRunner


class EvaluationEpisode:
    def __init__(
        self,
        name: str,
        suite: str,
        start_pose: Tuple[float, float, float],
        goal: Tuple[float, float],
        obstacles: List[Tuple[float, float, float]],
        lookahead_waypoints: Optional[List[Tuple[float, float]]] = None,
        max_steps: int = 250
    ):
        self.name = name
        self.suite = suite
        self.start_pose = start_pose
        self.goal = goal
        self.obstacles = obstacles
        self.lookahead_waypoints = lookahead_waypoints
        self.max_steps = max_steps


def create_benchmark_suites() -> List[EvaluationEpisode]:
    episodes: List[EvaluationEpisode] = []

    # ---------------------------------------------------------
    # SUITE A: Omnidirectional 360-Degree Target Reaching (16 episodes)
    # Robot starts in center (1.5, 0.0), goal placed at diverse angles [-180, +180]
    # ---------------------------------------------------------
    angles_deg = [
        0.0, 20.0, -20.0, 45.0, -45.0, 60.0, -60.0, 85.0,
        -85.0, 110.0, -110.0, 135.0, -135.0, 160.0, -160.0, 180.0
    ]
    for deg in angles_deg:
        rad = math.radians(deg)
        dist = 1.05
        gx = 1.50 + dist * math.cos(rad)
        gy = 0.00 + dist * math.sin(rad)
        episodes.append(
            EvaluationEpisode(
                name=f"Omni 360: Goal at {deg:+4.0f} deg (dist={dist:.2f}m)",
                suite="Suite A: Omnidirectional 360",
                start_pose=(1.50, 0.00, 0.00),
                goal=(gx, gy),
                obstacles=[],
                max_steps=220
            )
        )

    # ---------------------------------------------------------
    # SUITE B: Fine-Grained Ca Nuoc (Cup Obstacle) Bypass (16 episodes)
    # Small obstacle (radius 0.04m - 0.06m) placed in path.
    # Evaluates smooth S-curve bypass without overshooting or getting stuck.
    # ---------------------------------------------------------
    ca_nuoc_configs = [
        # (ox, oy, oradius, start_theta, desc)
        (1.20,  0.00, 0.045,  0.0, "Dead Center 0.7m Ahead"),
        (1.10,  0.02, 0.045,  0.0, "Slight Left Offset (+2cm)"),
        (1.10, -0.02, 0.045,  0.0, "Slight Right Offset (-2cm)"),
        (1.30,  0.05, 0.050,  0.0, "Moderate Left Offset (+5cm)"),
        (1.30, -0.05, 0.050,  0.0, "Moderate Right Offset (-5cm)"),
        (1.00,  0.08, 0.060,  0.0, "Wide Left Offset (+8cm)"),
        (1.00, -0.08, 0.060,  0.0, "Wide Right Offset (-8cm)"),
        (1.20,  0.00, 0.050,  0.15, "Dead Center with Initial +8.6 deg Yaw"),
        (1.20,  0.00, 0.050, -0.15, "Dead Center with Initial -8.6 deg Yaw"),
        (1.25,  0.03, 0.045,  0.25, "Left Offset with Initial +14.3 deg Yaw"),
        (1.25, -0.03, 0.045, -0.25, "Right Offset with Initial -14.3 deg Yaw"),
        (1.15,  0.00, 0.040,  0.0, "Small Cup (r=4cm) Direct Ahead"),
        (1.40,  0.00, 0.040,  0.0, "Small Cup Far Ahead (1.4m)"),
        (0.95,  0.00, 0.045,  0.0, "Small Cup Close Ahead (0.95m)"),
        (1.10,  0.04, 0.040, -0.20, "Diagonal Approach to Left Cup"),
        (1.10, -0.04, 0.040,  0.20, "Diagonal Approach to Right Cup"),
    ]
    for ox, oy, r_obs, theta_init, desc in ca_nuoc_configs:
        episodes.append(
            EvaluationEpisode(
                name=f"Ca Nuoc: {desc}",
                suite="Suite B: Ca Nuoc Bypass",
                start_pose=(0.50, 0.00, theta_init),
                goal=(2.30, 0.00),
                obstacles=[(ox, oy, r_obs)],
                max_steps=240
            )
        )


    # ---------------------------------------------------------
    # SUITE C: Multi-Obstacle Slalom & Clutter Corridors (10 episodes)
    # Staggered obstacles requiring alternating bypass.
    # ---------------------------------------------------------
    slalom_configs = [
        ([ (1.00,  0.10, 0.06), (1.60, -0.10, 0.06) ], "Double S-Curve Slalom (Left-Right)"),
        ([ (1.00, -0.10, 0.06), (1.60,  0.10, 0.06) ], "Double S-Curve Slalom (Right-Left)"),
        ([ (0.95,  0.12, 0.07), (1.45, -0.12, 0.07), (1.95, 0.08, 0.06) ], "Triple Obstacle Wiggle"),
        ([ (1.10,  0.00, 0.06), (1.80,  0.00, 0.06) ], "Two Inline Obstacles Along Centerline"),
        ([ (1.20,  0.14, 0.08), (1.20, -0.14, 0.08) ], "Chicane Gate (Narrow Doorway 0.28m)"),
        ([ (0.90,  0.06, 0.05), (1.30, -0.06, 0.05), (1.70, 0.00, 0.06) ], "Triangular Constriction"),
        ([ (1.10, -0.08, 0.06), (1.50,  0.08, 0.06), (1.90, -0.05, 0.05) ], "Corridor Zigzag"),
        ([ (1.00,  0.00, 0.08) ], "Larger Single Obstacle (r=8cm) Dead Center"),
        ([ (1.30, -0.02, 0.07) ], "Larger Single Obstacle (r=7cm) Dead Center"),
        ([ (1.10,  0.12, 0.06), (1.70, -0.08, 0.06) ], "Asymmetric Slalom Path"),
    ]
    for obs_list, desc in slalom_configs:
        episodes.append(
            EvaluationEpisode(
                name=f"Slalom: {desc}",
                suite="Suite C: Multi-Obstacle Slalom",
                start_pose=(0.40, 0.00, 0.00),
                goal=(2.35, 0.00),
                obstacles=obs_list,
                max_steps=260
            )
        )

    # ---------------------------------------------------------
    # SUITE D: Local Lookahead Waypoint Tracking (8 episodes)
    # Simulates coordinator lookahead along global path
    # ---------------------------------------------------------
    waypoint_routes = [
        ([(0.8, 0.2), (1.4, 0.2), (2.0, 0.0)], (2.2, 0.0), [(1.1, 0.0, 0.05)], "L-Curved Corridor with Obstacle"),
        ([(0.8, -0.2), (1.4, -0.2), (2.0, 0.0)], (2.2, 0.0), [(1.1, 0.0, 0.05)], "Inverted L-Curved Corridor"),
        ([(0.9, 0.3), (1.5, 0.3), (2.1, 0.1)], (2.3, 0.0), [], "North-Shifted Global A* Path"),
        ([(0.9, -0.3), (1.5, -0.3), (2.1, -0.1)], (2.3, 0.0), [], "South-Shifted Global A* Path"),
        ([(0.7, 0.15), (1.2, -0.15), (1.8, 0.10)], (2.2, 0.0), [(1.5, 0.0, 0.05)], "Meandering Path Around Center Pillar"),
        ([(1.0, 0.0), (1.5, 0.25), (2.0, 0.0)], (2.4, 0.0), [(1.5, -0.10, 0.06)], "Lateral Detour Waypoint Tracking"),
        ([(0.8, -0.1), (1.4, -0.1), (1.9, 0.1)], (2.2, 0.2), [], "Diagonal Lookahead Waypoints"),
        ([(0.7, 0.0), (1.2, 0.0), (1.8, 0.0)], (2.3, 0.0), [(1.2, 0.0, 0.045)], "Linear Waypoints with Ca Nuoc Insertion"),

    ]
    for wps, final_goal, obs, desc in waypoint_routes:
        episodes.append(
            EvaluationEpisode(
                name=f"Lookahead: {desc}",
                suite="Suite D: Lookahead Waypoints",
                start_pose=(0.40, 0.00, 0.00),
                goal=final_goal,
                obstacles=obs,
                lookahead_waypoints=wps,
                max_steps=250
            )
        )

    return episodes


def evaluate_single_episode(
    episode: EvaluationEpisode,
    policy_runner: ILPolicyCoordinatorRunner,
    dt: float = 0.10
) -> Dict[str, Any]:
    """Runs a single episode simulation and calculates all formal metrics."""
    env = SimulationEnvironment(arena_width=3.0, arena_height=2.4)
    env.reset(
        custom_start=episode.start_pose,
        custom_goal=episode.goal,
        num_obstacles=0
    )
    env.obstacles = list(episode.obstacles)

    # Initialize lookahead tracking if provided
    remaining_wps = list(episode.lookahead_waypoints) if episode.lookahead_waypoints else []

    trajectory_x = [env.x]
    trajectory_y = [env.y]
    w_history = [env.w]
    latencies = []
    min_clearance = float("inf")

    total_path_length = 0.0
    start_dist_to_goal = math.hypot(episode.goal[0] - env.x, episode.goal[1] - env.y)

    success = False
    collision = False
    collision_reason = ""
    step_count = 0

    # Reset runner internal rate-limiters
    policy_runner.active_v = 0.0
    policy_runner.active_w = 0.0
    policy_runner.last_step_time = 0.0

    for step in range(episode.max_steps):
        step_count += 1

        # Determine target point: Lookahead waypoint or final goal
        current_target_x, current_target_y = episode.goal
        if remaining_wps:
            wp_x, wp_y = remaining_wps[0]
            if math.hypot(wp_x - env.x, wp_y - env.y) < 0.25:
                remaining_wps.pop(0)
            if remaining_wps:
                current_target_x, current_target_y = remaining_wps[0]

        # Generate 360-degree LiDAR scan
        lidar_scan = env.get_lidar_scan()


        # Policy forward pass
        v_cmd, w_cmd, diag = policy_runner.compute_command(
            lidar_ranges_360=lidar_scan,
            goal_x=current_target_x,
            goal_y=current_target_y,
            robot_x=env.x,
            robot_y=env.y,
            robot_theta=env.theta,
            curr_v=env.v,
            curr_w=env.w
        )
        latencies.append(diag.get("latency_ms", 0.05))

        # Check collision with obstacles
        for ox, oy, oradius in env.obstacles:
            center_dist = math.hypot(env.x - ox, env.y - oy)
            clearance = center_dist - ROBOT_RADIUS_M - oradius
            if clearance < min_clearance:
                min_clearance = clearance
            if center_dist < (ROBOT_RADIUS_M + oradius):
                collision = True
                collision_reason = f"Obstacle Hit at ({ox:.2f}, {oy:.2f})"
                break

        # Check collision with arena boundaries
        half_h = env.arena_h / 2.0
        wall_margin = ROBOT_RADIUS_M + 0.03
        if (env.x < wall_margin or env.x > (env.arena_w - wall_margin) or
            env.y < (-half_h + wall_margin) or env.y > (half_h - wall_margin)):
            collision = True
            collision_reason = f"Arena Wall Hit at ({env.x:.2f}, {env.y:.2f})"

        if collision:
            break

        # Step physics
        prev_x, prev_y = env.x, env.y
        env.step(v_cmd, w_cmd, dt=dt)
        step_dist = math.hypot(env.x - prev_x, env.y - prev_y)
        total_path_length += step_dist

        trajectory_x.append(env.x)
        trajectory_y.append(env.y)
        w_history.append(env.w)

        # Check arrival at final goal
        dist_final_goal = math.hypot(episode.goal[0] - env.x, episode.goal[1] - env.y)
        if dist_final_goal <= GOAL_TOLERANCE_M:
            success = True
            break

    # Calculate Jerk: mean |w_t - w_{t-1}| / dt
    angular_jerk = 0.0
    if len(w_history) > 1:
        diffs = [abs(w_history[i] - w_history[i - 1]) / dt for i in range(1, len(w_history))]
        angular_jerk = float(np.mean(diffs))

    # Calculate SPL (Success weighted by Path Length)
    if success:
        spl = (start_dist_to_goal / max(total_path_length, start_dist_to_goal)) * 100.0
    else:
        spl = 0.0

    time_to_goal = step_count * dt

    return {
        "name": episode.name,
        "suite": episode.suite,
        "success": success,
        "collision": collision,
        "collision_reason": collision_reason,
        "steps": step_count,
        "time_to_goal_s": round(time_to_goal, 2),
        "path_length_m": round(total_path_length, 3),
        "geodesic_m": round(start_dist_to_goal, 3),
        "spl_pct": round(spl, 1),
        "angular_jerk": round(angular_jerk, 2),
        "min_clearance_m": round(min_clearance, 3) if min_clearance != float("inf") else 0.50,
        "avg_latency_ms": round(float(np.mean(latencies)), 3)
    }


def run_full_benchmark(
    model_path: str = "models/bc_policy.onnx",
    report_path: str = "logs/evaluation_benchmark_report.json"
) -> Dict[str, Any]:
    print("\n" + "=" * 80)
    print("      CS532 AUTONOMOUS JETBOT -- IMITATION LEARNING BENCHMARK EVALUATION")
    print("      Testing Omnidirectional 360, Ca Nuoc Obstacle Bypass & Lookahead")
    print("=" * 80)


    # Initialize Runner
    runner = ILPolicyCoordinatorRunner(model_path=model_path)
    print(f"[BENCHMARK] Active Inference Engine : {runner.engine_type}")
    print(f"[BENCHMARK] Target Model Path       : {model_path}")

    episodes = create_benchmark_suites()
    print(f"[BENCHMARK] Total Evaluation Episodes: {len(episodes)}")
    print(f"[BENCHMARK] Starting automated evaluation run...\n")

    results: List[Dict[str, Any]] = []
    suites_summary: Dict[str, Dict[str, Any]] = {}

    t_start = time.time()
    for idx, ep in enumerate(episodes, 1):
        res = evaluate_single_episode(ep, runner)
        results.append(res)

        # Status symbol
        if res["success"]:
            tag = "[PASS: SUCCESS]"
        elif res["collision"]:
            tag = f"[FAIL: COLLISION - {res['collision_reason']}]"
        else:
            tag = "[FAIL: TIMEOUT]"

        print(
            f"  Ep {idx:02d}/50 | {ep.suite[:18]:<18} | {res['name'][:34]:<34} | "
            f"SPL: {res['spl_pct']:5.1f}% | TTG: {res['time_to_goal_s']:4.1f}s | {tag}"
        )

    t_elapsed = time.time() - t_start

    # Aggregate by Suite
    suite_names = sorted(list(set(r["suite"] for r in results)))
    for s_name in suite_names:
        s_res = [r for r in results if r["suite"] == s_name]
        n_eps = len(s_res)
        n_succ = sum(1 for r in s_res if r["success"])
        n_coll = sum(1 for r in s_res if r["collision"])
        sr = (n_succ / n_eps) * 100.0
        cr = (n_coll / n_eps) * 100.0
        mean_spl = float(np.mean([r["spl_pct"] for r in s_res]))
        mean_ttg = float(np.mean([r["time_to_goal_s"] for r in s_res if r["success"]])) if n_succ > 0 else 0.0
        mean_jerk = float(np.mean([r["angular_jerk"] for r in s_res]))
        mean_clr = float(np.mean([r["min_clearance_m"] for r in s_res if r["min_clearance_m"] < 1.0])) if any(r["min_clearance_m"] < 1.0 for r in s_res) else 0.0
        mean_lat = float(np.mean([r["avg_latency_ms"] for r in s_res]))

        suites_summary[s_name] = {
            "episodes": n_eps,
            "success_rate_pct": round(sr, 1),
            "collision_rate_pct": round(cr, 1),
            "spl_pct": round(mean_spl, 1),
            "mean_ttg_s": round(mean_ttg, 2),
            "mean_jerk_rad_s2": round(mean_jerk, 2),
            "mean_clearance_m": round(mean_clr, 3),
            "mean_latency_ms": round(mean_lat, 3)
        }

    # Overall Summary
    total_eps = len(results)
    total_succ = sum(1 for r in results if r["success"])
    total_coll = sum(1 for r in results if r["collision"])
    overall_sr = (total_succ / total_eps) * 100.0
    overall_cr = (total_coll / total_eps) * 100.0
    overall_spl = float(np.mean([r["spl_pct"] for r in results]))
    overall_ttg = float(np.mean([r["time_to_goal_s"] for r in results if r["success"]])) if total_succ > 0 else 0.0
    overall_jerk = float(np.mean([r["angular_jerk"] for r in results]))
    overall_lat = float(np.mean([r["avg_latency_ms"] for r in results]))

    overall_summary = {
        "episodes": total_eps,
        "success_rate_pct": round(overall_sr, 1),
        "collision_rate_pct": round(overall_cr, 1),
        "spl_pct": round(overall_spl, 1),
        "mean_ttg_s": round(overall_ttg, 2),
        "mean_jerk_rad_s2": round(overall_jerk, 2),
        "mean_latency_ms": round(overall_lat, 3),
        "total_eval_time_s": round(t_elapsed, 2)
    }

    # Print Final Academic Evaluation Table
    print("\n" + "=" * 80)
    print("                    FORMAL BENCHMARK EVALUATION METRICS TABLE")
    print("=" * 80)
    print(f"{'Benchmark Suite':<30} | {'Eps':^5} | {'SR (%)':^8} | {'CR (%)':^8} | {'SPL (%)':^9} | {'TTG (s)':^8} | {'Jerk':^7}")
    print("-" * 80)
    for s_name, m in suites_summary.items():
        print(
            f"{s_name:<30} | {m['episodes']:^5d} | {m['success_rate_pct']:^8.1f} | "
            f"{m['collision_rate_pct']:^8.1f} | {m['spl_pct']:^9.1f} | {m['mean_ttg_s']:^8.2f} | {m['mean_jerk_rad_s2']:^7.2f}"
        )
    print("=" * 80)
    print(
        f"{'OVERALL AGGREGATE':<30} | {total_eps:^5d} | {overall_sr:^8.1f} | "
        f"{overall_cr:^8.1f} | {overall_spl:^9.1f} | {overall_ttg:^8.2f} | {overall_jerk:^7.2f}"
    )
    print("=" * 80)
    print(f"[SUMMARY] Mean Inference Latency : {overall_lat:.3f} ms (Target < 2.0 ms)")
    print(f"[SUMMARY] Total Evaluation Time  : {t_elapsed:.2f} s\n")

    # Save to JSON Report
    os.makedirs(os.path.dirname(report_path), exist_ok=True)
    report_data = {
        "model_path": model_path,
        "engine": runner.engine_type,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "overall": overall_summary,
        "suites": suites_summary,
        "episodes": results
    }
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(report_data, f, indent=2)
    print(f"[SUCCESS] Formal benchmark evaluation report saved to: {report_path}\n")

    return report_data


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate JetBot Imitation Learning Benchmark")
    parser.add_argument("--model", type=str, default="07-imitation-learning/models/bc_policy.onnx", help="Path to policy model")
    parser.add_argument("--report", type=str, default="07-imitation-learning/logs/evaluation_benchmark_report.json", help="Path to output report")
    args = parser.parse_args()

    run_full_benchmark(model_path=args.model, report_path=args.report)
