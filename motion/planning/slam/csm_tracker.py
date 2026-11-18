"""Wide-Window Correlative Scan Matcher (CSM) Tracker (Olson 2009, Hess 2016)."""
import math
import os
import sys
from typing import Tuple, Optional, List
import numpy as np

try:
    from cartographer_core.submap_2d import ActiveSubmaps2D
except ImportError:
    _cart_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "cartographer"))
    if _cart_path not in sys.path:
        sys.path.insert(0, _cart_path)
    from cartographer_core.submap_2d import ActiveSubmaps2D


def wrap_angle_rad(th):
    """Wrap angle or array of angles to [-pi, pi)."""
    return (th + np.pi) % (2.0 * np.pi) - np.pi


class CSMTracker:
    """
    Correlative Scan Matcher tracking heading and position continuously across turns.
    Coarse search: +-45 deg (1.5 deg step), +-10 cm (5 cm step).
    Fine search: +-1.5 deg (0.25 deg step), +-2 cm (1 cm step).
    """

    def __init__(
        self,
        resolution: float = 0.05,
        grid_dim: int = 160,
        submap_num_range_data: int = 35,
        lever_arm_m: float = 0.01,
        step: int = 2,
        w_trans: float = 800.0,
        w_rot: float = 280.0,
        max_dth_step_rad: float = 0.65
    ):
        self.resolution = float(resolution)
        self.grid_dim = int(grid_dim)
        self.submap_num_range_data = int(submap_num_range_data)
        self.lever_arm_m = float(lever_arm_m)
        self.step = int(step)
        self.w_trans = float(w_trans)
        self.w_rot = float(w_rot)
        self.max_dth_step_rad = float(max_dth_step_rad)

        self.submaps_mgr = ActiveSubmaps2D(
            num_range_data=self.submap_num_range_data,
            resolution=resolution,
            grid_width=grid_dim,
            grid_height=grid_dim
        )

        self.curr_pose = (0.0, 0.0, 0.0)
        self.prev_pose = (0.0, 0.0, 0.0)
        self.prev_ranges: Optional[np.ndarray] = None
        self.last_insert_pose = (0.0, 0.0, 0.0)
        self.still_counter = 0
        self.clean_trans_count = 0

        self.offset_x: float = 0.0
        self.offset_y: float = 0.0
        self.offset_th: float = 0.0

        self.angles_rad = np.radians(np.arange(360, dtype=np.float32))

        # Precompute coarse search offsets
        self.d_th_coarse = np.radians(np.arange(-45.0, 45.1, 1.5, dtype=np.float32))
        self.rot_pen_coarse = self.w_rot * (self.d_th_coarse ** 2)

        # Precompute fine search offsets
        self.d_th_fine = np.radians(np.arange(-1.5, 1.51, 0.25, dtype=np.float32))

    def internal_to_published(self, x_int: float, y_int: float, th_int: float) -> Tuple[float, float, float]:
        """Apply rigid offset (rotation + translation) to convert internal pose to published pose."""
        th_pub = float(wrap_angle_rad(self.offset_th + float(th_int)))
        c = math.cos(self.offset_th)
        s = math.sin(self.offset_th)
        x_pub = float(self.offset_x + (c * float(x_int) - s * float(y_int)))
        y_pub = float(self.offset_y + (s * float(x_int) + c * float(y_int)))
        return x_pub, y_pub, th_pub

    def set_pose(self, x: float, y: float, theta: float):
        """
        Sets published pose without resetting existing submaps or internal tracking.
        Stores a rigid offset (rotation + translation) between internal and published frame.
        """
        x_int, y_int, th_int = self.curr_pose
        self.offset_th = float(wrap_angle_rad(float(theta) - float(th_int)))
        c = math.cos(self.offset_th)
        s = math.sin(self.offset_th)
        self.offset_x = float(float(x) - (c * float(x_int) - s * float(y_int)))
        self.offset_y = float(float(y) - (s * float(x_int) + c * float(y_int)))

    def reset_pose(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Reset tracker pose and clear history."""
        self.curr_pose = (0.0, 0.0, 0.0)
        self.prev_pose = (0.0, 0.0, 0.0)
        self.prev_ranges = None
        self.last_insert_pose = (0.0, 0.0, 0.0)
        self.still_counter = 0
        self.clean_trans_count = 0
        self.offset_x = float(x)
        self.offset_y = float(y)
        self.offset_th = float(theta)
        self.submaps_mgr = ActiveSubmaps2D(
            num_range_data=self.submap_num_range_data,
            resolution=self.resolution,
            grid_width=self.grid_dim,
            grid_height=self.grid_dim
        )

    def detect_stillness(self, curr_ranges: np.ndarray) -> bool:
        """Detect whether the robot is stationary based on LiDAR sweep differences."""
        if self.prev_ranges is None:
            return True
        valid = (curr_ranges >= 0.05) & (curr_ranges <= 6.0)
        m = (self.prev_ranges > 0.05) & (self.prev_ranges < 6.0) & valid
        if np.count_nonzero(m) >= 30:
            diff = np.abs(curr_ranges[m] - self.prev_ranges[m])
            return (float(np.median(diff)) < 0.04) and (float(np.percentile(diff, 90)) < 0.08)
        return False

    def update(
        self,
        ranges: List[float]
    ) -> Tuple[float, float, float, bool, bool]:
        """
        Process a new scan through Correlative Scan Matching.
        Returns: (x, y, theta, is_still, can_write_map)
        """
        curr_ranges = np.asarray(ranges, dtype=np.float32)
        valid = (curr_ranges >= 0.05) & (curr_ranges <= 6.0)
        is_still = self.detect_stillness(curr_ranges)
        self.prev_ranges = curr_ranges

        # Transform raw ranges to body frame (with lever arm)
        lx = curr_ranges[valid] * np.cos(self.angles_rad[valid]) + self.lever_arm_m
        ly = curr_ranges[valid] * np.sin(self.angles_rad[valid])
        local_pts = np.column_stack([lx, ly])

        # First scan initialization
        if not self.submaps_mgr.submaps:
            self.submaps_mgr.insert_range_data(self.curr_pose, local_pts)
            self.last_insert_pose = self.curr_pose
            pub_x, pub_y, pub_th = self.internal_to_published(
                self.curr_pose[0], self.curr_pose[1], self.curr_pose[2]
            )
            return pub_x, pub_y, pub_th, is_still, True

        grid = self.submaps_mgr.submaps[0].grid
        raw_p = grid.odds / (1.0 + grid.odds)
        prob = np.where(grid.observed & (raw_p > 0.5), raw_p, 0.0).astype(np.float32)

        H, W = grid.height, grid.width
        pad_p = np.pad(prob, 2, mode="constant", constant_values=0.0)
        ox, oy = grid.origin_x, grid.origin_y
        inv_res = grid.inv_resolution

        pts_match = local_pts[::self.step] if len(local_pts) > 60 else local_pts

        # Constant velocity motion prior from previous estimates (never w_cmd / v_cmd)
        dth_prev = wrap_angle_rad(self.curr_pose[2] - self.prev_pose[2])
        dth_prev = max(-self.max_dth_step_rad, min(self.max_dth_step_rad, dth_prev))

        x0 = self.curr_pose[0]
        y0 = self.curr_pose[1]
        th0 = wrap_angle_rad(self.curr_pose[2] + dth_prev)

        # 1. Coarse search (+-45 deg in 1.5 deg steps, +-10 cm in 5 cm steps)
        cand_th_c = (th0 + self.d_th_coarse + np.pi) % (2.0 * np.pi) - np.pi
        c_c, s_c = np.cos(cand_th_c), np.sin(cand_th_c)

        rot_x = pts_match[:, 0][np.newaxis, :] * c_c[:, np.newaxis] - pts_match[:, 1][np.newaxis, :] * s_c[:, np.newaxis]
        rot_y = pts_match[:, 0][np.newaxis, :] * s_c[:, np.newaxis] + pts_match[:, 1][np.newaxis, :] * c_c[:, np.newaxis]

        col_0 = np.floor((rot_x + (x0 - ox)) * inv_res).astype(np.int32)
        row_0 = np.floor((rot_y + (y0 - oy)) * inv_res).astype(np.int32)

        best_score = -1e9
        best_th_idx = 0
        best_dx = 0.0
        best_dy = 0.0

        for sx in (-2, -1, 0, 1, 2):
            for sy in (-2, -1, 0, 1, 2):
                c_idx = np.clip(col_0 + (2 + sx), 0, W + 3)
                r_idx = np.clip(row_0 + (2 + sy), 0, H + 3)
                pen = self.w_trans * ((sx * 0.05) ** 2 + (sy * 0.05) ** 2) + self.rot_pen_coarse
                sc = np.sum(pad_p[r_idx, c_idx], axis=1) - pen
                m_i = int(np.argmax(sc))
                if sc[m_i] > best_score:
                    best_score = float(sc[m_i])
                    best_th_idx = m_i
                    best_dx = sx * 0.05
                    best_dy = sy * 0.05

        best_coarse_th = cand_th_c[best_th_idx]
        best_coarse_x = x0 + best_dx
        best_coarse_y = y0 + best_dy

        # 2. Fine search (+-1.5 deg in 0.25 deg steps, +-2 cm in 1 cm steps)
        cand_th_f = (best_coarse_th + self.d_th_fine + np.pi) % (2.0 * np.pi) - np.pi
        c_f, s_f = np.cos(cand_th_f), np.sin(cand_th_f)

        rot_xf = pts_match[:, 0][np.newaxis, :] * c_f[:, np.newaxis] - pts_match[:, 1][np.newaxis, :] * s_f[:, np.newaxis]
        rot_yf = pts_match[:, 0][np.newaxis, :] * s_f[:, np.newaxis] + pts_match[:, 1][np.newaxis, :] * c_f[:, np.newaxis]

        best_fine_score = -1e9
        final_x, final_y, final_th = best_coarse_x, best_coarse_y, best_coarse_th

        for fdx in (-0.02, -0.01, 0.0, 0.01, 0.02):
            for fdy in (-0.02, -0.01, 0.0, 0.01, 0.02):
                c_f_idx = np.floor((rot_xf + (best_coarse_x + fdx - ox)) * inv_res).astype(np.int32)
                r_f_idx = np.floor((rot_yf + (best_coarse_y + fdy - oy)) * inv_res).astype(np.int32)
                c_idx = np.clip(c_f_idx + 2, 0, W + 3)
                r_idx = np.clip(r_f_idx + 2, 0, H + 3)
                diff_th = wrap_angle_rad(cand_th_f - th0)
                pen = self.w_trans * ((fdx) ** 2 + (fdy) ** 2) + self.w_rot * (diff_th ** 2)
                sc = np.sum(pad_p[r_idx, c_idx], axis=1) - pen
                m_i = int(np.argmax(sc))
                if sc[m_i] > best_fine_score:
                    best_fine_score = float(sc[m_i])
                    final_th = cand_th_f[m_i]
                    final_x = best_coarse_x + fdx
                    final_y = best_coarse_y + fdy

        final_x = float(final_x)
        final_y = float(final_y)
        final_th = float(final_th)
        self.prev_pose = self.curr_pose
        self.curr_pose = (final_x, final_y, final_th)

        # 3. Submap insertion & map gating (LiDAR-only, no command reliance)
        step_p = math.hypot(final_x - self.prev_pose[0], final_y - self.prev_pose[1])
        step_rot = abs(wrap_angle_rad(final_th - self.prev_pose[2]))
        d_p = math.hypot(final_x - self.last_insert_pose[0], final_y - self.last_insert_pose[1])
        d_th_insert = abs(wrap_angle_rad(final_th - self.last_insert_pose[2]))

        # Clean translation from LiDAR kinematics: not still, step displacement >= 1.0 cm,
        # rotation rate <= 14 deg/step (<= 2.4 rad/s arc), translation/rotation ratio >= 5 cm/rad,
        # and sustained for at least 2 consecutive sweeps to filter initial pivot acceleration transients.
        is_step_trans = (
            (not is_still) and
            (step_rot <= math.radians(14.0)) and
            (step_p >= 0.010) and
            (step_p >= step_rot * 0.05)
        )
        if is_step_trans:
            self.clean_trans_count += 1
        else:
            self.clean_trans_count = 0

        can_write_map = is_still or (self.clean_trans_count >= 2)

        # Submaps only populated with stationary sweeps (skip unchanged view)
        should_insert = False
        if is_still:
            self.still_counter += 1
            if d_p > 0.015 or d_th_insert > math.radians(1.0) or self.still_counter >= 5:
                should_insert = True
                self.still_counter = 0

        if should_insert:
            c, s = math.cos(final_th), math.sin(final_th)
            wx = final_x + local_pts[:, 0] * c - local_pts[:, 1] * s
            wy = final_y + local_pts[:, 0] * s + local_pts[:, 1] * c
            self.submaps_mgr.insert_range_data(self.curr_pose, np.column_stack([wx, wy]))
            self.last_insert_pose = self.curr_pose

        pub_x, pub_y, pub_th = self.internal_to_published(final_x, final_y, final_th)
        return pub_x, pub_y, pub_th, is_still, can_write_map
