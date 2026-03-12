from abc import ABC, abstractmethod
from grid_map import GridMap
import math
import numpy as np


# ============================================================
# Planner Base Class
# ============================================================
class PlannerBase(ABC):
    """
    Abstract planner interface.

    All concrete planners must implement:
        - plan(start_grid, goal_grid)

    Shared utilities (window extraction, waypoint extraction)
    are implemented here and reused by all planners.
    """

    def __init__(self, grid_map: GridMap):
        self.map = grid_map

    # --------------------------------------------------------
    def map_reconstruct(self, grid_map: GridMap):
        """Inject updated map (used by navigator)."""
        self.map = grid_map

    # --------------------------------------------------------
    @abstractmethod
    def plan(self, start_grid, goal_grid):
        """Return grid path or None."""
        raise NotImplementedError

    # ========================================================
    # Window Control Point Extraction (shared)
    # ========================================================
    def extract_first_window_ctrl_points(self, path, min_dist_m):
        """
        方案 A（推荐）：弧长均匀 + 曲率引导 的第一段窗口控制点提取

        目标：
          - 先取 path 的第一段窗口（累计弧长达到 min_dist_m 的那一段）
          - 在窗口内选出 4 个控制点：起点、中点1、中点2、终点
          - 中点位置以“弧长均匀”为主，并在局部范围内向“转弯更明显(曲率更大)”的位置轻微偏置

        返回：
            [(gx0, gy0), (gx1, gy1), (gx2, gy2), (gx3, gy3)]
        """

        if not path or len(path) < 2:
            return None

        # =====================================================
        # 1) 收集窗口内的离散点（累计弧长 >= min_dist_m 即停止）
        # =====================================================
        window_pts = [path[0]]
        dist_acc = 0.0

        for i in range(1, len(path)):
            p_prev = path[i - 1]
            p_cur = path[i]

            step_dist = math.hypot(
                (p_cur[0] - p_prev[0]) * self.map.resolution,
                (p_cur[1] - p_prev[1]) * self.map.resolution,
            )

            dist_acc += step_dist
            window_pts.append(p_cur)

            if dist_acc >= min_dist_m:
                break

        n = len(window_pts)
        if n < 2:
            return None

        # =====================================================
        # 2) 若窗口点数不足 4：对窗口折线做“弧长插值”补足 4 点
        #    （保证：一定返回 4 个控制点）
        # =====================================================
        def _interp_along_polyline(pts, alpha):
            """沿折线按弧长比例 alpha ∈ [0,1] 插值，返回 (gx,gy) 整数"""
            if len(pts) == 1:
                return tuple(pts[0])

            # 累计弧长（单位：米）
            s_local = [0.0]
            for k in range(1, len(pts)):
                a = pts[k - 1]
                b = pts[k]
                ds = math.hypot(
                    (b[0] - a[0]) * self.map.resolution,
                    (b[1] - a[1]) * self.map.resolution,
                )
                s_local.append(s_local[-1] + ds)

            total_local = s_local[-1]
            if total_local <= 1e-9:
                return tuple(pts[0])

            target = alpha * total_local

            # 找到 target 所在的段
            idx = int(np.searchsorted(np.array(s_local), target, side="right") - 1)
            idx = max(0, min(idx, len(pts) - 2))

            s0, s1 = s_local[idx], s_local[idx + 1]
            p0 = np.array(pts[idx], dtype=float)
            p1 = np.array(pts[idx + 1], dtype=float)

            if s1 - s0 <= 1e-9:
                p = p0
            else:
                r = (target - s0) / (s1 - s0)
                p = (1.0 - r) * p0 + r * p1

            return (int(round(p[0])), int(round(p[1])))

        if n < 4:
            # 用弧长均匀的 4 点（0, 1/3, 2/3, 1）补齐
            p0 = tuple(window_pts[0])
            p1 = _interp_along_polyline(window_pts, 1.0 / 3.0)
            p2 = _interp_along_polyline(window_pts, 2.0 / 3.0)
            p3 = tuple(window_pts[-1])

            # 去重（极端情况下可能会插到同一个格子上）
            out = [p0, p1, p2, p3]
            out2 = [out[0]]
            for pt in out[1:]:
                if pt != out2[-1]:
                    out2.append(pt)
            while len(out2) < 4:
                out2.insert(-1, out2[-2])
            return out2[:4]

        # =====================================================
        # 3) 弧长均匀：先找到 1/3、2/3 弧长处的“基准索引”
        # =====================================================
        s = np.zeros(n, dtype=float)
        for i in range(1, n):
            a = window_pts[i - 1]
            b = window_pts[i]
            s[i] = s[i - 1] + math.hypot(
                (b[0] - a[0]) * self.map.resolution,
                (b[1] - a[1]) * self.map.resolution,
            )

        total = float(s[-1])
        if total <= 1e-9:
            p = tuple(window_pts[0])
            return [p, p, p, p]

        t1 = total / 3.0
        t2 = 2.0 * total / 3.0

        base1 = int(np.searchsorted(s, t1, side="left"))
        base2 = int(np.searchsorted(s, t2, side="left"))
        base1 = max(1, min(base1, n - 2))
        base2 = max(1, min(base2, n - 2))

        # =====================================================
        # 4) 曲率（转角）计算：只用于“轻微偏置”
        #    curv[i] = (Δθ/π)^2  (i 为内点)
        # =====================================================
        curv = np.zeros(n, dtype=float)
        for i in range(1, n - 1):
            p_prev = np.array(window_pts[i - 1], dtype=float)
            p_cur = np.array(window_pts[i], dtype=float)
            p_next = np.array(window_pts[i + 1], dtype=float)

            v1 = p_cur - p_prev
            v2 = p_next - p_cur

            n1 = np.linalg.norm(v1)
            n2 = np.linalg.norm(v2)
            if n1 < 1e-9 or n2 < 1e-9:
                curv[i] = 0.0
                continue

            a1 = math.atan2(v1[1], v1[0])
            a2 = math.atan2(v2[1], v2[0])
            da = abs(a2 - a1)
            da = min(da, 2.0 * math.pi - da)

            x = da / math.pi
            curv[i] = x * x

        # =====================================================
        # 5) 在 base 附近 ±K 搜索：
        #    主目标：弧长接近 target
        #    次目标：偏向曲率更大的点
        # =====================================================
        def _pick_index_near(base_idx, target_s, K=4):
            best_i = base_idx
            best_score = -1e18

            lo = max(1, base_idx - K)
            hi = min(n - 2, base_idx + K)

            for i in range(lo, hi + 1):
                ds = abs(float(s[i]) - float(target_s)) / total
                c = math.sqrt(float(curv[i]) + 1e-12)

                # 简单打分：弧长为主（惩罚），曲率为辅（奖励）
                score = (0.35 * c) - (1.0 * ds)

                if score > best_score:
                    best_score = score
                    best_i = i

            return best_i

        idx1 = _pick_index_near(base1, t1)
        idx2 = _pick_index_near(base2, t2)

        if idx2 <= idx1:
            idx2 = min(n - 2, idx1 + 1)

        return [
            tuple(window_pts[0]),
            tuple(window_pts[idx1]),
            tuple(window_pts[idx2]),
            tuple(window_pts[-1]),
        ]
