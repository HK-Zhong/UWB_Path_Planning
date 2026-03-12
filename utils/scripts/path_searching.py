import heapq
import math
import numpy as np
from abc import ABC, abstractmethod
from grid_map import GridMap


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


# ============================================================
# EDT-aware A* Planner
# ============================================================
class EDTAwareAStarPlanner(PlannerBase):
    """
    Your current EDT-aware A* implementation.
    Behavior unchanged (except you can tune weights).
    """

    def __init__(self, grid_map: GridMap, edt_hard_min=0.5):
        super().__init__(grid_map)

        self.w_dist = 1.0
        self.w_safe = 5.0
        self.w_turn = 2.0
        self.edt_cost_power = 2.0

        self.edt_hard_min = edt_hard_min
        self.safe_epsilon = 1e-3

    # --------------------------------------------------------
    def heuristic(self, a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return dx + dy + (math.sqrt(2) - 2) * min(dx, dy)

    # --------------------------------------------------------
    def safety_cost(self, node):
        edt = float(self.map.edt_map[node[0], node[1]])
        return 1.0 / ((edt + self.safe_epsilon) ** self.edt_cost_power)

    # --------------------------------------------------------
    def get_neighbors(self, node):
        x, y = node
        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1),
        ]

        neighbors = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy

            if not (0 <= nx < self.map.grid_size and 0 <= ny < self.map.grid_size):
                continue

            # hard constraint: unknown/obstacle
            if self.map.grid_map[nx, ny] == 1:
                continue

            # hard constraint: too close to unknown/obstacle
            if self.map.edt_map[nx, ny] <= self.edt_hard_min:
                continue

            move_cost = 1.0 if dx == 0 or dy == 0 else math.sqrt(2)
            neighbors.append(((nx, ny), move_cost))

        return neighbors

    # --------------------------------------------------------
    def turn_cost(self, came_from, current, neighbor):
        parent = came_from.get(current, None)
        if parent is None:
            return 0.0

        v1x, v1y = current[0] - parent[0], current[1] - parent[1]
        v2x, v2y = neighbor[0] - current[0], neighbor[1] - current[1]

        if (v1x == 0 and v1y == 0) or (v2x == 0 and v2y == 0):
            return 0.0

        a1 = math.atan2(v1y, v1x)
        a2 = math.atan2(v2y, v2x)
        da = abs(a2 - a1)
        da = min(da, 2.0 * math.pi - da)

        x = da / math.pi
        return x * x

    # --------------------------------------------------------
    def plan(self, start_grid, goal_grid):
        """Weighted-scaling A* (soft multi-objective), optimized version.

        Design goal (unchanged):
          - Primary objective: shortest path (distance) dominates.
          - Secondary objective: prefer safer (larger EDT) BUT should not hijack distance.
          - Optional: mild turn smoothing.

        This version keeps the same external behavior, but uses:
          - 1D node indices instead of (x, y) tuples in the search loop
          - NumPy arrays instead of dict/set for g-score / closed-set / parent
          - precomputed EDT safety penalty
          - precomputed heuristic-to-goal map
        """

        grid_map = self.map.grid_map
        edt_map = self.map.edt_map
        grid_size = int(self.map.grid_size)
        edt_hard_min = float(self.edt_hard_min)
        safe_epsilon = float(self.safe_epsilon)
        edt_cost_power = float(self.edt_cost_power)
        w_dist = float(self.w_dist)
        w_safe = float(self.w_safe)
        w_turn = float(self.w_turn)
        res = float(self.map.resolution)
        sqrt2 = math.sqrt(2.0)
        pi = math.pi

        sx, sy = start_grid
        gx, gy = goal_grid

        if grid_map[sx, sy] == 1 or grid_map[gx, gy] == 1:
            return None

        # hard EDT feasibility check for start / goal (与原 get_neighbors 约束保持一致)
        if float(edt_map[sx, sy]) <= edt_hard_min or float(edt_map[gx, gy]) <= edt_hard_min:
            return None

        # ---------- 预计算 ----------
        # 安全惩罚图（只算一次）
        penalty_safe_map = ((edt_hard_min + safe_epsilon) / (edt_map.astype(np.float64) + safe_epsilon)) ** edt_cost_power
        penalty_safe_map = np.clip(penalty_safe_map, 0.0, 1.0)

        # 到 goal 的 octile heuristic 图（只算一次）
        xs = np.arange(grid_size, dtype=np.int32)
        ys = np.arange(grid_size, dtype=np.int32)
        dx_map = np.abs(xs[:, None] - gx)
        dy_map = np.abs(ys[None, :] - gy)
        h_map = w_dist * res * (dx_map + dy_map + (sqrt2 - 2.0) * np.minimum(dx_map, dy_map))

        total_nodes = grid_size * grid_size
        inf = float("inf")

        # ---------- 搜索状态（1D 索引） ----------
        g_score = np.full(total_nodes, inf, dtype=np.float64)
        parent_idx = np.full(total_nodes, -1, dtype=np.int32)
        closed = np.zeros(total_nodes, dtype=np.bool_)

        def to_idx(x, y):
            return x * grid_size + y

        start_idx = to_idx(sx, sy)
        goal_idx = to_idx(gx, gy)

        g_score[start_idx] = 0.0

        heappush = heapq.heappush
        heappop = heapq.heappop

        open_set = []
        heappush(open_set, (0.0, start_idx))

        directions = (
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, sqrt2), (-1, 1, sqrt2), (1, -1, sqrt2), (1, 1, sqrt2),
        )

        while open_set:
            _, cur_idx = heappop(open_set)

            if closed[cur_idx]:
                continue
            closed[cur_idx] = True

            if cur_idx == goal_idx:
                # 直接基于 parent_idx 重建 path，避免再构造 came_from 字典
                path = []
                idx = cur_idx
                while idx != -1:
                    x = idx // grid_size
                    y = idx % grid_size
                    path.append((x, y))
                    idx = int(parent_idx[idx])
                return path[::-1]

            cx = cur_idx // grid_size
            cy = cur_idx % grid_size
            g_current = g_score[cur_idx]

            p_idx = int(parent_idx[cur_idx])
            if p_idx != -1:
                px = p_idx // grid_size
                py = p_idx % grid_size
                v1x = cx - px
                v1y = cy - py
                has_parent_dir = not (v1x == 0 and v1y == 0)
                if has_parent_dir:
                    a1 = math.atan2(v1y, v1x)
                else:
                    a1 = 0.0
            else:
                has_parent_dir = False
                a1 = 0.0

            for dx, dy, move_cost in directions:
                nx = cx + dx
                ny = cy + dy

                if nx < 0 or nx >= grid_size or ny < 0 or ny >= grid_size:
                    continue

                if grid_map[nx, ny] == 1:
                    continue

                edt = float(edt_map[nx, ny])
                if edt <= edt_hard_min:
                    continue

                n_idx = nx * grid_size + ny
                if closed[n_idx]:
                    continue

                # 1) distance cost
                step_m = move_cost * res
                cost_dist = w_dist * step_m

                # 2) safety cost
                cost_safe = w_safe * float(penalty_safe_map[nx, ny]) * step_m

                # 3) turn cost
                if has_parent_dir:
                    a2 = math.atan2(dy, dx)
                    da = abs(a2 - a1)
                    if da > pi:
                        da = 2.0 * pi - da
                    x = da / pi
                    cost_turn = w_turn * (x * x)
                else:
                    cost_turn = 0.0

                tentative_g = g_current + cost_dist + cost_safe + cost_turn
                if tentative_g < g_score[n_idx]:
                    g_score[n_idx] = tentative_g
                    parent_idx[n_idx] = cur_idx
                    f_score = tentative_g + float(h_map[nx, ny])
                    heappush(open_set, (f_score, n_idx))

        return None

    # --------------------------------------------------------
    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]


# ============================================================
# Classic planners for comparison
#   1) Grid A* (distance-only, no EDT / no turn penalty)
#   2) Dijkstra (A* with zero heuristic)
# ============================================================

class GridAStarPlanner(PlannerBase):
    """Classic grid A* (distance-only).

    Hard constraints:
      - grid_map == 1 is forbidden (unknown/obstacle)

    Cost:
      - 4/8-neighborhood move cost (1 or sqrt(2))
      - heuristic: octile distance (admissible for 8-neighborhood)

    This is a baseline to compare against EDT-aware A*.
    """

    def __init__(self, grid_map: GridMap, allow_diagonal: bool = True):
        super().__init__(grid_map)
        self.allow_diagonal = allow_diagonal

    def heuristic(self, a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        # Octile distance (admissible for 8-connected grid)
        return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

    def get_neighbors(self, node):
        x, y = node
        if self.allow_diagonal:
            directions = [
                (-1, 0), (1, 0), (0, -1), (0, 1),
                (-1, -1), (-1, 1), (1, -1), (1, 1),
            ]
        else:
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        neighbors = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < self.map.grid_size and 0 <= ny < self.map.grid_size):
                continue
            if self.map.grid_map[nx, ny] == 1:
                continue
            move_cost = 1.0 if dx == 0 or dy == 0 else math.sqrt(2)
            neighbors.append(((nx, ny), move_cost))
        return neighbors

    def plan(self, start_grid, goal_grid):
        if self.map.grid_map[start_grid] == 1 or self.map.grid_map[goal_grid] == 1:
            return None

        open_set = []
        heapq.heappush(open_set, (0.0, start_grid))

        came_from = {}
        g_score = {start_grid: 0.0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_grid:
                return self.reconstruct_path(came_from, current)

            for neighbor, move_cost in self.get_neighbors(current):
                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score, neighbor))

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]


class DijkstraPlanner(PlannerBase):
    """Classic Dijkstra on grid (equivalent to A* with heuristic = 0).

    Hard constraints:
      - grid_map == 1 is forbidden (unknown/obstacle)

    Cost:
      - 4/8-neighborhood move cost (1 or sqrt(2))

    This is a baseline to compare against A* variants.
    """

    def __init__(self, grid_map: GridMap, allow_diagonal: bool = True):
        super().__init__(grid_map)
        self.allow_diagonal = allow_diagonal

    def get_neighbors(self, node):
        x, y = node
        if self.allow_diagonal:
            directions = [
                (-1, 0), (1, 0), (0, -1), (0, 1),
                (-1, -1), (-1, 1), (1, -1), (1, 1),
            ]
        else:
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        neighbors = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < self.map.grid_size and 0 <= ny < self.map.grid_size):
                continue
            if self.map.grid_map[nx, ny] == 1:
                continue
            move_cost = 1.0 if dx == 0 or dy == 0 else math.sqrt(2)
            neighbors.append(((nx, ny), move_cost))
        return neighbors

    def plan(self, start_grid, goal_grid):
        if self.map.grid_map[start_grid] == 1 or self.map.grid_map[goal_grid] == 1:
            return None

        # Priority by g only
        open_set = []
        heapq.heappush(open_set, (0.0, start_grid))

        came_from = {}
        g_score = {start_grid: 0.0}

        while open_set:
            g_cur, current = heapq.heappop(open_set)

            # Skip stale entries
            if g_cur > g_score.get(current, float('inf')):
                continue

            if current == goal_grid:
                return self.reconstruct_path(came_from, current)

            for neighbor, move_cost in self.get_neighbors(current):
                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    heapq.heappush(open_set, (tentative_g, neighbor))

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]
