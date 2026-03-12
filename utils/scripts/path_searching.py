import heapq
import math
import numpy as np
from abc import ABC, abstractmethod
from grid_map import GridMap
from PlannerBase import PlannerBase


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

        # cached safety penalty map (recomputed only when map or params change)
        self._cached_penalty_safe_map = None
        self._cache_key = None

        # 8-direction ids and precomputed turn-cost table
        self._dir_to_id = {
            (-1, 0): 0,
            (1, 0): 1,
            (0, -1): 2,
            (0, 1): 3,
            (-1, -1): 4,
            (-1, 1): 5,
            (1, -1): 6,
            (1, 1): 7,
        }
        self._turn_cost_table = self._build_turn_cost_table()

    def _build_turn_cost_table(self):
        """Precompute 8x8 turn-cost lookup table for discrete neighbor directions."""
        dirs = [None] * 8
        for k, v in self._dir_to_id.items():
            dirs[v] = k

        table = np.zeros((8, 8), dtype=np.float64)
        for i, (dx1, dy1) in enumerate(dirs):
            a1 = math.atan2(dy1, dx1)
            for j, (dx2, dy2) in enumerate(dirs):
                a2 = math.atan2(dy2, dx2)
                da = abs(a2 - a1)
                da = min(da, 2.0 * math.pi - da)
                x = da / math.pi
                table[i, j] = x * x
        return table

    def _get_penalty_safe_map(self):
        """Return cached safety penalty map; recompute only when needed."""
        edt_map = self.map.edt_map
        cache_key = (
            id(edt_map),
            edt_map.shape,
            float(self.edt_hard_min),
            float(self.safe_epsilon),
            float(self.edt_cost_power),
        )

        if self._cached_penalty_safe_map is None or self._cache_key != cache_key:
            penalty_safe_map = ((self.edt_hard_min + self.safe_epsilon) /
                                (edt_map.astype(np.float64) + self.safe_epsilon)) ** self.edt_cost_power
            penalty_safe_map = np.clip(penalty_safe_map, 0.0, 1.0)
            self._cached_penalty_safe_map = penalty_safe_map
            self._cache_key = cache_key

        return self._cached_penalty_safe_map

    def map_reconstruct(self, grid_map: GridMap):
        super().map_reconstruct(grid_map)
        self._cached_penalty_safe_map = None
        self._cache_key = None

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
        h_diag_coeff = (sqrt2 - 2.0)
        dir_to_id = self._dir_to_id
        turn_cost_table = self._turn_cost_table

        sx, sy = start_grid
        gx, gy = goal_grid

        if grid_map[sx, sy] == 1 or grid_map[gx, gy] == 1:
            return None

        # hard EDT feasibility check for start / goal (与原 get_neighbors 约束保持一致)
        if float(edt_map[sx, sy]) <= edt_hard_min or float(edt_map[gx, gy]) <= edt_hard_min:
            return None

        # ---------- 预计算 ----------
        # 安全惩罚图（缓存复用）
        penalty_safe_map = self._get_penalty_safe_map()


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
                prev_dir_id = dir_to_id.get((v1x, v1y), None)
            else:
                prev_dir_id = None

            for dx, dy, move_cost in directions:
                nx = cx + dx
                ny = cy + dy

                if nx < 0 or nx >= grid_size or ny < 0 or ny >= grid_size:
                    continue

                if grid_map[nx, ny] == 1:
                    continue

                edt = edt_map[nx, ny]
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

                # 3) turn cost (lookup table)
                if prev_dir_id is not None:
                    next_dir_id = dir_to_id[(dx, dy)]
                    cost_turn = w_turn * float(turn_cost_table[prev_dir_id, next_dir_id])
                else:
                    cost_turn = 0.0

                tentative_g = g_current + cost_dist + cost_safe + cost_turn
                if tentative_g < g_score[n_idx]:
                    g_score[n_idx] = tentative_g
                    parent_idx[n_idx] = cur_idx
                    hx = abs(nx - gx)
                    hy = abs(ny - gy)
                    h = w_dist * res * (hx + hy + h_diag_coeff * min(hx, hy))
                    f_score = tentative_g + h
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
