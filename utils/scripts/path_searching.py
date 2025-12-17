import heapq
import math
import numpy as np
from grid_map import GridMap


class AStar:
    """
    EDT-aware A* path planner

    Hard constraints:
      - grid_map == 1  (unknown / obstacle forbidden)
      - edt < edt_hard_min forbidden

    Soft constraints:
      - prefer large EDT (stay in corridor center)
      - shortest path
    """

    def __init__(self, grid_map: GridMap, edt_hard_min=0.5):
        
        self.map = grid_map

        self.w_dist = 1.0
        self.w_safe = 10.0

        self.edt_hard_min = edt_hard_min

        self.safe_epsilon = 1e-3


    # =====================================================
    # Heuristic（几何距离，保持 admissible）
    # =====================================================
    def heuristic(self, a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return dx + dy + (math.sqrt(2) - 2) * min(dx, dy)

    # =====================================================
    # EDT-based safety cost（核心）
    # =====================================================
    def safety_cost(self, node):
        """
        Simple EDT-based safety cost:
        EDT 越大，代价越小
        """
        edt_value = self.map.edt_map[node[0], node[1]]

        return 1.0 / (edt_value + self.safe_epsilon)
        # return -edt_value

    # =====================================================
    # 8 邻域扩展
    # =====================================================
    def get_neighbors(self, node):
        x, y = node
        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)
        ]

        neighbors = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy

            if not (0 <= nx < self.map.grid_size and 0 <= ny < self.map.grid_size):
                continue

            # ❗硬约束 1：未知 / 障碍
            if self.map.grid_map[nx, ny] == 1:
                continue

            # ❗硬约束 2：EDT 太小（贴未知区域太近）
            if self.map.edt_map[nx, ny] <= self.edt_hard_min:
                continue

            move_cost = 1.0 if dx == 0 or dy == 0 else math.sqrt(2)
            neighbors.append(((nx, ny), move_cost))

        return neighbors

    # =====================================================
    # 主搜索
    # =====================================================
    def find_path(self, start_grid, goal_grid):

        if (
            self.map.grid_map[start_grid] == 1
            or self.map.grid_map[goal_grid] == 1
        ):
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

                cost_dist = self.w_dist * move_cost
                cost_safe = self.w_safe * self.safety_cost(neighbor)

                tentative_g = g_score[current] + cost_dist + cost_safe

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score, neighbor))

        return None

    # =====================================================
    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    # =====================================================
    def map_reconstruct(self, grid_map: GridMap):
        self.map = grid_map

    # =====================================================
    # 关键航点提取（防抖 + 平滑）
    # =====================================================
    def extract_key_waypoints(self, path):
        """
        输入：
            path: [(x, y), ...]
        输出：
            关键航点（栅格坐标，不含起点）
        """

        if not path or len(path) < 2:
            return []

        min_dist_m = max(1.5, 4 * self.map.resolution)

        waypoints = []
        last_kept = path[0]

        prev_dx = path[1][0] - path[0][0]
        prev_dy = path[1][1] - path[0][1]

        for i in range(1, len(path)):
            x_prev, y_prev = path[i - 1]
            x_cur, y_cur = path[i]

            dx = x_cur - x_prev
            dy = y_cur - y_prev

            dist = math.hypot(
                (x_cur - last_kept[0]) * self.map.resolution,
                (y_cur - last_kept[1]) * self.map.resolution
            )

            turn = (dx != prev_dx) or (dy != prev_dy)
            far_enough = dist >= min_dist_m

            if turn or far_enough:
                waypoints.append((x_cur, y_cur))
                last_kept = (x_cur, y_cur)

            prev_dx, prev_dy = dx, dy

        if waypoints and waypoints[-1] != path[-1]:
            waypoints.append(path[-1])
        elif not waypoints:
            waypoints.append(path[-1])

        # ❗删除过近的第一个航点（防顿挫）
        # if waypoints:
        #     first = waypoints[0]
        #     dist0 = math.hypot(
        #         (first[0] - path[0][0]) * self.map.resolution,
        #         (first[1] - path[0][1]) * self.map.resolution
        #     )
        #     if dist0 < 0.5 * min_dist_m:
        #         waypoints.pop(0)

        return waypoints
