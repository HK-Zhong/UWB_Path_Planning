import numpy as np
import heapq
import math
from grid_map import GridMap

class AStar:
    """
    A* 搜索（支持 8 邻域）
    """
    def __init__(self, grid_map: GridMap):
        self.map = grid_map

    def set_obstacle(self, obstacles):
        for x, y in obstacles:
            self.map.grid_map[x, y] = 1

    def heuristic(self, a, b):
        """
        对角距离（更适合 8 邻域 A*）
        """
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return dx + dy + (math.sqrt(2) - 2) * min(dx, dy)

    def get_neighbors(self, node):
        """
        获取 8 邻域节点
        """
        x, y = node
        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),      # 上下左右
            (-1, -1), (-1, 1), (1, -1), (1, 1)     # 四个对角
        ]

        neighbors = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if (
                0 <= nx < self.map.grid_size and
                0 <= ny < self.map.grid_size and
                self.map.grid_map[nx, ny] == 0
            ):
                # 区分直走 vs 对角线
                move_cost = 1 if dx == 0 or dy == 0 else math.sqrt(2)
                neighbors.append(((nx, ny), move_cost))

        return neighbors

    def find_path(self, start_grid, goal_grid):
        if self.map.grid_map[start_grid] == 1 or self.map.grid_map[goal_grid] == 1:
            return None

        open_set = []
        heapq.heappush(open_set, (0, start_grid))

        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self.heuristic(start_grid, goal_grid)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_grid:
                return self.reconstruct_path(came_from, current)

            for (neighbor, move_cost) in self.get_neighbors(current):
                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def map_reconstruct(self, grid_map: GridMap):
        self.map = grid_map
        
    def extract_key_waypoints(self, path):
        if not path or len(path) <= 2:
            return path  # 这种情况说明本来就没有移动

        waypoints = [path[0]]

        prev_dx = path[1][0] - path[0][0]
        prev_dy = path[1][1] - path[0][1]

        for i in range(2, len(path)):
            x_prev, y_prev = path[i-1]
            x_curr, y_curr = path[i]

            dx = x_curr - x_prev
            dy = y_curr - y_prev

            if dx != prev_dx or dy != prev_dy:
                waypoints.append((x_prev, y_prev))

            prev_dx, prev_dy = dx, dy

        waypoints.append(path[-1])

        # 🚀 删除起点，让无人机直接飞向下一个点
        if len(waypoints) > 1:
            waypoints = waypoints[1:]  # 关键：跳过起点

        return waypoints



if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import random

    # ========== 创建 30x30 网格地图 ==========
    grid_map = GridMap(size=16)
    grid_map.grid_map[:, :] = 0   # 全部可通行

    # 创建 A* 实例
    pathfinder = AStar(grid_map)

    # ========== 设置大面积障碍物 ==========
    obstacles = []

    # 1. 中间一条粗横障碍带
    for y in range(5, 25):
        obstacles.append((10, y))
        obstacles.append((11, y))
        obstacles.append((12, y))

    # 2. 大的矩形障碍块
    for x in range(18, 25):
        for y in range(3, 10):
            obstacles.append((x, y))

    # 3. 随机撒一些障碍点（更自然）
    random.seed(0)
    for _ in range(60):
        ox = random.randint(0, 29)
        oy = random.randint(0, 29)
        obstacles.append((ox, oy))

    # 添加障碍到地图
    pathfinder.set_obstacle(obstacles)

    # ========== 设置起点和终点 ==========
    start = (0, 0)
    goal = (29, 29)

    # 执行 A* 搜索
    path = pathfinder.find_path(start, goal)

    print("Grid Map (1 = obstacle, 0 = free):")
    print(grid_map.grid_map)

    if path:
        print("\nA* 完整路径:")
        print(path)
    else:
        print("未找到可行路径")
        exit()

    # ========== 提取关键航点 ==========
    def extract_key_waypoints(path):
        if not path or len(path) <= 2:
            return path

        waypoints = [path[0]]

        prev_dx = path[1][0] - path[0][0]
        prev_dy = path[1][1] - path[0][1]

        for i in range(2, len(path)):
            x_prev, y_prev = path[i-1]
            x_curr, y_curr = path[i]

            dx = x_curr - x_prev
            dy = y_curr - y_prev

            if dx != prev_dx or dy != prev_dy:
                waypoints.append((x_prev, y_prev))

            prev_dx, prev_dy = dx, dy

        waypoints.append(path[-1])
        return waypoints

    key_wps = extract_key_waypoints(path)

    print("\n关键航点:")
    print(key_wps)

    # =========================
    #      可视化路径
    # =========================
    grid = grid_map.grid_map.copy()

    # 标记 A* 路径
    for (x, y) in path:
        grid[x, y] = 2

    # 标记关键航点
    for (x, y) in key_wps:
        grid[x, y] = 3

    plt.figure(figsize=(8, 8))
    plt.imshow(grid, cmap="gray_r")
    plt.title("A* Path & Key Waypoints (30×30 Grid)")

    # 绘制完整路径
    xs = [p[1] for p in path]
    ys = [p[0] for p in path]
    plt.plot(xs, ys, c="blue", linewidth=2, label="A* Path")

    # 绘制关键点
    xs_wp = [p[1] for p in key_wps]
    ys_wp = [p[0] for p in key_wps]
    plt.scatter(xs_wp, ys_wp, c="red", s=60, label="Key Waypoints")

    # 起点终点
    plt.scatter(start[1], start[0], c="yellow", s=80, label="Start")
    plt.scatter(goal[1], goal[0], c="green", s=80, label="Goal")

    plt.legend()
    plt.show()
