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


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    # 创建 10x10 网格地图（初始全是 1，不可通行）
    grid_map = GridMap(size=10)

    # ========== 清空地图，让所有网格可通行 ==========
    grid_map.grid_map[:, :] = 0   # 关键代码：全部设为 0（可通过）

    # 创建 A* 实例
    pathfinder = AStar(grid_map)

    # ========== 设置障碍物 ==========
    obstacles = [
        (2, 2), (2, 3), (2, 4),
        (3, 4),
        (5, 5), (6, 5), (7, 5)
    ]
    pathfinder.set_obstacle(obstacles)

    # 起点和终点
    start = (0, 0)
    goal = (9, 9)

    # 执行 A* 搜索
    path = pathfinder.find_path(start, goal)

    # 输出网格
    print("Grid Map (1 = obstacle, 0 = free):")
    print(grid_map.grid_map)

    # 输出路径
    if path:
        print("\n找到路径:")
        print(path)
    else:
        print("未找到可行路径")

    # ==============================
    # 可视化路径
    # ==============================
    if path:
        grid = grid_map.grid_map.copy()
        
        # 标记路径
        for (x, y) in path:
            grid[x, y] = 2  # 2 表示路径

        # 可视化
        plt.imshow(grid, cmap="gray_r")
        plt.title("A* Path (8-邻域)")

        plt.scatter(start[1], start[0], c="yellow", label="Start")
        plt.scatter(goal[1], goal[0], c="red", label="Goal")

        xs = [p[1] for p in path]
        ys = [p[0] for p in path]
        plt.plot(xs, ys, c="blue")

        plt.legend()
        plt.show()
