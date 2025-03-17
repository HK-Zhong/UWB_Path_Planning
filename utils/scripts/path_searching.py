import numpy as np
import heapq
from grid_map import GridMap

class AStar:
    """
    都是在grid_map（网格地图）中搜索路径
    """
    def __init__(self, grid_map: GridMap):
        """
        初始化网格地图
        :param grid_size: 网格大小
        """
        self.map = grid_map
        self.grid_size = self.map.grid_size
        self.grid_map = self.map.grid_map

    def set_obstacle(self, obstacles):
        """
        设置障碍物
        :param obstacles: [(x1, y1), (x2, y2), ...]  障碍物坐标列表
        """
        for x, y in obstacles:
            self.grid_map[x, y] = 1  # 设为障碍物

    def heuristic(self, a, b):
        """
        计算曼哈顿距离
        :param a: (x1, y1)
        :param b: (x2, y2)
        :return: 曼哈顿距离
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, node):
        """
        获取当前节点的邻居节点
        :param node: (x, y)
        :return: 邻居节点列表
        """
        x, y = node
        neighbors = [
            (x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)  # 只考虑上下左右四个方向
        ]
        valid_neighbors = [
            (nx, ny) for nx, ny in neighbors
            if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size and self.grid_map[nx, ny] == 0
        ]
        return valid_neighbors

    def find_path(self, start_grid, goal_grid):
        """
        使用 A* 搜索算法寻找路径
        :param start_grid: 起点 (x, y)
        :param goal_grid: 终点 (x, y)
        :return: 最优路径列表 [(x1, y1), (x2, y2), ...] 或 None（若无路径）
        """
        if self.grid_map[start_grid] == 1 or self.grid_map[goal_grid] == 1:
            return None  # 起点或终点是障碍物，直接返回

        open_set = []
        heapq.heappush(open_set, (0, start_grid))  # (优先级, 节点)
        came_from = {}  # 记录路径
        g_score = {start_grid: 0}
        f_score = {start_grid: self.heuristic(start_grid, goal_grid)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_grid:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1  # 由于是网格地图，每步的移动成本是 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # 无法到达目标点

    def reconstruct_path(self, came_from, current):
        """
        回溯路径
        :param came_from: 记录路径的字典
        :param current: 当前节点
        :return: 还原的路径
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def map_reconstruct(self, grid_map: GridMap):
        self.map = grid_map


# 测试 A* 搜索
if __name__ == "__main__":
    grid_map = GridMap(10)
    pathfinder = AStar(grid_map)
    pathfinder.set_obstacle([(2, 2), (2, 3), (2, 4), (3, 4)])  # 设定障碍物

    start = (0, 0)
    goal = (7, 7)

    path = pathfinder.find_path(start, goal)
    if path:
        print("找到路径:", path)
    else:
        print("无可行路径")
