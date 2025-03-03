import numpy as np
from path_planning import DStarLite

class GridMap:
    def __init__(self, size):
        self.grid_size = size
        self.grid = np.zeros((size, size), dtype=int)  # 0 表示可通行，1 表示障碍物
    
    def add_obstacle(self, obstacles):
        for x, y in obstacles:
            self.grid[x, y] = 1

    def remove_obstacle(self, obstacles):
        for x, y in obstacles:
            self.grid[x, y] = 0

# 创建地图
size = 10
map_instance = GridMap(size)
map_instance.add_obstacle([(4, 0), (4, 5), (4, 6), (4, 7), (5, 7)])  # 设置障碍物

# 初始化 D* Lite
start = (0, 0)
goal = (9, 9)
dstar = DStarLite(map_instance)
dstar.initialize(start, goal)

# 计算路径
print("初始路径:")
path = dstar.find_path()
print(path)

# 更新障碍物
print("添加新的障碍物后:")
map_instance.add_obstacle([(3, 3), (3, 4), (3, 5)])
dstar.update_obstacle([(3, 3), (3, 4), (3, 5)])
path = dstar.find_path()
print(path)

# 机器人移动
print("机器人移动后:")
dstar.move_start((1, 1))
path = dstar.find_path()
print(path)
