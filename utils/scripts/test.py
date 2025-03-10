import numpy as np
from grid_map import GridMap
from path_searching import AStar


grid_map = GridMap(10)
grid_map.load_uwb_anchors()
grid_map.map_init()

planner = AStar(grid_map)
start_real = (1, 1)  # 真实起点坐标
goal_real = (9, 9)  # 真实终点坐标

path = planner.a_star_search(start_real, goal_real)
print(path)
planner.visualize_path(path)
