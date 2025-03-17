import yaml
import numpy as np
import matplotlib.pyplot as plt


class GridMap:
    def __init__(self, size=50, resolution=0.5):
        """
        初始化栅格地图
        :param size: 地图大小（米）
        :param resolution: 栅格分辨率（米）
        """
        self.size = size
        self.resolution = resolution
        self.grid_size = int(size / resolution)  # 计算栅格总数
        self.grid_map = np.ones((self.grid_size, self.grid_size), dtype=np.int8)  # 0 表示可通行，1 表示障碍物
        self.uwb_anchors_file = '/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/UWB_Anchors.yml'
        self.real_anchors_position = self.load_uwb_anchors()
        
        # UWB锚点编号之间的连通性
        self.uwb_los = [(0, 1), (1, 2), (2, 3), (1, 4), (2, 5), (3, 6), (3, 8), (0, 7), (7, 9), (9, 10), (9, 11), (11, 12), (12, 13), (11, 13), (8, 13)]
    
    def load_uwb_anchors(self):
        """
        从 UWB_Anchors.yml 文件加载锚点坐标,返回真实坐标
        """
        real_anchors = dict()
        with open(self.uwb_anchors_file, 'r') as file:
            data = yaml.safe_load(file)
        
        for anchor in data['UWB_Anchors']:
            real_anchors[anchor["id"]] = (anchor["x"], anchor["y"])
            
        return real_anchors
        
    def set_obstacle(self, obstacles):
        for x, y in obstacles:
            self.grid_map[x, y] = 1

    def remove_obstacle(self, obstacles):
        for x, y in obstacles:
            self.grid_map[x, y] = 0

    def to_grid(self, x_real, y_real):
        """ 将实际坐标转换为栅格坐标 """
        x_grid = int((x_real + self.size / 2) / self.resolution)
        y_grid = int((y_real + self.size / 2) / self.resolution)
        return x_grid, y_grid

    def to_real(self, x_grid, y_grid):
        """ 将栅格坐标转换为实际坐标 """
        x_real = round(x_grid * self.resolution - self.size / 2, 1)
        y_real = round(y_grid * self.resolution - self.size / 2, 1)
        return x_real, y_real

    def get_line_grids(self, start_real, end_real):
        """
        获取两点连线上所有的栅格坐标（使用 Bresenham 直线算法）
        :param start_real: 起点的真实坐标 (x, y)
        :param end_real: 终点的真实坐标 (x, y)
        :return: 经过的所有栅格点列表 [(gx1, gy1), (gx2, gy2), ...]
        """
        start_grid = self.to_grid(*start_real)
        end_grid = self.to_grid(*end_real)

        x0, y0 = start_grid
        x1, y1 = end_grid

        points = []

        # Bresenham 直线算法
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return points
    
    def map_update(self, start_real, end_real):
        """
        更新地图,输入的位置均为真实位置
        :param start_real: 起点的真实坐标 (x, y)
        :param end_real: 终点的真实坐标 (x, y)
        """
        points = self.get_line_grids(start_real, end_real)
        for point in points:
            self.grid_map[point] = 0
                
    def map_update_by_los(self, uav_position_real, los_data: dict):
        """
        los_data: {0: True, 1: True, 2: False, 3: False, 4: True, 5: False, 6: False, 7: True, 8: False, 9: False, 10: False, 11: False, 12: False, 13: False}
        """
        for id in los_data.keys():
            # 如果无人机与锚点之间是视距
            if los_data[id]:
                self.map_update(uav_position_real, self.real_anchors_position[id])
    
    def map_init(self):
        """
        初始化UWB锚点间的连通性
        """
        for path in self.uwb_los:
            start_real = self.real_anchors_position[path[0]]
            end_real = self.real_anchors_position[path[1]]
            self.map_update(start_real, end_real)
            
    def visualize(self):
        """
        可视化网格地图。
        """
        plt.figure(figsize=(6, 6))
        plt.imshow(self.grid_map, cmap='gray_r', origin='upper')

        # 添加网格线
        plt.xticks(np.arange(-0.5, self.grid_size, 1), [])
        plt.yticks(np.arange(-0.5, self.grid_size, 1), [])
        plt.grid(color='black', linestyle='-', linewidth=0.5)

        plt.show()


if __name__ == "__main__":
    grid_map = GridMap()
    grid_map.map_init()
    grid_map.visualize()