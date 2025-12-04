import yaml
import numpy as np
import matplotlib.pyplot as plt
import rospy
import os
import time

# ⭐ 新增：用于欧氏距离变换
from scipy.ndimage import distance_transform_edt


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
        # 1 表示障碍物/未知，0 表示可通行
        self.grid_map = np.ones((self.grid_size, self.grid_size), dtype=np.int8)

        self.uwb_anchors_file = '/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/UWB_Anchors.yml'
        self.real_anchors_position = self.load_uwb_anchors()
        
        # UWB锚点编号之间的连通性
        self.uwb_los = [
            (0, 1), (1, 2), (2, 3), (1, 4),
            (2, 5), (3, 6), (3, 8), (0, 7),
            (7, 9), (9, 10), (9, 11), (11, 12),
            (12, 13), (11, 13), (8, 13), (4, 5),
            (5, 6)
        ]
    
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
        x_grid = int((x_real + self.size / 2.0) / self.resolution)
        y_grid = int((y_real + self.size / 2.0) / self.resolution)
        return x_grid, y_grid

    def to_real(self, x_grid, y_grid):
        """ 将栅格坐标转换为实际坐标 """
        x_real = round(x_grid * self.resolution - self.size / 2.0, 1)
        y_real = round(y_grid * self.resolution - self.size / 2.0, 1)
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
    
    def map_update(self, start_real, end_real, free_expand=1):
        """
        更新地图，并将 LOS 的可通行区域扩宽 free_expand 格
        """
        points = self.get_line_grids(start_real, end_real)

        for (gx, gy) in points:
            # 把 LOS 线附近 free_expand 格内全部置为 0（视为可通行）
            for dx in range(-free_expand, free_expand + 1):
                for dy in range(-free_expand, free_expand + 1):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                        self.grid_map[nx, ny] = 0
                
    def map_update_by_los(self, uav_position_real, los_data):
        """
        los_data: [{'id': 0, 'LOS': True}, ...]
        """
        for tag in los_data:
            # 如果无人机与锚点之间是视距
            if tag["LOS"]:
                self.map_update(uav_position_real, self.real_anchors_position[tag["id"]])
    
    def map_init(self, free_expand=1):
        """
        初始化UWB锚点间的连通性
        """
        for path in self.uwb_los:
            start_real = self.real_anchors_position[path[0]]
            end_real = self.real_anchors_position[path[1]]
            self.map_update(start_real, end_real, free_expand=free_expand)

    # ========= ⭐ 新增：障碍物“膨胀”（通过距离场收缩可行区域） =========
    def inflate_obstacles(self, safe_distance_m=0.5):
        """
        使用欧氏距离变换，对障碍物进行“膨胀”，
        实际效果：离障碍物/未知 小于 safe_distance_m 的 free(0) 栅格全部变成 1

        safe_distance_m: 无人机期望与障碍物保持的最小安全距离（米）
        """
        # free_mask=True 的地方是可通行区域
        free_mask = (self.grid_map == 0)

        # 如果目前还没有任何 free 区域，直接返回
        if not free_mask.any():
            rospy.logwarn("inflate_obstacles: no free cells yet, skip inflation.")
            return

        # 计算：每个 free 格子到最近障碍(=1)的距离（单位：格）
        # SciPy 语义：输入中非零元素视为“object”，计算其到背景(0)的距离
        # 所以传入 free_mask，得到的是 free 到最近 non-free(=障碍) 的距离
        dist_cells = distance_transform_edt(free_mask)

        # 转成米
        dist_m = dist_cells * self.resolution

        # 距离小于安全距离的全部标记为障碍/未知(1)，其余保留为 0
        self.grid_map = np.where(dist_m >= safe_distance_m, 0, 1)

    def inflate_map(self, safe_distance_m=1.0):
        """
        对外暴露的封装接口（方便在其他模块中调用）
        """
        self.inflate_obstacles(safe_distance_m=safe_distance_m)
    # ========= ⭐ 新增结束 =========
    
    def get_pic_dir(self):
        """返回当前脚本所在目录下的 pics/ 目录路径"""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        pic_dir = os.path.join(script_dir, "pics")
        os.makedirs(pic_dir, exist_ok=True)
        return pic_dir
            
    def visualize(self, filename="map"):
        """
        将当前栅格地图保存为图片到 scripts/pics/ 目录（带时间戳）
        """
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"{filename}_{timestamp}.png"

        save_path = os.path.join(self.get_pic_dir(), filename)

        plt.figure(figsize=(6, 6))
        plt.imshow(self.grid_map.T, cmap='gray_r', origin='lower')

        plt.xticks(np.arange(-0.5, self.grid_size, 1), [])
        plt.yticks(np.arange(-0.5, self.grid_size, 1), [])
        plt.grid(color='black', linestyle='-', linewidth=0.5)

        plt.savefig(save_path, dpi=200, bbox_inches='tight')
        plt.close()

        print(f"[GridMap] 地图已保存到: {save_path}")

    def visualize_edt(self, filename="edt"):
        """
        保存 EDT 距离场图像（带时间戳）
        """
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"{filename}_{timestamp}.png"

        save_path = os.path.join(self.get_pic_dir(), filename)

        obstacle_mask = (self.grid_map == 1)
        dist_cells = distance_transform_edt(~obstacle_mask)
        dist_m = dist_cells * self.resolution

        plt.figure(figsize=(6, 6))
        plt.imshow(dist_m.T, cmap='jet', origin='lower')
        plt.colorbar(label="Distance to nearest obstacle (meters)")
        plt.title("Euclidean Distance Transform (EDT) Map")

        plt.xticks([])
        plt.yticks([])

        plt.savefig(save_path, dpi=200, bbox_inches='tight')
        plt.close()

        print(f"[GridMap] EDT 图像已保存到: {save_path}")


if __name__ == "__main__":
    grid_map = GridMap()

    # 1. 用锚点 LOS 初始化基础可通行通道（可以调 free_expand 扩宽初始通道）
    grid_map.map_init(free_expand=1)

    # 2. 对障碍物做“膨胀”，比如保持 1 米安全距离
    #    分辨率 0.5 -> 1 米 ≈ 2 格
    grid_map.inflate_map(safe_distance_m=1.0)

    # 3. 可视化结果
    grid_map.visualize()
