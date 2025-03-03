import rospy, json, yaml
import numpy as np
import heapq
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped

class GridMap:
    def __init__(self, size=50, resolution=0.1):
        """
        初始化栅格地图
        :param size: 地图大小（米）
        :param resolution: 栅格分辨率（米）
        """
        self.size = size
        self.resolution = resolution
        self.grid_anchors = []
        self.grid_size = int(size / resolution)  # 计算栅格总数
        self.grid_map = np.ones((self.grid_size, self.grid_size), dtype=np.int8)  # 0 表示可通行，1 表示障碍物

    def set_obstacle(self, gx, gy):
        """ 在地图上设置障碍物 """
        self.grid_map[gx, gy] = 1  # 1 表示障碍物
        
    def set_free(self, gx, gy):
        self.grid_map[gx, gy] = 0  # 0 表示可通行

    def to_grid(self, x, y):
        """ 将实际坐标转换为栅格坐标 """
        gx = int((x + self.size / 2) / self.resolution)
        gy = int((y + self.size / 2) / self.resolution)
        return gx, gy

    def to_real(self, gx, gy):
        """ 将栅格坐标转换为实际坐标 """
        x = round(gx * self.resolution - self.size / 2, 1)
        y = round(gy * self.resolution - self.size / 2, 1)
        return x, y

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
    
    def map_init(self, real_anchors):
        for anchor in real_anchors:
            grid_point = self.to_grid(anchor['x'], anchor['y'])
            self.grid_anchors.append({
                "id": anchor['id'],
                "position": grid_point
            })
        
        # TODO: 根据已知信息，设置锚点之间的连接状态。
    
    def map_update(self, start_real, end_real, mode:str):
        """
        更新地图，占据或空闲
        :param mode: occupy or free
        """
        points = self.get_line_grids(start_real, end_real)
        for point in points:
            if mode == "occupy":
                self.grid_map[point[0], point[1]] = 1
            else:
                self.grid_map[point[0], point[1]] = 0


class DStarLite:
    def __init__(self, grid_map):
        self.map = grid_map
        self.g_score = {}
        self.rhs = {}
        self.open_list = []
        self.km = 0
        self.start = None
        self.goal = None
        self.last_start = None

    def heuristic(self, node1, node2):
        return abs(node1[0] - node2[0]) + abs(node1[1] - node2[1])

    def initialize(self, start, goal):
        self.start = start
        self.goal = goal
        self.g_score = {start: float('inf')}
        self.rhs = {goal: 0}
        self.open_list = []
        heapq.heappush(self.open_list, (self.calculate_key(goal), goal))
        self.compute_shortest_path()
        self.last_start = start

    def calculate_key(self, node):
        min_g_rhs = min(self.g_score.get(node, float('inf')), self.rhs.get(node, float('inf')))
        return (min_g_rhs + self.heuristic(self.start, node) + self.km, min_g_rhs)

    def update_vertex(self, node):
        if node != self.goal:
            self.rhs[node] = min(self.g_score.get(n, float('inf')) + 1 for n in self.get_neighbors(node))
        if node in [n for _, n in self.open_list]:
            self.open_list = [(k, n) for k, n in self.open_list if n != node]
            heapq.heapify(self.open_list)
        if self.g_score.get(node, float('inf')) != self.rhs.get(node, float('inf')):
            heapq.heappush(self.open_list, (self.calculate_key(node), node))

    def compute_shortest_path(self):
        while self.open_list and (self.open_list[0][0] < self.calculate_key(self.start) or self.rhs.get(self.start, float('inf')) != self.g_score.get(self.start, float('inf'))):
            k, node = heapq.heappop(self.open_list)
            if self.g_score.get(node, float('inf')) > self.rhs.get(node, float('inf')):
                self.g_score[node] = self.rhs[node]
                for neighbor in self.get_neighbors(node):
                    self.update_vertex(neighbor)
            else:
                self.g_score[node] = float('inf')
                self.update_vertex(node)
                for neighbor in self.get_neighbors(node):
                    self.update_vertex(neighbor)

    def get_neighbors(self, node):
        x, y = node
        neighbors = [(x + dx, y + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]]
        return [(nx, ny) for nx, ny in neighbors if 0 <= nx < self.map.grid_size and 0 <= ny < self.map.grid_size and self.map.grid[nx, ny] == 0]

    def update_obstacle(self, obstacle_nodes):
        for node in obstacle_nodes:
            self.g_score[node] = float('inf')
            self.rhs[node] = float('inf')
            for neighbor in self.get_neighbors(node):
                self.update_vertex(neighbor)
        self.compute_shortest_path()

    def move_start(self, new_start):
        if new_start != self.start:
            self.km += self.heuristic(self.last_start, new_start)
            self.start = new_start
            self.last_start = new_start
            self.compute_shortest_path()

    def find_path(self):
        path = []
        current = self.start
        while current != self.goal:
            path.append(current)
            neighbors = self.get_neighbors(current)
            if not neighbors:
                return None
            current = min(neighbors, key=lambda n: self.g_score.get(n, float('inf')))
        path.append(self.goal)
        return path


class UAVNavigator:
    def __init__(self):
        """ 初始化 ROS 订阅并执行路径规划 """
        rospy.init_node("uav_path_planner", anonymous=True)
        self.grid_map = GridMap()  # 初始化地图
        self.goal_real = (15, 15)  # 设置目标位置
        self.current_position = (0, 0)  # 记录无人机当前位置
        self.los_data = []
        self.uwb_anchors_file = '/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/UWB_Anchors.yml'

        rospy.Subscriber("/ardrone_1/odometry_sensor1/position", PointStamped, self.position_callback)
        rospy.Subscriber("/los_status_json", String, self.los_callback)
        self.rate = rospy.Rate(1)  # 控制更新频率

    def position_callback(self, msg):
        """ 订阅无人机位置，并更新路径规划 """
        self.current_position = (msg.point.x, msg.point.y)
        
    def los_callback(self, msg):
        try:
            data = json.loads(msg.data)  # 解析 JSON 消息
            self.los_data.append(data)  # 存储数据
        except json.JSONDecodeError as e:
            rospy.logerr(f"解析 JSON 失败: {e}")
            
    def load_uwb_anchors(self):
        """
        从 UWB_Anchors.yml 文件加载锚点坐标
        """
        with open(self.uwb_anchors_file, 'r') as file:
            data = yaml.safe_load(file)

        anchors = []
        for anchor in data['UWB_Anchors']:
            anchors.append({
                "id": anchor['id'],
                "position": Point(anchor['x'], anchor['y'], anchor['z'])
            })
        return anchors
        
    def find_path(self, grid_map, start_real, goal_real):
        """ 计算路径 """
        start = grid_map.to_grid(*start_real)
        goal = grid_map.to_grid(*goal_real)

        d_star = DStarLite(grid_map)
        d_star.initialize(start, goal)
        d_star.compute_shortest_path()
        return d_star.find_path()
    
    def run(self):
        """
        持续进行路径规划
        """
        while not rospy.is_shutdown():
            # 计算新路径
            path = self.find_path(self.grid_map, self.current_position, self.goal_real)
            if path:
                rospy.loginfo("新路径: ")
                for p in path:
                    rospy.loginfo(self.grid_map.to_real(*p))
            else:
                rospy.logwarn("无法找到路径！")
                
            self.rate.sleep()


if __name__ == "__main__":
    navigator = UAVNavigator()
    navigator.run()
