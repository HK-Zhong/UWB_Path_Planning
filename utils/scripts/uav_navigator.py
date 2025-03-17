import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))  # 添加当前脚本所在目录

import rospy, json
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import String
from grid_map import GridMap
from path_searching import AStar

# TODO: A*路径规划实时更新地图
# TODO: 重写路径规划（run函数）等相关代码
# TODO: 在搜索完路径后，发送给无人机指挥无人机飞行


class UAVNavigator:
    def __init__(self):
        """ 初始化 ROS 订阅并执行路径规划 """
        rospy.init_node("uav_path_planner", anonymous=True)
        self.grid_map = GridMap()  # 初始化地图
        self.grid_map.map_init()
        self.goal_real = (11, 4)  # 目标位置
        self.current_position = (-15, -15)  # 记录无人机当前位置
        self.temp_los_data = []
        self.los_data = {}  # 存储LOS状态

        # 订阅传感器数据
        rospy.Subscriber("/ardrone_1/odometry_sensor1/position", PointStamped, self.position_callback)
        rospy.Subscriber("/los_status_json", String, self.los_callback)
        self.rate = rospy.Rate(2)  # 控制更新频率

        # 初始化 A* 规划器
        self.a_star = AStar(self.grid_map)

    def position_callback(self, msg):
        """ 订阅无人机位置，并增量更新路径 """
        self.current_position = (msg.point.x, msg.point.y)

    def los_callback(self, msg):
        try:
            self.temp_los_data = json.loads(msg.data)  # 解析 JSON 消息
        except json.JSONDecodeError as e:
            rospy.logerr(f"解析 JSON 失败: {e}")

    def find_path(self, start_real, end_real):
        """ 使用 A* 计算路径 """
        start_grid = self.grid_map.to_grid(start_real[0], start_real[1])
        end_grid = self.grid_map.to_grid(end_real[0], end_real[1])
        
        return self.a_star.find_path(start_grid, end_grid)

    def los_update(self):
        """更新无人机与各锚点之间的视距情况""" 
        for d in self.temp_los_data:
            self.los_data[d["id"]] = d["LOS"]  # 存储数据: id(int): True/False
        
    def map_update(self):
        self.grid_map.map_update_by_los(self.current_position, self.los_data)

    def run(self):
        try:
            """ 持续进行路径规划 """
            while not rospy.is_shutdown():
                self.los_update()
                self.map_update()
                
                self.a_star.map_reconstruct(self.grid_map)
                
                path = self.find_path(self.current_position, self.goal_real)
                
                self.rate.sleep()
                
        finally:
            self.grid_map.visualize()

if __name__ == "__main__":
    navigator = UAVNavigator()
    navigator.run()
