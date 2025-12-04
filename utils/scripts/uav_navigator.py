import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))  # 添加当前脚本所在目录

import rospy, json
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import String
from grid_map import GridMap
from path_searching import AStar
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math

# TODO: A*路径规划实时更新地图
# TODO: 重写路径规划（run函数）等相关代码
# TODO: 在搜索完路径后，发送给无人机指挥无人机飞行


class UAVNavigator:
    def __init__(self):
        """ 初始化 ROS 订阅并执行路径规划 """
        rospy.init_node("uav_path_planner", anonymous=True)
        self.grid_map = GridMap()  # 初始化地图
        self.grid_map.map_init()
        self.goal_real = (0, 15)  # 目标位置
        self.current_position = (-15, -15)  # 记录无人机当前位置
        # self.temp_los_data = []
        self.los_data = []  # 存储LOS状态

        # 订阅传感器数据
        # rospy.Subscriber("/ardrone_1/odometry_sensor1/position", PointStamped, self.position_callback)
        rospy.Subscriber("/los_status_json", String, self.los_callback)
        self.rate = rospy.Rate(2)  # 控制更新频率

        # 初始化 A* 规划器
        self.a_star = AStar(self.grid_map)
        
                # 当前无人机的位置信息
        self.current_position = None

        # 订阅无人机位置话题
        rospy.Subscriber('/ardrone_1/odometry_sensor1/odometry', Odometry, self.odometry_callback)

        # 发布无人机目标位置的 PoseStamped 消息
        self.goal_pub = rospy.Publisher('/ardrone_1/move_base_simple/goal', PoseStamped, queue_size=10)

        # 预定义的航点列表
        self.waypoints = []
        self.current_waypoint_index = 0  # 记录当前执行的航点索引
        self.tolerance = 0.2  # 目标点到达的误差范围

        rospy.loginfo("Drone Controller Initialized")

    def odometry_callback(self, msg):
        """
        回调函数，接收并保存无人机当前位置。
        """
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # rospy.loginfo(f"Current Position -> x: {self.current_position.position.x}, "
        #               f"y: {self.current_position.position.y}, "
        #               f"z: {self.current_position.position.z}")
                    # 只有当当前位置可用时，才检查是否到达目标点

    def has_reached_goal(self, x, y, z):
        """
        检查无人机是否到达目标点
        """
        if self.current_position is None:
            return False

        dx = self.current_position.position.x - x
        dy = self.current_position.position.y - y
        # dz = self.current_position.position.z - z
        distance = math.sqrt(dx**2 + dy**2)

        return distance < self.tolerance  # 如果距离小于设定的阈值，则认为到达
    
    # def position_callback(self, msg):
    #     """ 订阅无人机位置，并增量更新路径 """
    #     self.current_position = (msg.point.x, msg.point.y)

    def los_callback(self, msg):
        try:
            self.los_data = json.loads(msg.data)  # 解析 JSON 消息
        except json.JSONDecodeError as e:
            rospy.logerr(f"解析 JSON 失败: {e}")

    def find_path(self, start_real, end_real):
        """ 使用 A* 计算路径 """
        if not start_real or not end_real:
            return
        start_grid = self.grid_map.to_grid(start_real[0], start_real[1])
        end_grid = self.grid_map.to_grid(end_real[0], end_real[1])
        
        return self.a_star.find_path(start_grid, end_grid)

    # def los_update(self):
    #     """更新无人机与各锚点之间的视距情况""" 
    #     for d in self.temp_los_data:
    #         self.los_data[d["id"]] = d["LOS"]  # 存储数据: id(int): True/False
    #     rospy.loginfo(f"los_data: {self.los_data}")
        
        
    def map_update(self):
        self.grid_map.map_update_by_los(self.current_position, self.los_data)
    
    def move_to_next_waypoint_manual(self, path):
        """
        发布目标位置，依次前往航点，发布真实坐标的航点
        """
        if self.current_position is None:
            rospy.logwarn("Current position is not yet available. Waiting for odometry...")
            return

        if not path or len(path) == 0:
            rospy.loginfo("All waypoints reached. Task finished.")
            return

        # 将栅格坐标转换为真实坐标
        x, y = self.grid_map.to_real(path[0][0], path[0][1])

        # 构造目标位置的 PoseStamped 消息
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"  # 确保使用正确的参考系
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 1.0

        # 方向四元数设置为无旋转
        # goal.pose.orientation.w = 1.0

        # 发布目标位置
        self.goal_pub.publish(goal)
        rospy.loginfo(f"Moving to Waypoint {self.current_waypoint_index} -> x: {x}, y: {y}, z: {1.0}")

    def run(self):
        try:
            """ 持续进行路径规划 """
            while not rospy.is_shutdown():
                # self.los_update()
                # 更新地图
                self.map_update()
                # self.grid_map.apply_safety_margin()
                self.a_star.map_reconstruct(self.grid_map)
                
                # 栅格地图路径（未转换到真实坐标系）
                path = self.find_path(self.current_position, self.goal_real)
                rospy.loginfo(f"grid path: {path}")
                
                key_waypoints = self.a_star.extract_key_waypoints(path)
                
                rospy.loginfo(f"key waypoint path: {key_waypoints}")
                
                self.move_to_next_waypoint_manual(key_waypoints)
                
                self.rate.sleep()
                
        finally:
            self.grid_map.visualize()

if __name__ == "__main__":
    navigator = UAVNavigator()
    navigator.run()
