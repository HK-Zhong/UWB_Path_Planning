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


class UAVNavigator:
    def __init__(self):
        """ 初始化 ROS 订阅并执行路径规划 """
        rospy.init_node("uav_path_planner", anonymous=True)
        self.resolution = 0.5  # 栅格分辨率（米）
        
        self.grid_map = GridMap(size=50, resolution=self.resolution)  # 初始化地图
        self.grid_map.map_init()
        self.goal_real = (0, 15)  # 目标位置
        self.current_position = (-15.0, -15.0)  # 记录无人机当前位置
        self.start_real = self.current_position # A*规划起始位置（上一次规划的目标位置）,初始为当前无人机位置
        self.los_data = []  # 存储LOS状态
        self.key_waypoints = []  # 存储关键航点
        self.active_goal_grid = None   # 当前正在飞的航点（栅格）
        self.active_goal_real = None   # 当前正在飞的航点（真实）
        self.need_replan = True        # 是否需要重新规划
        # Map update state
        self._map_initialized = False

        # self.map_update(free_expand=1)

        # 订阅UWB传感器数据
        rospy.Subscriber("/los_status_json", String, self.los_callback)
        self.rate = rospy.Rate(2)  # 控制更新频率

        # 初始化 A* 规划器
        self.a_star = AStar(self.grid_map, edt_hard_min=self.resolution)
        
        # 当前无人机的位置信息
        self.current_position = None

        # 订阅无人机位置话题
        rospy.Subscriber('/ardrone_1/odometry_sensor1/odometry', Odometry, self.odometry_callback)

        # 发布无人机目标位置的 PoseStamped 消息
        self.goal_pub = rospy.Publisher('/ardrone_1/move_base_simple/goal', PoseStamped, queue_size=10)

        # 预定义的航点列表
        self.waypoints = []
        # self.current_waypoint_index = 0  # 记录当前执行的航点索引
        self.tolerance = 0.3  # 目标点到达的误差范围

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

        dx = self.current_position[0] - x
        dy = self.current_position[1] - y
        # dz = self.current_position.position.z - z
        distance = math.sqrt(dx**2 + dy**2)

        return distance < self.tolerance  # 如果距离小于设定的阈值，则认为到达

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
     
    def map_update(self, free_expand):
        self.grid_map.map_update_by_los(self.current_position, self.los_data, free_expand=free_expand)
    
    def publish_waypoint(self, real_waypoint):
        """
        发布目标位置，依次前往航点，发布栅格坐标点
        """
        if self.current_position is None:
            rospy.logwarn("Current position is not yet available. Waiting for odometry...")
            return
        
        # 将栅格坐标转换为真实坐标
        x, y = real_waypoint[0], real_waypoint[1]

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
        # rospy.loginfo(f"UAV current position {self.current_position}")
        rospy.loginfo(f"Moving from {self.current_position}")
        rospy.loginfo(f"to Waypoint (x: {x}, y: {y}, z: {1.0})")
    
    def update_map_stateful(self):
        """
        Stateful map update:
        - Before mission start: one-time wide update (free_expand=1)
        - During mission: continuous narrow update (free_expand=0)
        """

        if self.current_position is None:
            return
        
        if not self.los_data:
            return

        # ===== 状态 1：任务开始前，执行一次宽更新 =====
        if not self._map_initialized:
            rospy.loginfo("[update_map_stateful] Initial wide LOS update (free_expand=1)")
            self.map_update(free_expand=1)
            self._map_initialized = True
            return

        # ===== 状态 2：任务执行中，持续窄更新 =====
        self.map_update(free_expand=0)
        rospy.loginfo("[update_map_stateful] Continuous narrow LOS update (free_expand=0)")

        
    def run(self):
        try:
            while not rospy.is_shutdown():
                                  
                # ========== 1. 更新地图（LOS → grid_map） ==========              
                self.update_map_stateful()
                
                # ========== 2. 更新 EDT 地图 ==========
                # ⚠️ 注意：这是“重算 edt_map”，不是改 grid_map
                self.grid_map.update_edt()

                # ========== 3. 更新 A* 使用的地图 ==========
                self.a_star.map_reconstruct(self.grid_map)

                # =================================================
                # 状态 A：当前没有执行航点 → 允许重新规划
                # =================================================
                if self.active_goal_real is None and self.need_replan:

                    rospy.loginfo("[Planner] Replanning...")

                    path = self.find_path(self.start_real, self.goal_real)
                    if not path:
                        rospy.logwarn("No path found.")
                        self.rate.sleep()
                        continue

                    key_waypoints = self.a_star.extract_key_waypoints(path)
                    if not key_waypoints:
                        rospy.logwarn("No key waypoints.")
                        self.rate.sleep()
                        continue

                    # 只取第一个关键航点作为执行目标
                    self.active_goal_grid = key_waypoints[0]
                    self.active_goal_real = self.grid_map.to_real(
                    self.active_goal_grid[0],
                    self.active_goal_grid[1])


                    # 发布给 EgoPlanner / move_base
                    self.publish_waypoint(self.active_goal_real)

                    rospy.loginfo(
                        f"[Planner] Active goal grid={self.active_goal_grid}, "
                        f"real={self.active_goal_real}")

                    self.need_replan = False

                # =================================================
                # 状态 B：正在飞 → 只判断是否到达
                # =================================================
                elif self.active_goal_real is not None:

                    if self.has_reached_goal(
                        self.active_goal_real[0],
                        self.active_goal_real[1],
                        1.0
                    ):
                        rospy.loginfo("[Planner] Active goal reached.")

                        # 规划起点前移（✔️ 正确的位置）
                        self.start_real = self.active_goal_real
                        self.key_waypoints.append(self.active_goal_grid)

                        # 清空执行态
                        self.active_goal_real = None
                        self.active_goal_grid = None
                        self.need_replan = True

                self.rate.sleep()

        finally:
            self.grid_map.visualize(self.key_waypoints)
            self.grid_map.visualize_edt(self.key_waypoints)
            # print("finished..")


if __name__ == "__main__":
    navigator = UAVNavigator()
    navigator.run()