import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))  # 添加当前脚本所在目录

import rospy, json
from std_msgs.msg import String
from grid_map import GridMap
from path_searching import AStar
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import math
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header




class UAVNavigator:
    def __init__(self):
        """ 初始化 ROS 订阅并执行路径规划 """
        
        rospy.init_node("uav_navigator", anonymous=True)
        
        self.resolution = 0.5  # 栅格分辨率（米）
        
        # 初始化
        self.grid_map = GridMap(size=50, resolution=self.resolution)
        self.grid_map.map_init()
        self.a_star = AStar(self.grid_map, edt_hard_min=self.resolution * 1.5)
        
        self.goal_real = (18, 5)  # 目标位置
        self.current_position = (-15.0, -15.0)  # 记录无人机当前位置
        self.start_real = self.current_position # A*规划起始位置（上一次规划的目标位置）,初始为当前无人机位置
        self.los_data = []  # 存储LOS状态
        self.key_waypoints = []  # 存储关键航点
        self.active_goal_grid = None   # 当前正在飞的航点（栅格）
        self.active_goal_real = None   # 当前正在飞的航点（真实）
        self.need_replan = True        # 是否需要重新规划
        self._map_initialized = False

        # 订阅UWB传感器数据
        rospy.Subscriber("/los_status_json", String, self.los_callback)
        self.bspline_ctrl_pub = rospy.Publisher(
            "/bspline/ctrl_points",
            PoseArray,
            queue_size=1
        )

        self.edt_map_pub = rospy.Publisher(
            "/bspline/edt_map",
            OccupancyGrid,
            queue_size=1,
            latch=True
        )
        
        self.goal_pub = rospy.Publisher(
            "/ardrone_1/command/pose",
            PoseStamped,
            queue_size=10
        )

        self.rate = rospy.Rate(2)  # 控制更新频率

        # 订阅无人机位置话题
        rospy.Subscriber('/ardrone_1/odometry_sensor1/odometry', Odometry, self.odometry_callback)

        # 预定义的航点列表
        self.waypoints = []
        # self.current_waypoint_index = 0  # 记录当前执行的航点索引
        self.tolerance = 0.2  # 目标点到达的误差范围

        rospy.loginfo("Drone Controller Initialized")

    def odometry_callback(self, msg):
        """
        回调函数，接收并保存无人机当前位置。
        """
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

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
        # rospy.loginfo(f"current position: {self.current_position}, goal position: ({x}, {y}), Distance to goal: {distance}")

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
        
    def publish_start_goal(self, start_real, goal_real):
        """
        发布给 Bspline 优化节点的 起点-终点
        """
        
        # ======== 必须的安全检查 ========
        if start_real is None or goal_real is None:
            rospy.logwarn(
                "[uav_navigator] start_real or goal_real is None, skip publish."
            )
            return
        
        msg = PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        p_start = Pose()
        p_start.position.x = start_real[0]
        p_start.position.y = start_real[1]
        p_start.position.z = 1.0

        p_goal = Pose()
        p_goal.position.x = goal_real[0]
        p_goal.position.y = goal_real[1]
        p_goal.position.z = 1.0

        msg.poses.append(p_start)
        msg.poses.append(p_goal)

        self.start_goal_pub.publish(msg)

        rospy.loginfo(
            f"[uav_navigator] Publish start-goal: "
            f"start={start_real}, goal={goal_real}"
        )

    def publish_edt_map(self):
        """
        Publish EDT map as nav_msgs/OccupancyGrid
        Each cell stores EDT distance (meters * 100, clipped)
        """

        if self.grid_map.edt_map is None:
            return

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.info.resolution = self.resolution
        msg.info.width = self.grid_map.grid_size
        msg.info.height = self.grid_map.grid_size

        # map origin (left-bottom corner)
        msg.info.origin.position.x = -self.grid_map.size / 2.0
        msg.info.origin.position.y = -self.grid_map.size / 2.0
        msg.info.origin.position.z = 0.0

        # OccupancyGrid 要求 int8 [-1,100]
        # 我们把 EDT (m) → cm → clip 到 [0,100]
        edt = self.grid_map.edt_map
        data = []

        for y in range(self.grid_map.grid_size):
            for x in range(self.grid_map.grid_size):
                d = edt[x, y]
                if d <= 0.0:
                    v = 0
                else:
                    v = int(min(d * 100.0, 100.0))
                data.append(v)

        msg.data = data
        self.edt_map_pub.publish(msg)

    def publish_bspline_ctrl_points(self, ctrl_pts_real):
        """
        发布给 Bspline 优化节点的 4 个控制点（cubic B-spline）

        ctrl_pts_real: [(x0,y0), (x1,y1), (x2,y2), (x3,y3)]
        """

        # ======== 安全检查 ========
        if ctrl_pts_real is None or len(ctrl_pts_real) < 4:
            rospy.logwarn(
                "[uav_navigator] ctrl_pts_real invalid (<4), skip publish."
            )
            return

        msg = PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        # ======== 按顺序填入 4 个控制点 ========
        for i, pt in enumerate(ctrl_pts_real[:4]):
            p = Pose()
            p.position.x = pt[0]
            p.position.y = pt[1]
            p.position.z = 1.0
            msg.poses.append(p)

        # 发布
        self.bspline_ctrl_pub.publish(msg)

        rospy.loginfo(
            "[uav_navigator] Publish B-spline ctrl points: "
            f"{ctrl_pts_real[:4]}"
        )
    
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
                
                self.publish_edt_map()

                # ========== 3. 更新 A* 使用的地图 ==========
                self.a_star.map_reconstruct(self.grid_map)

                # =================================================
                # 状态 A：当前没有执行航点 → 允许重新规划
                # =================================================
                if self.active_goal_real is None and self.need_replan:

                    rospy.loginfo("[Navigator] Replanning...")

                    grid_path = self.find_path(self.start_real, self.goal_real)
                    if not grid_path:
                        rospy.logwarn("[Navigator] No path found.")
                        self.rate.sleep()
                        continue
                    
                    # 2. 提取“第一段窗口”的 4 个控制点（栅格）
                    ctrl_pts_grid = self.a_star.extract_first_window_ctrl_points(
                        path=grid_path, 
                        min_dist_m=2.0
                    )
                    
                    if not ctrl_pts_grid or len(ctrl_pts_grid) < 4:
                        rospy.logwarn("[Navigator] Failed to extract B-spline control points. Directly flying to goal.")
                        
                        self.active_goal_grid = grid_path[-1]
                        self.active_goal_real = self.grid_map.to_real(
                            self.active_goal_grid[0],
                            self.active_goal_grid[1]
                            )
                        
                        self.publish_waypoint(self.active_goal_real)
                        rospy.loginfo(
                        f"[Navigator] Active goal grid={self.active_goal_grid}, "
                        f"real={self.active_goal_real}")
                        self.rate.sleep()
                        continue

                    # 3. 栅格 → 真实坐标
                    ctrl_pts_real = [
                        self.grid_map.to_real(gx, gy)
                        for gx, gy in ctrl_pts_grid
                    ]
                    
                    self.active_goal_real = ctrl_pts_real[-1]
                    self.active_goal_grid = ctrl_pts_grid[-1]
                    
                    self.publish_bspline_ctrl_points(ctrl_pts_real)


                    # 发布给 EgoPlanner / move_base
                    # self.publish_waypoint(self.active_goal_real)

                    rospy.loginfo(
                        f"[Navigator] Active goal grid={self.active_goal_grid}, "
                        f"real={self.active_goal_real}")

                    self.need_replan = False

                # =================================================
                # 状态 B：正在飞 → 只判断是否到达
                # =================================================
                elif self.active_goal_real is not None:
                    rospy.loginfo("[Navigator] flying ...")
                    if self.has_reached_goal(
                        self.active_goal_real[0],
                        self.active_goal_real[1],
                        1.0
                    ):
                        rospy.loginfo("[Navigator] Active goal reached.")

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