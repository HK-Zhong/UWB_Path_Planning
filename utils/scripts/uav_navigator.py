import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # 添加当前脚本所在目录

import rospy, json
from std_msgs.msg import String
from grid_map import GridMap
from path_searching import EDTAwareAStarPlanner, GridAStarPlanner, DijkstraPlanner
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Vector3
import math
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

from experiment_logger import SimpleExperimentLogger
import los_detect
from geometry_msgs.msg import Point


class UAVNavigator:
    def __init__(self):
        """ 初始化 ROS 订阅并执行路径规划 """

        rospy.init_node("uav_navigator", anonymous=True)

        # 从 launch 参数读取分辨率（默认 0.5m）
        self.resolution = rospy.get_param("~resolution", 0.5)
        self.map_no = rospy.get_param("~map_no", 1)

        rospy.loginfo(f"[UAVNavigator] Using map {self.map_no}, grid resolution = {self.resolution} m")

        # 目标位置（允许不是锚点本身，而是任意坐标点）
        self.goal_real = (18, 5) if self.map_no == 1 else (21, 4)

        # 初始化
        self.grid_map = GridMap(size=50, resolution=self.resolution, map_no=self.map_no)
        self.grid_map.map_init()

        # 复用 LOSDetector 的几何检测能力：显式传入 map_no，关闭其 ROS I/O
        self.goal_los_helper = los_detect.LOSDetector(
            rate=2,
            init_ros_node=False,
            enable_ros_io=False,
            map_no=self.map_no,
        )

        # 在系统启动时，对 goal_real 额外执行一次 goal-anchor LOS 更新
        self.update_goal_los_state_once()

        self.planner = EDTAwareAStarPlanner(self.grid_map, edt_hard_min=self.resolution * 1.5)
        # self.planner = GridAStarPlanner(self.grid_map)
        # self.planner = DijkstraPlanner(self.grid_map)
        self.current_position = (-18.0, -18.0)  # 记录无人机当前位置
        self.start_real = self.current_position  # A*规划起始位置（上一次规划的目标位置）,初始为当前无人机位置
        self.los_data = []  # 存储LOS状态
        self.key_waypoints = []  # 存储关键航点
        self.active_goal_grid = None  # 当前正在飞的航点（栅格）
        self.active_goal_real = None  # 当前正在飞的航点（真实）
        self.need_replan = True  # 是否需要重新规划
        self._map_initialized = False

        # 订阅UWB传感器数据
        rospy.Subscriber("/los_status_json", String, self.los_callback)
        self.optimizer_ctrl_pub = rospy.Publisher(
            "/optimizer/ctrl_points",
            PoseArray,
            queue_size=1
        )

        self.edt_map_pub = rospy.Publisher(
            "/optimizer/edt_map",
            OccupancyGrid,
            queue_size=1,
            latch=True
        )

        self.goal_pub = rospy.Publisher(
            "/ardrone_1/command/pose",
            PoseStamped,
            queue_size=10
        )

        # ===== Subscribe optimizer cost =====
        self._last_opt_cost = None  # (cost_v, cost_a, cost_j)
        self._last_opt_cost_stamp = None
        rospy.Subscriber("/optimizer/cost", Vector3, self.optimizer_cost_callback)

        # ===== Subscribe optimizer EDT stats =====
        # msg.x = edt_min (m), msg.y = edt_mean (m)
        self._last_opt_edt = None  # (edt_min, edt_mean)
        self._last_opt_edt_stamp = None
        rospy.Subscriber("/optimizer/edt", Vector3, self.optimizer_edt_callback)

        # ===== Subscribe optimizer trajectory info =====
        # msg.x = traj_length_m, msg.y = traj_time_s
        self._last_traj_info = None  # (length_m, time_s)
        self._last_traj_info_stamp = None
        rospy.Subscriber("/optimizer/traj_info", Vector3, self.optimizer_traj_info_callback)

        self.rate = rospy.Rate(2)  # 控制更新频率

        # 订阅无人机位置话题
        rospy.Subscriber('/ardrone_1/odometry_sensor1/odometry', Odometry, self.odometry_callback)

        # 预定义的航点列表
        self.waypoints = []
        # self.current_waypoint_index = 0  # 记录当前执行的航点索引
        self.tolerance = 0.2  # 目标点到达的误差范围

        self.logger = SimpleExperimentLogger()
        self.logger.start(planner_name=self.planner.__class__.__name__)

        rospy.loginfo("[UAVNavigator] Drone Controller Initialized")

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
        distance = math.sqrt(dx ** 2 + dy ** 2)
        # rospy.loginfo(f"current position: {self.current_position}, goal position: ({x}, {y}), Distance to goal: {distance}")

        return distance < self.tolerance  # 如果距离小于设定的阈值，则认为到达

    def los_callback(self, msg):
        try:
            self.los_data = json.loads(msg.data)  # 解析 JSON 消息
        except json.JSONDecodeError as e:
            rospy.logerr(f"解析 JSON 失败: {e}")

    def optimizer_cost_callback(self, msg: Vector3):
        """Receive optimizer control costs (v, a, jerk)."""
        self._last_opt_cost = (float(msg.x), float(msg.y), float(msg.z))
        self._last_opt_cost_stamp = rospy.Time.now()

    def optimizer_edt_callback(self, msg: Vector3):
        """Receive optimizer EDT metrics (edt_min, edt_mean)."""
        self._last_opt_edt = (float(msg.x), float(msg.y))
        self._last_opt_edt_stamp = rospy.Time.now()

    def optimizer_traj_info_callback(self, msg: Vector3):
        """Receive optimizer trajectory info (length_m, time_s)."""
        self._last_traj_info = (float(msg.x), float(msg.y))
        self._last_traj_info_stamp = rospy.Time.now()

    def update_goal_los_state_once(self):
        """
        在系统初始化阶段，把 goal_real 当作一个“临时观测点”，
        计算它到所有 anchors 的 LOS 结果，并直接调用
        self.grid_map.map_update_by_los(self.goal_real, los_data, free_expand=...)
        对地图做一次额外更新。
        """
        if self.goal_real is None:
            rospy.logwarn("[UAVNavigator] goal_real is None, skip startup goal LOS update.")
            return

        # 构造一个任意查询点（z 取与指令目标一致的 1.0m）
        goal_point = Point()
        goal_point.x = float(self.goal_real[0])
        goal_point.y = float(self.goal_real[1])
        goal_point.z = 1.0

        try:
            goal_los_data = self.goal_los_helper.check_point_los_to_anchors(goal_point)
        except Exception as e:
            rospy.logwarn(f"[UAVNavigator] Failed to compute startup goal LOS: {e}")
            return

        # 至少需要与一个 anchor 保持 LOS；否则给出提醒
        connected_ids = [item["id"] for item in goal_los_data if item.get("LOS", False)]
        if len(connected_ids) == 0:
            rospy.logwarn("[UAVNavigator] goal_real is not LOS-connected to any anchor at startup.")
            return

        rospy.loginfo(
            f"[UAVNavigator] Startup goal LOS-connected anchors: {connected_ids}"
        )

        # 只保留与 goal_real 距离最近的那个 LOS anchor，用于地图更新
        anchor_pos_map = {
            a["id"]: a["position"]
            for a in self.goal_los_helper.uwb_anchors
        }

        gx, gy = float(self.goal_real[0]), float(self.goal_real[1])
        best_anchor_id = None
        best_dist = float("inf")

        for aid in connected_ids:
            pos = anchor_pos_map.get(aid, None)
            if pos is None:
                continue
            dx = float(pos.x) - gx
            dy = float(pos.y) - gy
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < best_dist:
                best_dist = dist
                best_anchor_id = aid

        if best_anchor_id is None:
            rospy.logwarn("[UAVNavigator] Failed to select nearest LOS anchor for goal_real.")
            return

        filtered_goal_los_data = []
        for item in goal_los_data:
            filtered_goal_los_data.append({
                "id": item["id"],
                "LOS": (item["id"] == best_anchor_id)
            })

        rospy.loginfo(
            f"[UAVNavigator] Startup goal uses nearest LOS anchor id={best_anchor_id}, dist={best_dist:.3f} m"
        )

        # 对 goal 的 LOS 只做一次较窄更新，避免过度扩张。
        goal_free_expand = 2 if self.resolution == 0.25 else 1
        self.grid_map.map_update_by_los(self.goal_real, filtered_goal_los_data, free_expand=goal_free_expand)
        rospy.loginfo(
            f"[UAVNavigator] Startup goal LOS update applied with free_expand={goal_free_expand}."
        )

    def plan(self, start_real, end_real):
        """ 使用 A* 计算路径 """
        if not start_real or not end_real:
            return
        start_grid = self.grid_map.to_grid(start_real[0], start_real[1])
        end_grid = self.grid_map.to_grid(end_real[0], end_real[1])

        return self.planner.plan(start_grid, end_grid)

    def map_update(self, free_expand):
        self.grid_map.map_update_by_los(self.current_position, self.los_data, free_expand=free_expand)

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
        # 我们改为：把 EDT (m) 映射到一个可调上限 max_range_m → [0,100]
        # 这样可以表达 >1m 的距离，而不是被强行截断到 1m

        edt = self.grid_map.edt_map
        data = []

        max_range_m = 5.0  # 可调参数：最多表达 5m 的安全距离
        scale = 100.0 / max_range_m  # 线性映射到 [0,100]

        for y in range(self.grid_map.grid_size):
            for x in range(self.grid_map.grid_size):
                d = edt[x, y]

                if d <= 0.0:
                    v = 0
                else:
                    v = int(min(d * scale, 100.0))

                data.append(v)

        msg.data = data
        self.edt_map_pub.publish(msg)

    def publish_optimizer_ctrl_points(self, ctrl_pts_real):
        """
        发布给 optimizer 优化节点的 4 个控制点（cubic B-spline）

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
        self.optimizer_ctrl_pub.publish(msg)

        rospy.loginfo(
            "[UAVNavigator] Publish optimizer ctrl points: "
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
        rospy.loginfo(f"[UAVNavigator] Moving from {self.current_position}")
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

        # ===== 状态 1：任务开始前，执行一次宽更新，宽度为一边固定0.5m =====
        if not self._map_initialized:
            rospy.loginfo("[update_map_stateful] Initial wide LOS update...")
            if self.resolution == 0.25:
                self.map_update(free_expand=2)
            else:
                self.map_update(free_expand=1)
            self._map_initialized = True
            return

        # ===== 状态 2：任务执行中，持续窄更新 =====
        if self.resolution == 0.25:
            self.map_update(free_expand=1)
        else:
            self.map_update(free_expand=0)
        rospy.loginfo("[update_map_stateful] Continuous narrow LOS update...")

    def run(self):
        try:
            while not rospy.is_shutdown():

                # ========== 1. 更新地图（LOS → grid_map） ==========
                self.update_map_stateful()

                # ========== 2. 更新 EDT 地图 ==========
                # ⚠️ 注意：这是“重算 edt_map”，不是改 grid_map
                self.grid_map.update_edt()

                gx, gy = self.grid_map.to_grid(self.current_position[0], self.current_position[1])
                rospy.logwarn(f"[DEBUG][AFTER EDT] start grid=({gx},{gy}) "
                              f"grid_val={self.grid_map.grid_map[gx, gy]} "
                              f"EDT(m)={self.grid_map.edt_map[gx, gy]}")

                self.publish_edt_map()

                # ========== 3. 更新 A* 使用的地图 ==========
                self.planner.map_reconstruct(self.grid_map)

                # =================================================
                # 状态 A：当前没有执行航点 → 允许重新规划
                # =================================================
                if self.active_goal_real is None and self.need_replan:

                    rospy.loginfo("[UAVNavigator] Replanning...")

                    # 1. 路径规划，输出栅格坐标路径
                    self.logger.tic()
                    grid_path = self.plan(self.start_real, self.goal_real)
                    planning_time = self.logger.toc()

                    if not grid_path:
                        rospy.logwarn("[Navigator] No path found.")
                        self.rate.sleep()
                        continue

                    # 2. 提取“第一段窗口”的 4 个控制点（栅格）
                    ctrl_pts_grid = self.planner.extract_first_window_ctrl_points(
                        path=grid_path,
                        min_dist_m=4.0
                    )

                    if ctrl_pts_grid is None:
                        rospy.logwarn("[Navigator] extract_first_window_ctrl_points returned None.")
                        self.rate.sleep()
                        continue

                    if len(ctrl_pts_grid) < 4:
                        rospy.logwarn("[UAVNavigator] Failed to extract 4 control points. Directly flying to goal.")
                        self.active_goal_grid = ctrl_pts_grid[-1]
                        self.active_goal_real = self.grid_map.to_real(
                            self.active_goal_grid[0],
                            self.active_goal_grid[1]
                        )

                        self.publish_waypoint(self.active_goal_real)
                        rospy.loginfo(
                            f"[UAVNavigator] Active goal grid={self.active_goal_grid}, "
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

                    # 4. 轨迹优化
                    self.logger.tic()
                    self.publish_optimizer_ctrl_points(ctrl_pts_real)
                    optimizing_time = self.logger.toc()

                    cost_v, cost_a, cost_j = (None, None, None)
                    if self._last_opt_cost is not None:
                        cost_v, cost_a, cost_j = self._last_opt_cost

                    edt_min, edt_mean = (None, None)
                    if self._last_opt_edt is not None:
                        edt_min, edt_mean = self._last_opt_edt

                    rospy.loginfo(
                        f"[UAVNavigator] Optimizer EDT: edt_min={edt_min}, edt_mean={edt_mean}"
                    )

                    traj_len_m, traj_time_exec = (None, None)
                    if self._last_traj_info is not None:
                        traj_len_m, traj_time_exec = self._last_traj_info

                    rospy.loginfo(
                        f"[UAVNavigator] Optimizer TRAJ: length_m={traj_len_m}, time_s={traj_time_exec}"
                    )

                    self.logger.log_segment(
                        start_real=self.start_real,
                        goal_real=self.active_goal_real,
                        planning_time=planning_time,
                        optimization_time=optimizing_time,
                        cost_v=cost_v,
                        cost_a=cost_a,
                        cost_j=cost_j,
                        edt_min=edt_min,
                        edt_mean=edt_mean,
                        traj_len_m=traj_len_m,
                        traj_time_exec=traj_time_exec,
                    )

                    rospy.loginfo(
                        f"[UAVNavigator] Active goal grid={self.active_goal_grid}, "
                        f"real={self.active_goal_real}")

                    self.need_replan = False

                # =================================================
                # 状态 B：正在飞 → 只判断是否到达
                # =================================================
                elif self.active_goal_real is not None:
                    rospy.loginfo("[UAVNavigator] flying ...")
                    if self.has_reached_goal(
                            self.active_goal_real[0],
                            self.active_goal_real[1],
                            1.0
                    ):
                        rospy.loginfo("[UAVNavigator] Active goal reached.")

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
            self.logger.save()


if __name__ == "__main__":
    navigator = UAVNavigator()
    navigator.run()
