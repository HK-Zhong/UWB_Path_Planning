import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))  # 添加当前脚本所在目录

import rospy, json
from std_msgs.msg import String
from grid_map import GridMap
from path_searching import EDTAwareAStarPlanner, GridAStarPlanner, DijkstraPlanner
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import math
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from experiment_logger import ExperimentLogger


class UAVNavigator:
    def __init__(self):
        """ 初始化 ROS 订阅并执行路径规划 """

        rospy.init_node("uav_navigator", anonymous=True)

        self.resolution = 0.5  # 栅格分辨率（米）

        # 初始化
        self.grid_map = GridMap(size=50, resolution=self.resolution)
        self.grid_map.map_init()
        self.planner = EDTAwareAStarPlanner(self.grid_map, edt_hard_min=self.resolution)
        # self.planner = GridAStarPlanner(self.grid_map)
        # self.planner = DijkstraPlanner(self.grid_map)

        self.goal_real = (18, 5)  # 目标位置
        self.current_position = (-15.0, -15.0)  # 记录无人机当前位置
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

        self.rate = rospy.Rate(2)  # 控制更新频率

        # 订阅无人机位置话题
        rospy.Subscriber('/ardrone_1/odometry_sensor1/odometry', Odometry, self.odometry_callback)

        # 预定义的航点列表
        self.waypoints = []
        # self.current_waypoint_index = 0  # 记录当前执行的航点索引
        self.tolerance = 0.2  # 目标点到达的误差范围

        rospy.loginfo("Drone Controller Initialized")

        # ===============================
        # Experiment logger (centralized)
        # ===============================
        self.exp_logger = ExperimentLogger()

        # ===============================
        # Episode state guards (avoid double-finish on shutdown)
        # ===============================
        self._episode_open = False
        self._execution_started = False
        self._episode_note = None

        # ===============================
        # Mission done guard
        # ===============================
        self._mission_done = False

        # ===============================
        # Trajectory record helpers
        # ===============================
    def _traj_record_start(self):
        """Start trajectory command recording (if logger supports it)."""
        if hasattr(self.exp_logger, "start_traj_record"):
            try:
                self.exp_logger.start_traj_record()
            except Exception as e:
                rospy.logwarn(f"[uav_navigator] start_traj_record failed: {e}")

    def _traj_record_pose(self, stamp, xyz):
        """Record one published command pose (if logger supports it)."""
        if hasattr(self.exp_logger, "record_pose"):
            try:
                self.exp_logger.record_pose(stamp, xyz)
            except Exception as e:
                rospy.logwarn(f"[uav_navigator] record_pose failed: {e}")

    def _traj_record_finish(self):
        """Finish trajectory command recording (if logger supports it)."""
        if hasattr(self.exp_logger, "finish_traj_record"):
            try:
                self.exp_logger.finish_traj_record()
            except Exception as e:
                rospy.logwarn(f"[uav_navigator] finish_traj_record failed: {e}")

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
        self._traj_record_pose(goal.header.stamp, (goal.pose.position.x, goal.pose.position.y, goal.pose.position.z))
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

                # ✅ Mission finished: stop replanning/logging and just idle
                if self._mission_done:
                    self.rate.sleep()
                    continue

                # ========== 1. 更新地图（LOS → grid_map） ==========
                self.update_map_stateful()

                # ========== 2. 更新 EDT 地图 ==========
                # ⚠️ 注意：这是“重算 edt_map”，不是改 grid_map
                self.grid_map.update_edt()

                if self.current_position is not None and self.grid_map.edt_map is not None:
                    gx, gy = self.grid_map.to_grid(self.current_position[0], self.current_position[1])
                    rospy.logwarn(
                        f"[DEBUG][AFTER EDT] start grid=({gx},{gy}) "
                        f"grid_val={self.grid_map.grid_map[gx, gy]} "
                        f"EDT(m)={self.grid_map.edt_map[gx, gy]}"
                    )

                self.publish_edt_map()

                # ========== 3. 更新 A* 使用的地图 ==========
                self.planner.map_reconstruct(self.grid_map)

                # =================================================
                # 状态 A：当前没有执行航点 → 允许重新规划
                # =================================================
                if self.active_goal_real is None and self.need_replan:

                    rospy.loginfo("[Navigator] Replanning...")

                    # Reset local episode guards
                    self._episode_open = False
                    self._execution_started = False
                    self._episode_note = None

                    # Start a new experiment episode when we decide to replan
                    self.exp_logger.start_episode(
                        start_real=self.start_real,
                        goal_real=self.goal_real,
                        planner=self.planner.__class__.__name__,
                    )
                    self._episode_open = True

                    # 1. 路径规划，输出栅格坐标路径
                    with self.exp_logger.timeit("planning_time"):
                        grid_path = self.plan(self.start_real, self.goal_real)

                    if not grid_path:
                        rospy.logwarn("[Navigator] No path found.")
                        self.exp_logger.finish_episode(success=False, note="no_path")
                        self._episode_open = False
                        self._execution_started = False
                        self.rate.sleep()
                        continue

                    # 2. 提取“第一段窗口”的 4 个控制点（栅格）
                    ctrl_pts_grid = self.planner.extract_first_window_ctrl_points(
                        path=grid_path,
                        min_dist_m=3.0
                    )

                    if ctrl_pts_grid is None:
                        rospy.logwarn("[Navigator] extract_first_window_ctrl_points returned None.")
                        self.exp_logger.finish_episode(success=False, note="no_ctrl_pts")
                        self._episode_open = False
                        self._execution_started = False
                        self.rate.sleep()
                        continue

                    if len(ctrl_pts_grid) < 4:
                        rospy.logwarn("[Navigator] Failed to extract 4 control points. Directly flying to goal.")
                        self.active_goal_grid = ctrl_pts_grid[-1]
                        self.active_goal_real = self.grid_map.to_real(
                            self.active_goal_grid[0],
                            self.active_goal_grid[1]
                        )

                        # Fallback: skip optimizer, fly directly to the window end.
                        # Do NOT finish the episode here — it may still succeed.
                        self._episode_note = "fallback_direct_goal"

                        self.publish_waypoint(self.active_goal_real)

                        # Start execution timing for fair comparison
                        if not self._execution_started:
                            self.exp_logger.start_execution()
                            self._traj_record_start()
                            self._execution_started = True

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

                    # 4. 轨迹优化
                    with self.exp_logger.timeit("optimization_time"):
                        self.publish_optimizer_ctrl_points(ctrl_pts_real)

                    # Execution starts once we have sent the optimizer its control points
                    if not self._execution_started:
                        self.exp_logger.start_execution()
                        self._traj_record_start()
                        self._execution_started = True

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

                        # ✅ Check whether FINAL mission goal is reached
                        final_reached = self.has_reached_goal(
                            self.goal_real[0],
                            self.goal_real[1],
                            1.0
                        )

                        if self._execution_started:
                            self.exp_logger.finish_execution()
                            self._traj_record_finish()
                            self._execution_started = False

                        note = self._episode_note
                        self.exp_logger.finish_episode(success=True, note=note)
                        self._episode_open = False
                        self._episode_note = None

                        # 记录通过的关键航点
                        self.start_real = self.active_goal_real
                        self.key_waypoints.append(self.active_goal_grid)

                        # 清空执行态
                        self.active_goal_real = None
                        self.active_goal_grid = None

                        # ✅ If final goal reached: stop everything
                        if final_reached:
                            rospy.logwarn("[Navigator] FINAL goal reached. Mission done; stop replanning/logging.")
                            self._mission_done = True
                            self.need_replan = False
                        else:
                            self.need_replan = True

                self.rate.sleep()

        finally:
            # If shutdown happens mid-episode, finalize once (avoid duplicate CSV rows)
            if self._episode_open and (not self._mission_done):
                if self._execution_started:
                    self.exp_logger.finish_execution()
                    self._traj_record_finish()
                    self._execution_started = False

                note = self._episode_note or "shutdown"
                self.exp_logger.finish_episode(success=False, note=note)
                self._episode_open = False
                self._episode_note = None

            if not self._mission_done:
                self.grid_map.visualize(self.key_waypoints)
                self.grid_map.visualize_edt(self.key_waypoints)
            # print("finished..")


if __name__ == "__main__":
    navigator = UAVNavigator()
    navigator.run()
