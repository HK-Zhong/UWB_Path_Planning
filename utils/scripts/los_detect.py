import rospy
import math
import json
from gazebo_msgs.srv import GetWorldProperties, GetModelState
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
import yaml


class LOSDetector:
    def __init__(self, rate, init_ros_node=True, enable_ros_io=True, map_no=None):
        """
        初始化 LOS 检测类，加载 UWB 锚点并订阅无人机位置。
        """
        if init_ros_node:
            rospy.init_node('los_detector', anonymous=True)

        # map_no supports explicit injection from caller (e.g., UAVNavigator).
        # If not provided, fall back to this node's private ROS param.
        if map_no is None:
            self.map_no = rospy.get_param("~map_no", 1)
        else:
            self.map_no = int(map_no)

        if self.map_no == 1:
            self.uwb_anchors_file = '/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/UWB_Anchors.yml'
            self.obstacle_file = '/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/indoor_environment1_obstacle.yml'

        elif self.map_no == 2:
            self.uwb_anchors_file = '/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/UWB_Anchors2.yml'
            self.obstacle_file = '/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/indoor_environment2_obstacle.yml'

        elif self.map_no == 3:
            self.uwb_anchors_file = '/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/UWB_Anchors3.yml'
            self.obstacle_file = '/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/indoor_environment3_obstacle.yml'

        self.drone_topic = '/ardrone_1/odometry_sensor1/position'
        self.current_drone_position = Point(0, 0, 0)  # 无人机当前位置

        self.rate = rospy.Rate(rate)

        self.step_size = rospy.get_param('~step_size', 0.2)
        self.wait_pose_timeout = rospy.get_param('~wait_pose_timeout', 5.0)
        self._have_pose = False

        try:
            self.obstacles = self.load_obstacles_from_yaml()
        except Exception as e:
            rospy.logwarn(f"[LOSDetector] Failed to load obstacles from YAML: {e}")
            self.obstacles = []

        # 读取 UWB 锚点
        self.uwb_anchors = self.load_uwb_anchors()

        # ROS I/O 只在独立节点模式下启用；被 Navigator 复用时可关闭
        self.enable_ros_io = enable_ros_io
        if self.enable_ros_io:
            # 订阅无人机位置
            rospy.Subscriber(self.drone_topic, PointStamped, self.drone_position_callback)
            # 创建 ROS 发布者，发布 JSON 格式的消息
            self.los_pub = rospy.Publisher('/los_status_json', String, queue_size=10)
        else:
            self.los_pub = None

    def drone_position_callback(self, msg):
        """
        订阅无人机位置的回调函数，实时更新位置。
        """
        self.current_drone_position = msg.point
        self._have_pose = True

    def obstacle_to_bounding_box(self, cx, cy, cz, sx, sy, sz):
        """
        根据中心点(cx, cy, cz)和尺寸(sx, sy, sz)生成三维 Bounding Box 的 8 个顶点。
        """
        hx = sx / 2.0
        hy = sy / 2.0
        hz = sz / 2.0
        return [
            Point(cx - hx, cy - hy, cz - hz),
            Point(cx + hx, cy - hy, cz - hz),
            Point(cx + hx, cy + hy, cz - hz),
            Point(cx - hx, cy + hy, cz - hz),
            Point(cx - hx, cy - hy, cz + hz),
            Point(cx + hx, cy - hy, cz + hz),
            Point(cx + hx, cy + hy, cz + hz),
            Point(cx - hx, cy + hy, cz + hz),
        ]

    def load_obstacles_from_yaml(self):
        """
        从 obstacle YAML 文件加载障碍物，并转换为 Bounding Box 列表。

        YAML 格式示例：
        obstacles:
          - name: obstacle1
            cx: -13.0
            cy: -7.0
            cz: 2.0
            sx: 10.0
            sy: 6.0
            sz: 4.0
        """
        with open(self.obstacle_file, 'r') as file:
            data = yaml.safe_load(file)

        obstacles = []
        for obs in data.get('obstacles', []):
            cx = float(obs['cx'])
            cy = float(obs['cy'])
            cz = float(obs['cz'])
            sx = float(obs['sx'])
            sy = float(obs['sy'])
            sz = float(obs['sz'])
            obstacles.append(self.obstacle_to_bounding_box(cx, cy, cz, sx, sy, sz))

        rospy.loginfo(f"[LOSDetector] Loaded {len(obstacles)} obstacles from {self.obstacle_file}")
        return obstacles

    def load_uwb_anchors(self):
        """
        从 UWB_Anchors.yml 文件加载锚点坐标。
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

    def sample_points(self, start, end, step_size=0.2):
        """
        按固定步长 0.2m 在起点和终点之间均匀采样。
        参数:
            start: 起点 (Point)
            end: 终点 (Point)
            step_size: 采样步长 (默认为 0.2m)
        返回:
            采样点列表 (List[Point])
        """
        points = []

        # 计算线段长度
        dx = end.x - start.x
        dy = end.y - start.y
        dz = end.z - start.z
        length = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        if length < 1e-6:
            return [Point(start.x, start.y, start.z), Point(end.x, end.y, end.z)]

        num_samples = max(1, int(math.ceil(length / step_size)))

        for i in range(num_samples + 1):  # 包括终点
            t = float(i) / float(num_samples)
            x = start.x + t * dx
            y = start.y + t * dy
            z = start.z + t * dz
            points.append(Point(x, y, z))
        return points

    def point_in_bounding_box(self, point, bounding_box):
        """
        检查点是否在三维 Bounding Box 内。
        """
        x_min = min(p.x for p in bounding_box)
        x_max = max(p.x for p in bounding_box)
        y_min = min(p.y for p in bounding_box)
        y_max = max(p.y for p in bounding_box)
        z_min = min(p.z for p in bounding_box)
        z_max = max(p.z for p in bounding_box)
        return (x_min <= point.x <= x_max) and (y_min <= point.y <= y_max) and (z_min <= point.z <= z_max)

    def line_segment_nlos_detection(self, start, end, step_size):
        """
        检测线段上的点是否落在任何障碍物的 Bounding Box 内。
        """
        sampled_points = self.sample_points(start, end, step_size)
        for point in sampled_points:
            for bounding_box in self.obstacles:
                if self.point_in_bounding_box(point, bounding_box):
                    return True  # NLOS
        return False  # LOS

    def check_point_los_to_anchors(self, point, step_size=None):
        """
        检测任意给定点到所有 UWB 锚点的 LOS，使用采样点检测。

        参数:
            point: geometry_msgs/Point，待检测的任意空间点
            step_size: 采样步长；若为 None，则使用 self.step_size
        返回:
            [{"id": anchor_id, "LOS": bool}, ...]
        """
        if step_size is None:
            step_size = self.step_size

        los_results = []

        for anchor in self.uwb_anchors:
            anchor_id = anchor["id"]
            anchor_position = anchor["position"]

            is_nlos = self.line_segment_nlos_detection(point, anchor_position, step_size)
            has_los = not is_nlos

            los_results.append({
                "id": anchor_id,
                "LOS": has_los  # True 表示 LOS，False 表示 NLOS
            })

        return los_results

    def check_los_to_anchors(self, step_size):
        """
        检测无人机当前位置到所有 UWB 锚点的 LOS，使用采样点检测。
        这是对 `check_point_los_to_anchors` 的一个封装。
        """
        return self.check_point_los_to_anchors(self.current_drone_position, step_size)

    def run(self):
        """
        运行 LOS 检测循环，并持续输出结果。
        """
        if not self.enable_ros_io:
            rospy.logwarn("[LOSDetector] run() called with ROS I/O disabled. Nothing will be published.")
            return
        while not rospy.is_shutdown():
            if not self._have_pose:
                rospy.logwarn_throttle(5.0, "[LOSDetector] Waiting for drone position...")
                self.rate.sleep()
                continue
            if not self.uwb_anchors:
                rospy.logerr_throttle(5.0, "[LOSDetector] No UWB anchors loaded!")
                self.rate.sleep()
                continue

            results = self.check_los_to_anchors(step_size=self.step_size)
            # 转换为 JSON 格式并发布
            los_json_msg = json.dumps(results)
            self.los_pub.publish(los_json_msg)

            rospy.loginfo_throttle(2.0, los_json_msg)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        # 初始化 LOS 检测器
        los_detector = LOSDetector(rate=2)
        # 运行 LOS 检测
        los_detector.run()
    except rospy.ROSInterruptException:
        pass
