import rospy
import math
import json
from gazebo_msgs.srv import GetWorldProperties, GetModelState
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
import yaml

class LOSDetector:
    def __init__(self, rate):
        """
        初始化 LOS 检测类，加载 UWB 锚点并订阅无人机位置。
        """
        rospy.init_node('los_detector')
        
        self.uwb_anchors_file = '/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/UWB_Anchors.yml'
        self.drone_topic = '/ardrone_1/odometry_sensor1/position'
        self.current_drone_position = Point(0, 0, 0)  # 无人机当前位置
        
        self.rate = rospy.Rate(rate)

        # 订阅无人机位置
        rospy.Subscriber(self.drone_topic, PointStamped, self.drone_position_callback)

        # 读取 UWB 锚点
        self.uwb_anchors = self.load_uwb_anchors()

        # 获取 Gazebo 障碍物
        self.obstacles = self.get_obstacles_from_gazebo()
        
        # 创建 ROS 发布者，发布 JSON 格式的消息
        self.los_pub = rospy.Publisher('/los_status_json', String, queue_size=10)

    def drone_position_callback(self, msg):
        """
        订阅无人机位置的回调函数，实时更新位置。
        """
        self.current_drone_position = msg.point

    def get_all_models(self):
        """
        获取 Gazebo 仿真环境中的所有模型名称。
        """
        rospy.wait_for_service('/gazebo/get_world_properties')
        try:
            get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            response = get_world_properties()
            rospy.loginfo(f"model names: {response.model_names}")
            return response.model_names
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call get_world_properties service: {e}")
            return []

    def get_model_bounding_box(self, model_name):
        """
        获取模型的三维位置信息，假设每个模型是一个立体障碍物，返回 Bounding Box 的 8 个顶点。
        """
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            response = get_model_state(model_name, '')
            position = response.pose.position
            if model_name == "obstacle3":
                size_x = 2
                size_y = 13
                size_z = 2
            else:
                size_x = 5  # 假设障碍物的半径
                size_y = 3
                size_z = 2
            # 定义 8 个顶点
            return [
                Point(position.x - size_x, position.y - size_y, position.z - size_z),
                Point(position.x + size_x, position.y - size_y, position.z - size_z),
                Point(position.x + size_x, position.y + size_y, position.z - size_z),
                Point(position.x - size_x, position.y + size_y, position.z - size_z),
                Point(position.x - size_x, position.y - size_y, position.z + size_z),
                Point(position.x + size_x, position.y - size_y, position.z + size_z),
                Point(position.x + size_x, position.y + size_y, position.z + size_z),
                Point(position.x - size_x, position.y + size_y, position.z + size_z)
            ]
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call get_model_state service for {model_name}: {e}")
            return []

    def get_obstacles_from_gazebo(self):
        """
        获取所有模型的位置信息并定义为障碍物。
        """
        obstacles = []
        models = self.get_all_models()
        for model in models:
            if model == "ardrone_1" or model == "floor":
                continue
            bounding_box = self.get_model_bounding_box(model)
            if bounding_box:
                obstacles.append(bounding_box)
                
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

    def sample_points(self, start, end, step_size=0.5):
        """
        按固定步长 0.5m 在起点和终点之间均匀采样。
        参数:
            start: 起点 (Point)
            end: 终点 (Point)
            step_size: 采样步长 (默认为 0.5m)
        返回:
            采样点列表 (List[Point])
        """
        points = []
        
        # 计算线段长度
        dx = end.x - start.x
        dy = end.y - start.y
        dz = end.z - start.z
        length = math.sqrt(dx**2 + dy**2 + dz**2)

        # 计算需要采样的点数
        num_samples = int(length // step_size)

        # 生成等间距的点
        for i in range(num_samples + 1):  # 包括终点
            t = i * step_size / length if length > 0 else 0
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

    def check_los_to_anchors(self, step_size):
        """
        检测无人机当前位置到所有 UWB 锚点的 LOS，使用采样点检测。
        """
        los_results = []

        for anchor in self.uwb_anchors:
            anchor_id = anchor["id"]
            anchor_position = anchor["position"]

            is_nlos = self.line_segment_nlos_detection(self.current_drone_position, anchor_position, step_size)
            has_los = not is_nlos

            los_results.append({
                "id": anchor_id,
                "LOS": has_los  # True 表示 LOS，False 表示 NLOS
            })
        
        return los_results

    def run(self):
        """
        运行 LOS 检测循环，并持续输出结果。
        """
        while not rospy.is_shutdown():
            results = self.check_los_to_anchors(step_size=0.2)
            # 转换为 JSON 格式并发布
            los_json_msg = json.dumps(results)
            self.los_pub.publish(los_json_msg)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # 初始化 LOS 检测器
        los_detector = LOSDetector(rate=1)
        # 运行 LOS 检测
        los_detector.run()
    except rospy.ROSInterruptException:
        pass
