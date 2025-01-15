import rospy
from gazebo_msgs.srv import GetWorldProperties, GetModelState
from geometry_msgs.msg import Point
import yaml

def get_all_models():
    """
    获取 Gazebo 仿真环境中的所有模型名称。
    """
    rospy.wait_for_service('/gazebo/get_world_properties')
    try:
        get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        response = get_world_properties()
        print(response.model_names)
        return response.model_names
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to call get_world_properties service: {e}")
        return []

def get_model_bounding_box(model_name):
    """
    获取模型的三维位置信息，假设每个模型是一个立体障碍物。
    返回 Bounding Box 的 8 个顶点。
    """
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        response = get_model_state(model_name, '')
        position = response.pose.position

        # 假设障碍物的半径为 0.5
        size = 0.5

        # 定义 8 个顶点
        return [
            Point(position.x - size, position.y - size, position.z - size),  # (x_min, y_min, z_min)
            Point(position.x + size, position.y - size, position.z - size),  # (x_max, y_min, z_min)
            Point(position.x + size, position.y + size, position.z - size),  # (x_max, y_max, z_min)
            Point(position.x - size, position.y + size, position.z - size),  # (x_min, y_max, z_min)
            Point(position.x - size, position.y - size, position.z + size),  # (x_min, y_min, z_max)
            Point(position.x + size, position.y - size, position.z + size),  # (x_max, y_min, z_max)
            Point(position.x + size, position.y + size, position.z + size),  # (x_max, y_max, z_max)
            Point(position.x - size, position.y + size, position.z + size)   # (x_min, y_max, z_max)
        ]
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to call get_model_state service for {model_name}: {e}")
        return []

def get_obstacles_from_gazebo():
    """
    获取所有模型的位置信息并定义为障碍物。
    """
    obstacles = []
    models = get_all_models()
    for model in models:
        # 忽略地面和固定模型
        if model == "ardrone_1" or "floor" in model or "UWB_Anchors" in model:
            continue
        bounding_box = get_model_bounding_box(model)
        print(model, bounding_box)
        if bounding_box:
            obstacles.append(bounding_box)
    return obstacles

def load_uwb_anchors(file_path):
    """
    从 UWB_Anchors.yml 文件加载锚点坐标。
    """
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return [Point(anchor['x'], anchor['y'], anchor['z']) for anchor in data['UWB_Anchors']]

def sample_points(start, end, num_samples):
    """
    在起点和终点之间均匀采样若干点。
    """
    points = []
    for i in range(num_samples + 1):
        t = i / num_samples
        x = start.x + t * (end.x - start.x)
        y = start.y + t * (end.y - start.y)
        z = start.z + t * (end.z - start.z)
        points.append(Point(x, y, z))
    return points

def point_in_bounding_box(point, bounding_box):
    """
    检查点是否在三维 Bounding Box 内。
    参数:
        point: 待检测点 (Point)
        bounding_box: 障碍物的 Bounding Box (List[Point], 8 个顶点)
    返回:
        是否在 Bounding Box 内 (bool)
    """
    x_min = min(p.x for p in bounding_box)
    x_max = max(p.x for p in bounding_box)
    y_min = min(p.y for p in bounding_box)
    y_max = max(p.y for p in bounding_box)
    z_min = min(p.z for p in bounding_box)
    z_max = max(p.z for p in bounding_box)

    return (x_min <= point.x <= x_max) and (y_min <= point.y <= y_max) and (z_min <= point.z <= z_max)


def line_segment_nlos_detection(start, end, obstacles, num_samples=10):
    """
    检测线段上的点是否落在任何障碍物的 Bounding Box 内。
    """
    sampled_points = sample_points(start, end, num_samples)
    for point in sampled_points:
        for bounding_box in obstacles:
            if point_in_bounding_box(point, bounding_box):
                return True  # NLOS
    return False  # LOS

def check_los_to_anchors_with_sampling(fixed_point, anchors, obstacles, num_samples=10):
    """
    检测固定点到所有 UWB 锚点的 LOS，使用采样点检测。
    """
    los_results = []
    for anchor in anchors:
        is_nlos = line_segment_nlos_detection(fixed_point, anchor, obstacles, num_samples)
        has_los = not is_nlos
        los_results.append((anchor, has_los))
    return los_results

if __name__ == '__main__':
    rospy.init_node('los_detection_with_sampling')

    # 固定起点 (0, 0, 0)
    fixed_point = Point(0, 0, 0)

    # 加载 UWB 锚点
    uwb_anchors_file = '/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/UWB_Anchors.yml'
    uwb_anchors = load_uwb_anchors(uwb_anchors_file)

    # 获取障碍物
    obstacles = get_obstacles_from_gazebo()

    # 检测 LOS
    results = check_los_to_anchors_with_sampling(fixed_point, uwb_anchors, obstacles, num_samples=20)

    # 输出结果
    for anchor, has_los in results:
        status = "LOS" if has_los else "NLOS"
        rospy.loginfo(f"固定点到锚点 ({anchor.x}, {anchor.y}, {anchor.z}) 的结果: {status}")
