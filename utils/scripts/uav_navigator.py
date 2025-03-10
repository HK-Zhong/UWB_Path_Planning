import rospy, json
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import String
from grid_map import GridMap
from path_searching import AStar

# TODO: 编写根据视距情况更新地图的代码逻辑
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
        self.los_data = {}  # 存储LOS状态

        # 订阅传感器数据
        rospy.Subscriber("/ardrone_1/odometry_sensor1/position", PointStamped, self.position_callback)
        rospy.Subscriber("/los_status_json", String, self.los_callback)
        self.rate = rospy.Rate(1)  # 控制更新频率

        # 初始化 A* 规划器
        self.a_star = AStar(self.grid_map)

    def position_callback(self, msg):
        """ 订阅无人机位置，并增量更新路径 """
        self.current_position = (msg.point.x, msg.point.y)

    def los_callback(self, msg):
        try:
            data = json.loads(msg.data)  # 解析 JSON 消息
            for d in data:
                self.los_data[d["id"]] = d["LOS"]  # 存储数据: id(int): True/False
        except json.JSONDecodeError as e:
            rospy.logerr(f"解析 JSON 失败: {e}")

    def find_path(self, start, end):
        """ 使用 A* 计算路径 """
        return self.a_star.find_path(start, end)

    def run(self):
        """ 持续进行路径规划 """
        while not rospy.is_shutdown():
            
            path = self.find_path()
            
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
