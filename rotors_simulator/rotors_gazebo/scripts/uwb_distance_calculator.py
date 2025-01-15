#!/usr/bin/env python

import rospy
import yaml
import math
import random
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray


class UWBAnchorDistanceCalculator:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('uwb_distance_calculator', anonymous=True)

        # 订阅无人机位置
        rospy.Subscriber('/ardrone_1/odometry_sensor1/position', PointStamped, self.position_callback)

        # 发布无人机与 UWB 锚点的距离
        self.distance_pub = rospy.Publisher('/uwb_distances', Float32MultiArray, queue_size=10)

        # 无人机当前位置
        self.drone_position = None

        # 加载 UWB 锚点配置
        config_file_path = "/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/UWB_Anchors.yml"
        self.uwb_anchors = self.load_uwb_anchors(config_file_path)

        # 随机扰动参数
        self.noise_stddev = rospy.get_param("~noise_stddev", 0.5)  # 默认标准差为 0.5

        # 发布频率参数
        self.publish_frequency = rospy.get_param("~publish_frequency", 10)  # 默认发布频率为 10Hz

        rospy.loginfo("UWB Distance Calculator Initialized with noise_stddev=%.2f and publish_frequency=%d Hz",
                      self.noise_stddev, self.publish_frequency)

    def load_uwb_anchors(self, file_path):
        """
        加载 UWB 锚点配置文件
        """
        try:
            with open(file_path, 'r') as file:
                config = yaml.safe_load(file)
                rospy.loginfo("UWB Anchors loaded successfully")
                return config['UWB_Anchors']
        except Exception as e:
            rospy.logerr(f"Failed to load UWB Anchors: {e}")
            return []

    def position_callback(self, msg):
        """
        处理无人机位置消息
        """
        self.drone_position = msg.point

    def calculate_and_publish_distances(self):
        """
        计算无人机与每个 UWB 锚点的距离并发布
        """
        if self.drone_position is None:
            rospy.logwarn("Drone position is not yet available.")
            return

        distances = Float32MultiArray()

        for anchor in self.uwb_anchors:
            # 获取锚点位置
            anchor_x = anchor['x']
            anchor_y = anchor['y']
            anchor_z = anchor['z']

            # 计算欧几里得距离
            distance = math.sqrt(
                (self.drone_position.x - anchor_x) ** 2 +
                (self.drone_position.y - anchor_y) ** 2 +
                (self.drone_position.z - anchor_z) ** 2
            )

            # 添加随机扰动
            noise = random.gauss(0, self.noise_stddev)
            noisy_distance = distance + noise

            distances.data.append(noisy_distance)

        # 发布距离数据
        self.distance_pub.publish(distances)

    def run(self):
        """
        运行主循环，控制发布频率
        """
        rate = rospy.Rate(self.publish_frequency)  # 设置发布频率
        while not rospy.is_shutdown():
            if self.drone_position is not None:
                self.calculate_and_publish_distances()
            rate.sleep()


if __name__ == '__main__':
    try:
        calculator = UWBAnchorDistanceCalculator()
        calculator.run()
    except rospy.ROSInterruptException:
        pass
