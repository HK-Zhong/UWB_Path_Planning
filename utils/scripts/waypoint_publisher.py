#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class DroneController:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('drone_controller', anonymous=True)

        # 当前无人机的位置信息
        self.current_position = None

        # 订阅无人机位置话题
        rospy.Subscriber('/ardrone_1/odometry_sensor1/odometry', Odometry, self.odometry_callback)

        # 发布无人机目标位置的 PoseStamped 消息
        self.goal_pub = rospy.Publisher('/ardrone_1/move_base_simple/goal', PoseStamped, queue_size=10)

        rospy.loginfo("Drone Controller Initialized")

    def odometry_callback(self, msg):
        """
        回调函数，接收并保存无人机当前位置。
        """
        # 从 Odometry 消息中提取位置
        self.current_position = msg.pose.pose
        rospy.loginfo(f"Current Position -> x: {self.current_position.position.x}, "
                      f"y: {self.current_position.position.y}, "
                      f"z: {self.current_position.position.z}")

    def move_to_goal(self, x, y, z):
        """
        发布目标位置，控制无人机移动。
        """
        if self.current_position is None:
            rospy.logwarn("Current position is not yet available. Waiting for odometry...")
            return

        # 构造目标位置的 PoseStamped 消息
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"  # 确保使用正确的参考系
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z

        # 方向四元数设置为无旋转
        # goal.pose.orientation.x = 0.0
        # goal.pose.orientation.y = 0.0
        # goal.pose.orientation.z = 0.0
        # goal.pose.orientation.w = 1.0

        # 发布目标位置
        self.goal_pub.publish(goal)
        rospy.loginfo(f"Goal Published -> x: {x}, y: {y}, z: {z}")

    def run(self):
        """
        主循环，用于发布目标位置。
        """
        rate = rospy.Rate(1)  # 1Hz 发布频率

        while not rospy.is_shutdown():
            # 示例：将无人机移动到 (5, 5, 2) 位置
            self.move_to_goal(-5.0, 5.0, 2.0)
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass