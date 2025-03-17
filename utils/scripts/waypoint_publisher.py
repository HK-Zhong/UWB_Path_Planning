#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math

class DroneController:
    def __init__(self, waypoints, tolerance=0.1):
        # 初始化 ROS 节点
        rospy.init_node('drone_controller', anonymous=True)

        # 当前无人机的位置信息
        self.current_position = None

        # 订阅无人机位置话题
        rospy.Subscriber('/ardrone_1/odometry_sensor1/odometry', Odometry, self.odometry_callback)

        # 发布无人机目标位置的 PoseStamped 消息
        self.goal_pub = rospy.Publisher('/ardrone_1/move_base_simple/goal', PoseStamped, queue_size=10)

        # 预定义的航点列表
        self.waypoints = waypoints
        self.current_waypoint_index = 0  # 记录当前执行的航点索引
        self.tolerance = tolerance  # 目标点到达的误差范围

        rospy.loginfo("Drone Controller Initialized")

    def odometry_callback(self, msg):
        """
        回调函数，接收并保存无人机当前位置。
        """
        self.current_position = msg.pose.pose
        rospy.loginfo(f"Current Position -> x: {self.current_position.position.x}, "
                      f"y: {self.current_position.position.y}, "
                      f"z: {self.current_position.position.z}")
                    # 只有当当前位置可用时，才检查是否到达目标点

    def has_reached_goal(self, x, y, z):
        """
        检查无人机是否到达目标点
        """
        if self.current_position is None:
            return False

        dx = self.current_position.position.x - x
        dy = self.current_position.position.y - y
        # dz = self.current_position.position.z - z
        distance = math.sqrt(dx**2 + dy**2)

        return distance < self.tolerance  # 如果距离小于设定的阈值，则认为到达

    def move_to_next_waypoint(self):
        """
        发布目标位置，依次前往航点
        """
        if self.current_position is None:
            rospy.logwarn("Current position is not yet available. Waiting for odometry...")
            return

        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("All waypoints reached.")
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        x, y, z = waypoint

        # 构造目标位置的 PoseStamped 消息
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"  # 确保使用正确的参考系
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z

        # 方向四元数设置为无旋转
        # goal.pose.orientation.w = 1.0

        # 发布目标位置
        self.goal_pub.publish(goal)
        rospy.loginfo(f"Moving to Waypoint {self.current_waypoint_index} -> x: {x}, y: {y}, z: {z}")

    def run(self):
        """
        主循环，逐步前往航点。
        """
        rate = rospy.Rate(2)  # 1Hz 发布频率

        while not rospy.is_shutdown():
            
            if self.current_waypoint_index < len(self.waypoints):
                target_x, target_y, target_z = self.waypoints[self.current_waypoint_index]
                if self.has_reached_goal(target_x, target_y, target_z):
                    # rospy.loginfo(f"Waypoint {self.current_waypoint_index} reached! Moving to next waypoint.")
                    self.current_waypoint_index += 1  # 切换到下一个航点
                            
            if self.current_waypoint_index < len(self.waypoints):
                self.move_to_next_waypoint()
                
            rate.sleep()

if __name__ == '__main__':
    try:
        # 手动输入航点列表，每个航点是 (x, y, z) 坐标
        waypoints = [
            (-5, -12, 2),
            (-5, 0, 2),
            (-5, 12, 2),
            (0, 15, 2),
            (11, 13, 2),
            (4, 5, 2),
            (4, -5, 2),
            (11, -10, 2),
        ]
        
        controller = DroneController(waypoints, tolerance=0.2)
        controller.run()
    except rospy.ROSInterruptException:
        pass
