#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped


def create_goal_msg(x, y, z=1.0):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"

    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z

    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0

    return msg


def main():

    rospy.init_node("send_goal_interactive", anonymous=True)

    pub = rospy.Publisher("/uav_navigator/new_goal", PoseStamped, queue_size=1)

    rospy.sleep(0.5)

    print("\n========== UAV GOAL PUBLISHER ==========")
    print("输入目标点:  x y")
    print("例如:  10 -5")
    print("输入 q 退出")
    print("========================================\n")

    while not rospy.is_shutdown():

        try:
            user_input = input("Goal (x y): ")

            if user_input.strip().lower() == "q":
                print("Exit.")
                break

            parts = user_input.split()

            if len(parts) < 2:
                print("Format error. Example: 10 -5")
                continue

            x = float(parts[0])
            y = float(parts[1])

            msg = create_goal_msg(x, y)

            pub.publish(msg)

            rospy.loginfo(f"[send_goal] New goal published: ({x}, {y})")

        except Exception as e:
            print(f"Error: {e}")


if __name__ == "__main__":
    main()