#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import rospy
from nav_msgs.msg import Odometry


class OdometryWaypointRecorder:
    def __init__(self):
        rospy.init_node("odometry_waypoint_recorder", anonymous=True)

        self.topic_name = rospy.get_param(
            "~topic_name",
            "/ardrone_1/odometry_sensor1/odometry"
        )

        self.min_distance = 0.5
        self.records = []
        self.last_point = None

        # 当前脚本运行目录下创建 waypoints 文件夹
        self.save_dir = os.path.join(os.getcwd(), "waypoints")
        os.makedirs(self.save_dir, exist_ok=True)

        timestamp = rospy.Time.now().to_sec()
        self.save_path = os.path.join(
            self.save_dir,
            f"waypoints_{int(timestamp)}.csv"
        )

        rospy.Subscriber(self.topic_name, Odometry, self.odom_callback)

        rospy.loginfo(f"[WaypointRecorder] Subscribe topic: {self.topic_name}")
        rospy.loginfo(f"[WaypointRecorder] Save dir: {self.save_dir}")
        rospy.loginfo(f"[WaypointRecorder] Save file: {self.save_path}")

        rospy.on_shutdown(self.save_to_file)

    def odom_callback(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        z = float(msg.pose.pose.position.z)
        t = msg.header.stamp.to_sec() if msg.header.stamp != rospy.Time() else rospy.Time.now().to_sec()

        current_point = (x, y, z)

        # 如果设置了最小距离，则只有移动足够远才记录
        if self.last_point is not None and self.min_distance > 0.0:
            dx = current_point[0] - self.last_point[0]
            dy = current_point[1] - self.last_point[1]
            dz = current_point[2] - self.last_point[2]
            dist = (dx * dx + dy * dy + dz * dz) ** 0.5
            if dist < self.min_distance:
                return

        self.records.append([t, x, y, z])
        self.last_point = current_point

        rospy.loginfo_throttle(
            1.0,
            f"[WaypointRecorder] Recorded point: x={x:.3f}, y={y:.3f}, z={z:.3f}, total={len(self.records)}"
        )

    def save_to_file(self):
        if not self.records:
            rospy.logwarn("[WaypointRecorder] No waypoint data recorded, nothing to save.")
            return

        try:
            with open(self.save_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time", "x", "y", "z"])
                writer.writerows(self.records)

            rospy.loginfo(
                f"[WaypointRecorder] Saved {len(self.records)} waypoints to {self.save_path}"
            )
        except Exception as e:
            rospy.logerr(f"[WaypointRecorder] Failed to save waypoints: {e}")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    recorder = OdometryWaypointRecorder()
    recorder.run()