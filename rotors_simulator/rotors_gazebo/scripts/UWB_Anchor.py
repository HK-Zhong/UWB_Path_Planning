#!/usr/bin/env python3

import rospy
import yaml
from geometry_msgs.msg import PointStamped

def load_anchors(file_path):
    """
    加载 UWB 锚点配置文件，解析每个锚点的 x, y, z 坐标
    :param file_path: UWB_Anchors.yml 的路径
    :return: 锚点列表，包含每个锚点的 id 和 xyz 坐标
    """
    with open(file_path, 'r') as file:
        anchors = yaml.safe_load(file)
    return anchors

def publish_anchor_data(anchor_id, x, y, z, rate_hz):
    """
    创建 ROS 发布者，发布锚点数据
    :param anchor_id: 锚点 ID
    :param x, y, z: 锚点的 x, y, z 坐标
    :param rate_hz: 发布频率
    """
    topic_name = f"/uwb_anchor_{anchor_id}"  # 每个锚点独立的话题
    pub = rospy.Publisher(topic_name, PointStamped, queue_size=10)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = f"anchor_{anchor_id}"  # 使用锚点 ID 作为 frame ID
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = z

        pub.publish(point_msg)
        rate.sleep()

def main():
    rospy.init_node("uwb_anchors_simulator", anonymous=True)

    # 加载 UWB 锚点配置文件
    config_file_path = rospy.get_param("~config_file", "UWB_Anchors.yml")
    rate_hz = rospy.get_param("~publish_rate", 1)  # 默认发布频率为 1Hz

    anchors = load_anchors(config_file_path)

    # 为每个锚点创建一个独立的发布者
    for anchor in anchors:
        anchor_id = anchor["id"]
        x, y, z = anchor["x"], anchor["y"], anchor["z"]

        rospy.loginfo(f"Initializing UWB anchor {anchor_id} at ({x}, {y}, {z})")
        
        # 创建多线程发布，每个锚点一个线程
        rospy.Timer(
            rospy.Duration(1.0 / rate_hz),
            lambda event, aid=anchor_id, ax=x, ay=y, az=z: publish_anchor_data(aid, ax, ay, az, rate_hz)
        )

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
