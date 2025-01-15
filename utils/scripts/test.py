#!/usr/bin/env python
import yaml
import math

# 定义计算距离的函数
def calculate_distance(point1, point2):
    """
    计算两个点之间的欧几里得距离
    :param point1: 第一个点 (x1, y1, z1)
    :param point2: 第二个点 (x2, y2, z2)
    :return: 欧几里得距离
    """
    return math.sqrt(
        (point1[0] - point2[0]) ** 2 +
        (point1[1] - point2[1]) ** 2 +
        (point1[2] - point2[2]) ** 2
    )

# 加载 YAML 文件中的 UWB 锚点
def load_uwb_anchors(file_path):
    """
    加载 UWB 锚点数据
    :param file_path: YAML 文件路径
    :return: 锚点列表，每个锚点是一个字典 {'id': int, 'x': float, 'y': float, 'z': float}
    """
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data['UWB_Anchors']

# 主函数
if __name__ == "__main__":
    # 定义参考点
    reference_point = (0, 0, 0.5)

    # 加载 UWB 锚点
    anchors = load_uwb_anchors("UWB_Anchors.yml")

    # 计算每个锚点与参考点的距离
    print("计算点 (0, 0, 0.5) 与所有 UWB 锚点之间的距离：")
    for anchor in anchors:
        anchor_position = (anchor['x'], anchor['y'], anchor['z'])
        distance = calculate_distance(reference_point, anchor_position)
        print(f"Anchor ID: {anchor['id']}, Distance: {distance:.2f} meters")

