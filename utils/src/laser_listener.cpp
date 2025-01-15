#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/laserscan_stamped.pb.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <string>

// 定义 UWB 锚点结构体
struct UWBAnchor {
    int id;
    double x, y, z;
};

// 加载 UWB 锚点函数
std::vector<UWBAnchor> loadUWBAnchors(const std::string &filename) {
    std::vector<UWBAnchor> anchors;
    YAML::Node config = YAML::LoadFile(filename);

    for (const auto &anchor : config["UWB_Anchors"]) {
        UWBAnchor a;
        a.id = anchor["id"].as<int>();
        a.x = anchor["x"].as<double>();
        a.y = anchor["y"].as<double>();
        a.z = anchor["z"].as<double>();
        anchors.push_back(a);
    }
    return anchors;
}

// 定义全局变量
std::vector<UWBAnchor> anchors;
double laser_x = 0.0, laser_y = 0.0, laser_z = 0.5;

// 计算激光方向与锚点方向之间的角度索引
int getLaserIndex(const UWBAnchor& anchor, const gazebo::msgs::LaserScan& scan) {
    // 计算锚点方向的水平角度
    double dx = anchor.x - laser_x;
    double dy = anchor.y - laser_y;
    double angle_to_anchor = std::atan2(dy, dx);

    // 激光扫描的角度范围和步长
    double min_angle = scan.angle_min();
    double max_angle = scan.angle_max();
    double angle_step = scan.angle_step();

    // 检查锚点是否在激光传感器的扫描范围内
    if (angle_to_anchor < min_angle || angle_to_anchor > max_angle) {
        return -1; // 不在扫描范围内
    }

    // 计算锚点对应的光线索引
    int index = static_cast<int>((angle_to_anchor - min_angle) / angle_step);
    return index;
}

// 激光回调函数
void LaserCallback(ConstLaserScanStampedPtr &msg) {
    const auto& scan = msg->scan();
    double max_range = scan.range_max();

    std::cout << "Laser scan received." << std::endl;

    // 遍历所有 UWB 锚点
    for (const auto& anchor : anchors) {
        // 获取对应锚点的激光束索引
        int laser_index = getLaserIndex(anchor, scan);
        if (laser_index == -1) {
            std::cout << "Anchor " << anchor.id << " is outside the laser scan range." << std::endl;
            continue;
        }

        // 获取激光测距值
        double laser_distance = scan.ranges(laser_index);

        // 计算激光到锚点的直线距离
        double anchor_distance = std::sqrt(std::pow(anchor.x - laser_x, 2) +
                                           std::pow(anchor.y - laser_y, 2) +
                                           std::pow(anchor.z - laser_z, 2));

        // 判断激光是否被障碍物阻挡
        if (laser_distance < anchor_distance) {
            std::cout << "Anchor " << anchor.id << ": NLOS (Obstructed). "
                      << "Laser collision distance: " << laser_distance << " meters." << std::endl;
        } else {
            std::cout << "Anchor " << anchor.id << ": LOS (No Obstruction). "
                      << "Direct distance: " << anchor_distance << " meters." << std::endl;
        }
    }
}

int main(int argc, char** argv) {
    // 初始化 Gazebo 客户端
    gazebo::client::setup(argc, argv);

    // 加载 UWB 锚点
    anchors = loadUWBAnchors("/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/UWB_Anchors.yml");

    // 创建 Gazebo 节点
    gazebo::transport::NodePtr gazeboNode(new gazebo::transport::Node());
    gazeboNode->Init();

    // 订阅激光数据
    gazebo::transport::SubscriberPtr sub = gazeboNode->Subscribe(
        "~/laser_sensor_model/link/ray_sensor/scan", LaserCallback);

    // 主循环
    while (true) {
        gazebo::common::Time::MSleep(10);
    }

    // 清理
    gazebo::client::shutdown();
    return 0;
}
