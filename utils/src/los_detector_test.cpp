#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <yaml-cpp/yaml.h>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>

// 加载 UWB 锚点配置
std::vector<ignition::math::Vector3d> LoadUWBAnchors(const std::string &filePath) {
    std::vector<ignition::math::Vector3d> anchors;
    YAML::Node config = YAML::LoadFile(filePath);
    if (config["UWB_Anchors"]) {
        for (const auto &anchor : config["UWB_Anchors"]) {
            double x = anchor["x"].as<double>();
            double y = anchor["y"].as<double>();
            double z = anchor["z"].as<double>();
            anchors.emplace_back(x, y, z);
        }
    }
    return anchors;
}

// 障碍物信息回调函数
void ObstacleCallback(const boost::shared_ptr<const gazebo::msgs::Model> &msg,
                      std::vector<std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>> &obstacles) {
    for (int i = 0; i < msg->link_size(); ++i) {
        const auto &link = msg->link(i);
        ignition::math::Vector3d position(link.pose().position().x(),
                                          link.pose().position().y(),
                                          link.pose().position().z());
        ignition::math::Vector3d size(link.collision(0).geometry().box().size().x(),
                                      link.collision(0).geometry().box().size().y(),
                                      link.collision(0).geometry().box().size().z());
        obstacles.emplace_back(position, size);
    }
}

// 获取障碍物信息
std::vector<std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>> GetObstacles(gazebo::transport::NodePtr node) {
    std::vector<std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>> obstacles;

    node->Subscribe("~/obstacles/info", 
                    boost::bind(ObstacleCallback, _1, boost::ref(obstacles)));

    std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待数据
    return obstacles;
}

// 检测直线与 AABB 的相交
bool LineIntersectsAABB(const ignition::math::Vector3d &start, 
                        const ignition::math::Vector3d &end, 
                        const ignition::math::Vector3d &boxCenter, 
                        const ignition::math::Vector3d &boxSize) {
    ignition::math::Vector3d boxMin = boxCenter - boxSize / 2.0;
    ignition::math::Vector3d boxMax = boxCenter + boxSize / 2.0;

    double tmin = (boxMin.X() - start.X()) / (end.X() - start.X());
    double tmax = (boxMax.X() - start.X()) / (end.X() - start.X());
    if (tmin > tmax) std::swap(tmin, tmax);

    double tymin = (boxMin.Y() - start.Y()) / (end.Y() - start.Y());
    double tymax = (boxMax.Y() - start.Y()) / (end.Y() - start.Y());
    if (tymin > tymax) std::swap(tymin, tymax);

    if ((tmin > tymax) || (tymin > tmax))
        return false;

    if (tymin > tmin)
        tmin = tymin;
    if (tymax < tmax)
        tmax = tymax;

    double tzmin = (boxMin.Z() - start.Z()) / (end.Z() - start.Z());
    double tzmax = (boxMax.Z() - start.Z()) / (end.Z() - start.Z());
    if (tzmin > tzmax) std::swap(tzmin, tzmax);

    if ((tmin > tzmax) || (tzmin > tmax))
        return false;

    return true;
}

// 检测两点间的 LOS
bool CheckLOS(const ignition::math::Vector3d &start, 
              const ignition::math::Vector3d &end, 
              const std::vector<std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>> &obstacles) {
    for (const auto &obstacle : obstacles) {
        const auto &boxCenter = std::get<0>(obstacle);
        const auto &boxSize = std::get<1>(obstacle);
        if (LineIntersectsAABB(start, end, boxCenter, boxSize)) {
            return false;  // 被遮挡
        }
    }
    return true;  // 无障碍物遮挡
}

int main(int argc, char **argv) {
    // 初始化 Gazebo 节点
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // 获取障碍物信息
    auto obstacles = GetObstacles(node);

    // 加载 UWB 锚点
    std::string configPath = "/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/UWB_Anchors.yml";
    auto uwbAnchors = LoadUWBAnchors(configPath);

    // 无人机位置
    ignition::math::Vector3d dronePosition(0, 0, 0);

    while (true) {
        for (size_t i = 0; i < uwbAnchors.size(); ++i) {
            bool isLOS = CheckLOS(uwbAnchors[i], dronePosition, obstacles);
            std::cout << "Anchor " << i << " LOS: " << (isLOS ? "Yes" : "No") << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::seconds(2)); // 每 2 秒检测一次
    }

    return 0;
}
