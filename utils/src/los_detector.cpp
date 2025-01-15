#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <yaml-cpp/yaml.h>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>

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

bool CheckLOS(const ignition::math::Vector3d &start, const ignition::math::Vector3d &end, gazebo::transport::NodePtr node) {
    // 发布 Ray Cast 请求的 topic
    auto requestPub = node->Advertise<gazebo::msgs::Request>("~/ray/cast");
    // 响应的 topic
    std::string responseTopic = "~/ray/response";

    // 创建请求消息
    gazebo::msgs::Request req;
    req.set_id(gazebo::msgs::Request::NewId());
    req.set_request("ray_query");
    std::ostringstream data;
    data << start.X() << " " << start.Y() << " " << start.Z() << " "
         << end.X() << " " << end.Y() << " " << end.Z();
    req.set_data(data.str());

    // 用于接收响应的标志
    bool los = false;
    bool receivedResponse = false;

    // 设置回调函数，处理响应
    auto responseSub = node->Subscribe(responseTopic, 
        [&los, &receivedResponse](const boost::shared_ptr<const gazebo::msgs::Response> &msg) {
            if (msg->response() == "ray_query" && !msg->serialized_data().empty()) {
                // 如果 `serialized_data` 非空，说明有障碍物
                los = false;
            } else {
                // 如果 `serialized_data` 为空，说明无障碍物
                los = true;
            }
            receivedResponse = true;
        }
    );

    // 发布请求
    requestPub->Publish(req);

    // 等待响应
    int timeout = 50; // 最大等待时间 50 次循环
    while (!receivedResponse && timeout > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 每次等待 100 ms
        timeout--;
    }

    if (timeout == 0) {
        std::cerr << "Timeout while waiting for LOS response." << std::endl;
    }

    return los;
}

int main(int argc, char **argv) {
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // 加载UWB锚点配置
    std::string configPath = "/home/coolas-fly/UWB_Path_Planning/src/UWB_Path_Planning/utils/config/UWB_Anchors.yml";
    auto uwbAnchors = LoadUWBAnchors(configPath);

    // 模拟无人机位置订阅（实际需要订阅Gazebo的Pose话题）
    ignition::math::Vector3d dronePosition(0, 0, 0);

    while (true) {
        for (size_t i = 0; i < uwbAnchors.size(); ++i) {
            bool isLOS = CheckLOS(uwbAnchors[i], dronePosition, node);
            std::cout << "Anchor " << i << " LOS: " << (isLOS ? "Yes" : "No") << std::endl;

            gazebo::msgs::Any msg;
            msg.set_type(gazebo::msgs::Any::BOOLEAN);
            msg.set_bool_value(isLOS);

            std::string topicName = "/uwb_anchor_" + std::to_string(i) + "/los";
            std::cout << 1 << std::endl;
            auto publisher = node->Advertise<gazebo::msgs::Any>(topicName);
            std::cout << 2 << std::endl;
            publisher->Publish(msg);
        }

        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    return 0;
}
