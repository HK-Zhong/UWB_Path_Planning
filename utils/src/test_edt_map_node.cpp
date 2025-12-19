#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_edt_map_node");
  ros::NodeHandle nh;

  ros::Publisher pub =
      nh.advertise<nav_msgs::OccupancyGrid>(
          "/planning/edt_map", 1, true);

  ros::Duration(1.0).sleep();  // 等待订阅者

  nav_msgs::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 0.5;
  map.info.width = 100;
  map.info.height = 100;

  map.info.origin.position.x = -25.0;
  map.info.origin.position.y = -25.0;

  // EDT：全部设为 10 米（非常安全）
  map.data.resize(map.info.width * map.info.height);
  for (auto& v : map.data)
    v = 20;  // 20 * resolution = 10m

  pub.publish(map);

  ROS_INFO("[test_edt_map_node] Published safe EDT map.");
  ros::spin();
  return 0;
}
