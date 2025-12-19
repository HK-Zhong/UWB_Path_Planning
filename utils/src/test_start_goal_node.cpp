#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_start_goal_node");
  ros::NodeHandle nh;

  ros::Publisher pub =
      nh.advertise<geometry_msgs::PoseArray>(
          "/planning/start_goal", 1, true);

  ros::Duration(1.0).sleep();  // 等待订阅者

  geometry_msgs::PoseArray msg;
  msg.header.frame_id = "map";

  geometry_msgs::Pose start, goal;

  // 起点
  start.position.x = -5.0;
  start.position.y = -5.0;
  start.position.z = 1.0;

  // 终点
  goal.position.x = 5.0;
  goal.position.y = 5.0;
  goal.position.z = 1.0;

  msg.poses.push_back(start);
  msg.poses.push_back(goal);

  pub.publish(msg);

  ROS_INFO("[test_start_goal_node] Published start & goal.");
  ros::spin();
  return 0;
}
