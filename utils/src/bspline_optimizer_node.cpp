#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <memory>

// ✅ 新增：用于 yaw → quaternion
#include <tf/tf.h>

/**
 * ===========================
 * 参数配置
 * ===========================
 */

// 控制指令发布周期（50Hz）
static constexpr double CMD_DT = 0.02;

// B-spline 控制点时间间隔
static constexpr double BSPLINE_DT = 0.1;

// 轨迹安全检测采样时间
static constexpr double SAMPLE_DT = 0.05;

// cubic B-spline（这里仅用于说明，简化实现不显式使用该常量）
static constexpr int BSPLINE_ORDER = 3;

/**
 * =========================================================
 * 简化版 Uniform B-spline
 * 只负责：时间 → 位置
 * =========================================================
 */
class UniformBspline
{
public:
  UniformBspline(const std::vector<Eigen::Vector3d>& ctrl_pts, double dt)
    : pts_(ctrl_pts), dt_(dt) {}

  Eigen::Vector3d evaluate(double t) const
  {
    if (pts_.empty()) return Eigen::Vector3d::Zero();
    if (pts_.size() < 4) return pts_.front();

    int seg = std::min(std::max(int(t / dt_), 0), int(pts_.size()) - 4);

    double u = (t - seg * dt_) / dt_;
    u = std::min(std::max(u, 0.0), 1.0);

    const Eigen::Vector3d& p0 = pts_[seg];
    const Eigen::Vector3d& p1 = pts_[seg + 1];
    const Eigen::Vector3d& p2 = pts_[seg + 2];
    const Eigen::Vector3d& p3 = pts_[seg + 3];

    double u2 = u * u;
    double u3 = u2 * u;

    double b0 = (1 - 3*u + 3*u2 - u3) / 6.0;
    double b1 = (4 - 6*u2 + 3*u3) / 6.0;
    double b2 = (1 + 3*u + 3*u2 - 3*u3) / 6.0;
    double b3 = u3 / 6.0;

    return b0*p0 + b1*p1 + b2*p2 + b3*p3;
  }

  double duration() const
  {
    if (pts_.size() < 4) return 0.0;
    return (pts_.size() - 3) * dt_;
  }

private:
  std::vector<Eigen::Vector3d> pts_;
  double dt_;
};

/**
 * =========================================================
 * Bspline Optimizer Node
 *
 * 订阅：
 *  - /bspline/start_goal   (PoseArray, start + goal)
 *  - /bspline/edt_map      (OccupancyGrid, EDT map)
 *
 * 发布：
 *  - /ardrone_1/command/pose   (PoseStamped)
 * =========================================================
 */
class BsplineOptimizerNode
{
public:
  BsplineOptimizerNode()
  {
    ros::NodeHandle nh("~");

    // ===== 参数 =====
    nh.param("edt_hard_min", edt_hard_min_, 0.5 * 1.42);

    // ✅ 新增：前视时间（用于“超前飞” yaw）
    nh.param("yaw_lookahead", yaw_lookahead_, 0.2);   // 秒，建议 0.15~0.30

    // ✅ 新增：当速度方向很小/轨迹结束时，保持上一帧 yaw
    last_yaw_ = 0.0;

    // ===== 订阅起点终点 =====
    start_goal_sub_ = nh.subscribe(
        "/bspline/start_goal", 1,
        &BsplineOptimizerNode::startGoalCallback, this);

    // ===== 订阅 EDT =====
    edt_sub_ = nh.subscribe(
        "/bspline/edt_map", 1,
        &BsplineOptimizerNode::edtCallback, this);

    // ===== 发布控制指令 =====
    cmd_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
        "/ardrone_1/command/pose", 10);

    // ===== 定时器 =====
    timer_ = nh.createTimer(
        ros::Duration(CMD_DT),
        &BsplineOptimizerNode::cmdTimer, this);

    ROS_INFO("[bspline_optimizer_node] Ready. edt_hard_min=%.2f, yaw_lookahead=%.2f",
             edt_hard_min_, yaw_lookahead_);
  }

private:
  /**
   * =====================================================
   * 接收起点 + 终点
   * =====================================================
   */
  void startGoalCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
    if (msg->poses.size() < 2) return;

    // 如果当前轨迹还在执行，直接忽略（避免频繁重置轨迹导致“抖”）
    if (has_traj_ && executing_traj_)
    {
      ROS_WARN("[bspline] Trajectory executing, ignore new start-goal.");
      return;
    }

    Eigen::Vector3d start(
        msg->poses[0].position.x,
        msg->poses[0].position.y,
        msg->poses[0].position.z);

    Eigen::Vector3d goal(
        msg->poses[1].position.x,
        msg->poses[1].position.y,
        msg->poses[1].position.z);

    buildTrajectory(start, goal);
  }

  /**
   * =====================================================
   * 接收 EDT 地图
   * =====================================================
   */
  void edtCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    edt_map_ = *msg;
    has_edt_ = true;
  }

  /**
   * =====================================================
   * 构建 B-spline 轨迹（简单插值）
   * =====================================================
   */
  void buildTrajectory(const Eigen::Vector3d& start,
                       const Eigen::Vector3d& goal)
  {
    if (!has_edt_) return;

    std::vector<Eigen::Vector3d> ctrl_pts;

    // 贴合起点（重复两次）
    ctrl_pts.push_back(start);
    ctrl_pts.push_back(start);

    int N = 6;
    for (int i = 1; i <= N; ++i)
    {
      double r = double(i) / (N + 1);
      ctrl_pts.push_back(start * (1 - r) + goal * r);
    }

    // 贴合终点（重复两次）
    ctrl_pts.push_back(goal);
    ctrl_pts.push_back(goal);

    auto candidate = std::make_unique<UniformBspline>(ctrl_pts, BSPLINE_DT);

    if (!checkTrajectorySafety(*candidate))
    {
      ROS_WARN("[bspline] Trajectory violates EDT constraint.");
      return;
    }

    traj_ = std::move(candidate);
    traj_start_time_ = ros::Time::now();
    has_traj_ = true;
    executing_traj_ = true;

    ROS_INFO("[bspline] Trajectory accepted. duration=%.2fs",
             traj_->duration());
  }

  /**
   * =====================================================
   * EDT 安全性检查
   * =====================================================
   */
  bool checkTrajectorySafety(const UniformBspline& traj)
  {
    double T = traj.duration();
    if (T <= 0.0) return false;

    for (double t = 0.0; t <= T; t += SAMPLE_DT)
    {
      Eigen::Vector3d p = traj.evaluate(t);

      int gx = int((p.x() - edt_map_.info.origin.position.x) / edt_map_.info.resolution);
      int gy = int((p.y() - edt_map_.info.origin.position.y) / edt_map_.info.resolution);

      if (gx < 0 || gy < 0 ||
          gx >= (int)edt_map_.info.width ||
          gy >= (int)edt_map_.info.height)
        return false;

      int idx = gy * edt_map_.info.width + gx;

      // ⚠️ 注意：OccupancyGrid.data 是 int8
      // 你如果是“把 EDT 直接塞进 data”，这里需要你在发布端做缩放编码
      // 这里保留原逻辑：edt = data * resolution（假设 data 存的是“格数”）
      double edt = double(edt_map_.data[idx]) * edt_map_.info.resolution;

      if (edt < edt_hard_min_) return false;
    }

    return true;
  }

  /**
   * =====================================================
   * 计算“超前飞” yaw：朝向轨迹前方
   * =====================================================
   */
  double computeYawLookAhead(double t) const
  {
    if (!traj_) return 0.0;

    double t2 = std::min(t + yaw_lookahead_, traj_->duration());

    Eigen::Vector3d p_now = traj_->evaluate(t);
    Eigen::Vector3d p_fwd = traj_->evaluate(t2);

    Eigen::Vector2d dir = (p_fwd - p_now).head<2>();

    // 如果方向太小（接近终点/停住），返回 NaN 标记，让外层保持 last_yaw
    if (dir.norm() < 1e-3) return std::numeric_limits<double>::quiet_NaN();

    return std::atan2(dir.y(), dir.x());
  }

  /**
   * =====================================================
   * 定时发布控制指令
   * =====================================================
   */
  void cmdTimer(const ros::TimerEvent&)
  {
    if (!has_traj_ || !traj_) return;

    ros::Time now = ros::Time::now();
    double t = (now - traj_start_time_).toSec();

    if (t >= traj_->duration())
    {
      t = traj_->duration();
      executing_traj_ = false;
    }

    // 位置
    Eigen::Vector3d p = traj_->evaluate(t);

    // ✅ yaw：朝向轨迹前方（超前飞）
    double yaw = computeYawLookAhead(t);
    if (std::isnan(yaw))
      yaw = last_yaw_;
    else
      last_yaw_ = yaw;

    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    geometry_msgs::PoseStamped cmd;
    cmd.header.stamp = now;
    cmd.header.frame_id = "map";
    cmd.pose.position.x = p.x();
    cmd.pose.position.y = p.y();
    cmd.pose.position.z = p.z();
    cmd.pose.orientation.x = q.x();
    cmd.pose.orientation.y = q.y();
    cmd.pose.orientation.z = q.z();
    cmd.pose.orientation.w = q.w();

    cmd_pub_.publish(cmd);
  }

private:
  // ROS
  ros::Subscriber start_goal_sub_;
  ros::Subscriber edt_sub_;
  ros::Publisher  cmd_pub_;
  ros::Timer      timer_;

  // EDT
  nav_msgs::OccupancyGrid edt_map_;
  bool has_edt_ = false;

  // Trajectory
  std::unique_ptr<UniformBspline> traj_;
  ros::Time traj_start_time_;
  bool has_traj_ = false;
  bool executing_traj_ = false;

  // Params
  double edt_hard_min_;

  // ✅ yaw params/state
  double yaw_lookahead_;
  double last_yaw_;
};

/**
 * =====================================================
 * main
 * =====================================================
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "bspline_optimizer_node");
  BsplineOptimizerNode node;
  ros::spin();
  return 0;
}
