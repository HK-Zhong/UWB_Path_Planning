#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/OccupancyGrid.h>

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include <tf/tf.h>

static constexpr double CMD_DT = 0.02;   // 50Hz
static constexpr double SAMPLE_DT = 0.05;

class PiecewiseCubicSpline
{
public:
  PiecewiseCubicSpline(const std::vector<Eigen::Vector3d>& pts, double speed_mps)
      : pts_(pts), speed_(std::max(1e-3, speed_mps))
  {
    computeTangents();
  }

  double duration() const
  {
    return (speed_ > 1e-6) ? (total_length_ / speed_) : 0.0;
  }

  Eigen::Vector3d evaluate(double t) const
  {
    if (pts_.size() < 2) return pts_.front();

    double s = t * speed_;
    s = std::min(s, total_length_);

    // 找到段
    int seg = 0;
    while (seg < seg_length_prefix_.size() &&
           seg_length_prefix_[seg] < s)
      seg++;

    if (seg >= pts_.size() - 1)
      seg = pts_.size() - 2;

    double s0 = (seg == 0) ? 0.0 : seg_length_prefix_[seg - 1];
    double ds = seg_length_[seg];
    double u = (ds > 1e-6) ? (s - s0) / ds : 0.0;

    return hermite(seg, u);
  }

private:
  std::vector<Eigen::Vector3d> pts_;
  double speed_ = 1.0;  // m/s
  std::vector<Eigen::Vector3d> tangents_;
  std::vector<double> seg_length_;
  std::vector<double> seg_length_prefix_;
  double total_length_ = 0.0;

  void computeTangents()
  {
    int n = pts_.size();
    tangents_.resize(n);

    // Natural cubic style tangent approximation
    for (int i = 0; i < n; ++i)
    {
      if (i == 0)
        tangents_[i] = pts_[1] - pts_[0];
      else if (i == n - 1)
        tangents_[i] = pts_[n - 1] - pts_[n - 2];
      else
        tangents_[i] = 0.5 * (pts_[i + 1] - pts_[i - 1]);
    }

    // 计算段长度
    total_length_ = 0.0;
    seg_length_.resize(n - 1);
    seg_length_prefix_.resize(n - 1);

    for (int i = 0; i < n - 1; ++i)
    {
      double d = (pts_[i + 1] - pts_[i]).norm();
      seg_length_[i] = d;
      total_length_ += d;
      seg_length_prefix_[i] = total_length_;
    }
  }

  Eigen::Vector3d hermite(int i, double u) const
  {
    const auto& p0 = pts_[i];
    const auto& p1 = pts_[i + 1];
    const auto& m0 = tangents_[i];
    const auto& m1 = tangents_[i + 1];

    double u2 = u * u;
    double u3 = u2 * u;

    double h00 = 2*u3 - 3*u2 + 1;
    double h10 = u3 - 2*u2 + u;
    double h01 = -2*u3 + 3*u2;
    double h11 = u3 - u2;

    return h00*p0 + h10*m0 + h01*p1 + h11*m1;
  }
};

class SplineOptimizerNode
{
public:
  SplineOptimizerNode()
  {
    ros::NodeHandle nh("~");

    nh.param("edt_hard_min", edt_hard_min_, 0.5);
    nh.param("speed_s_per_sec", speed_s_per_sec_, 1.5);
    nh.param("yaw_lookahead_s", yaw_lookahead_s_, 0.3);
    nh.param("ignore_while_exec", ignore_while_exec_, true);

    ctrl_sub_ = nh.subscribe(
        "/optimizer/ctrl_points", 1,
        &SplineOptimizerNode::ctrlCallback, this);

    edt_sub_ = nh.subscribe(
        "/optimizer/edt_map", 1,
        &SplineOptimizerNode::edtCallback, this);

    cmd_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
        "/ardrone_1/command/pose", 10);

    cost_pub_ = nh.advertise<geometry_msgs::Vector3>(
        "/optimizer/cost", 10);

    timer_ = nh.createTimer(
        ros::Duration(CMD_DT),
        &SplineOptimizerNode::timerCallback, this);

    ROS_INFO("[piecewise_spline_optimizer] Ready. edt_hard_min=%.3f, speed_s_per_sec=%.3f, yaw_lookahead_s=%.3f, ignore_while_exec=%s",
             edt_hard_min_, speed_s_per_sec_, yaw_lookahead_s_, ignore_while_exec_ ? "true" : "false");
  }

private:
  ros::Subscriber ctrl_sub_;
  ros::Subscriber edt_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher cost_pub_;
  ros::Timer timer_;

  nav_msgs::OccupancyGrid edt_map_;
  bool has_edt_ = false;

  std::unique_ptr<PiecewiseCubicSpline> traj_;
  ros::Time start_time_;
  bool executing_ = false;

  double edt_hard_min_;
  double speed_s_per_sec_ = 1.5;
  double yaw_lookahead_s_ = 0.3;
  bool ignore_while_exec_ = true;
  double last_yaw_ = 0.0;
  double computeYaw(double t)
  {
    if (!traj_) return last_yaw_;

    double t2 = std::min(t + yaw_lookahead_s_, traj_->duration());

    auto p_now = traj_->evaluate(t);
    auto p_fwd = traj_->evaluate(t2);

    Eigen::Vector2d d = (p_fwd - p_now).head<2>();

    if (d.norm() < 1e-3)
      return last_yaw_;

    last_yaw_ = std::atan2(d.y(), d.x());
    return last_yaw_;
  }

  void edtCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    edt_map_ = *msg;
    has_edt_ = true;
  }

  void ctrlCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
    if (msg->poses.size() < 4) return;

    if (executing_ && ignore_while_exec_)
    {
      ROS_WARN("[PiecewiseCubicSpline_optimizer] Trajectory executing, ignore ctrl points (ignore_while_exec=true).");
      return;
    }

    if (executing_ && !ignore_while_exec_)
    {
      ROS_WARN("[PiecewiseCubicSpline_optimizer] Trajectory executing, replanning with new ctrl points (ignore_while_exec=false).");
    }

    std::vector<Eigen::Vector3d> pts;
    for (auto& p : msg->poses)
      pts.emplace_back(p.position.x, p.position.y, p.position.z);

    traj_ = std::make_unique<PiecewiseCubicSpline>(pts, speed_s_per_sec_);

    if (!checkSafety())
    {
      ROS_WARN("[PiecewiseCubicSpline_optimizer] Trajectory violates EDT.");
      return;
    }

    start_time_ = ros::Time::now();
    executing_ = true;

    // Publish costs once per accepted trajectory
    publishCostsForTraj(*traj_);

    ROS_INFO("[PiecewiseCubicSpline_optimizer] Trajectory accepted.");
  }

  bool checkSafety()
  {
    if (!traj_ || !has_edt_) return false;

    double T = traj_->duration();

    for (double t = 0.0; t <= T; t += SAMPLE_DT)
    {
      auto p = traj_->evaluate(t);
      if (!isSafe(p.x(), p.y()))
        return false;
    }
    return true;
  }

  bool isSafe(double x, double y)
  {
    double ox = edt_map_.info.origin.position.x;
    double oy = edt_map_.info.origin.position.y;
    double res = edt_map_.info.resolution;

    int gx = int((x - ox) / res);
    int gy = int((y - oy) / res);

    if (gx < 0 || gy < 0 ||
        gx >= (int)edt_map_.info.width ||
        gy >= (int)edt_map_.info.height)
      return false;

    int idx = gy * edt_map_.info.width + gx;
    int raw = edt_map_.data[idx];
    if (raw < 0) return false;

    constexpr double kMaxRange = 5.0;
    double edt = std::min(100, std::max(0, raw)) * (kMaxRange / 100.0);

    return edt >= edt_hard_min_;
  }

  void publishCostsForTraj(const PiecewiseCubicSpline& c)
  {
    // Compute simple discrete costs along execution time:
    // v, a, jerk derived from finite differences of position.

    const double T = c.duration();
    if (T <= 1e-6) return;

    // Use controller dt as the sampling dt for cost.
    const double dt = CMD_DT;

    // Need enough samples for velocity/acc/jerk.
    const int N = std::max(2, int(std::ceil(T / dt)) + 1);

    double cost_v = 0.0;
    double cost_a = 0.0;
    double cost_j = 0.0;

    Eigen::Vector3d p_prev = c.evaluate(0.0);
    Eigen::Vector3d v_prev = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_prev = Eigen::Vector3d::Zero();

    for (int i = 1; i < N; ++i)
    {
      double t = std::min(T, i * dt);
      Eigen::Vector3d p = c.evaluate(t);

      Eigen::Vector3d v = (p - p_prev) / dt;
      Eigen::Vector3d a = (v - v_prev) / dt;
      Eigen::Vector3d j = (a - a_prev) / dt;

      // Integral of squared norms (scaled by dt)
      cost_v += v.squaredNorm() * dt;
      cost_a += a.squaredNorm() * dt;
      cost_j += j.squaredNorm() * dt;

      p_prev = p;
      v_prev = v;
      a_prev = a;

      if (t >= T - 1e-9) break;
    }

    geometry_msgs::Vector3 msg;
    msg.x = cost_v;
    msg.y = cost_a;
    msg.z = cost_j;
    cost_pub_.publish(msg);

    ROS_INFO_STREAM("[PiecewiseCubicSpline_optimizer][COST] cost_v=" << cost_v
                    << " cost_a=" << cost_a
                    << " cost_jerk=" << cost_j);
  }

  void timerCallback(const ros::TimerEvent&)
  {
    if (!executing_ || !traj_) return;

    double t = (ros::Time::now() - start_time_).toSec();
    if (t >= traj_->duration())
    {
      executing_ = false;
      return;
    }

    auto p = traj_->evaluate(t);

    double yaw = computeYaw(t);

    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    geometry_msgs::PoseStamped cmd;
    cmd.header.stamp = ros::Time::now();
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
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "piecewise_cubic_spline_optimizer_node");
  SplineOptimizerNode node;
  ros::spin();
  return 0;
}