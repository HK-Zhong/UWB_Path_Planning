// ================================================================
// ego_optimizer_node.cpp
//
// 说明：
//   这是一个“简化 EGO 风格”的后端轨迹优化节点，用于替换当前的
//   centripetal_catmull_rom_optimizer_node.cpp。
//
//   它保持与你当前系统一致的输入/输出接口：
//     输入：
//       - /optimizer/ctrl_points   (geometry_msgs/PoseArray)
//       - /optimizer/edt_map       (nav_msgs/OccupancyGrid)
//     输出：
//       - /ardrone_1/command/pose  (geometry_msgs/PoseStamped)
//       - /optimizer/cost          (geometry_msgs/Vector3)
//       - /optimizer/edt           (geometry_msgs/Vector3)
//       - /optimizer/traj_info     (geometry_msgs/Vector3)
//
//   特点：
//     - 使用 Uniform Cubic B-spline 作为局部轨迹表示
//     - 接收 Navigator 给出的局部控制点序列作为参考
//     - 基于 EDT 梯度做安全引导优化（EGO 风格）
//     - yaw 前视控制
//     - 轨迹执行前先做 yaw 对齐
//     - 轨迹采样检查 EDT >= edt_hard_min
//
//   注意：
//     这不是原版 EGO-Planner 的完整复现，而是一个适配当前工程结构的
//     EGO 风格后端优化器，适合做对比实验。
// ================================================================

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/OccupancyGrid.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <tf/tf.h>

// ----------------------- 常量 -----------------------
static constexpr double CMD_DT = 0.02;  // 50 Hz
static constexpr double EPS = 1e-6;

// ================================================================
// 1) Uniform Cubic B-spline
//    - 控制点数量 >= 4 时有效
//    - 参数 u ∈ [0, segment_count)
// ================================================================
class UniformCubicBSpline
{
public:
  UniformCubicBSpline() = default;

  explicit UniformCubicBSpline(std::vector<Eigen::Vector3d> ctrl_pts)
    : ctrl_pts_(std::move(ctrl_pts)) {}

  bool valid() const { return ctrl_pts_.size() >= 4; }

  int segmentCount() const
  {
    return valid() ? static_cast<int>(ctrl_pts_.size()) - 3 : 0;
  }

  double totalU() const
  {
    return static_cast<double>(segmentCount());
  }

  const std::vector<Eigen::Vector3d>& controlPoints() const { return ctrl_pts_; }
  std::vector<Eigen::Vector3d>& controlPoints() { return ctrl_pts_; }

  Eigen::Vector3d evaluate(double u) const
  {
    if (ctrl_pts_.empty()) return Eigen::Vector3d::Zero();
    if (ctrl_pts_.size() == 1) return ctrl_pts_.front();
    if (!valid()) return ctrl_pts_.front();

    const double U = totalU();
    u = std::max(0.0, std::min(u, U - 1e-9));

    const int seg = std::min(std::max(static_cast<int>(std::floor(u)), 0), segmentCount() - 1);
    const double t = u - static_cast<double>(seg);
    const double t2 = t * t;
    const double t3 = t2 * t;

    const Eigen::Vector3d& P0 = ctrl_pts_[seg + 0];
    const Eigen::Vector3d& P1 = ctrl_pts_[seg + 1];
    const Eigen::Vector3d& P2 = ctrl_pts_[seg + 2];
    const Eigen::Vector3d& P3 = ctrl_pts_[seg + 3];

    // Uniform cubic B-spline basis
    const double b0 = (-t3 + 3.0 * t2 - 3.0 * t + 1.0) / 6.0;
    const double b1 = ( 3.0 * t3 - 6.0 * t2 + 4.0) / 6.0;
    const double b2 = (-3.0 * t3 + 3.0 * t2 + 3.0 * t + 1.0) / 6.0;
    const double b3 = ( t3 ) / 6.0;

    return b0 * P0 + b1 * P1 + b2 * P2 + b3 * P3;
  }

private:
  std::vector<Eigen::Vector3d> ctrl_pts_;
};

// ================================================================
// 2) Simplified EGO-style optimizer node
// ================================================================
class EgoOptimizerNode
{
public:
  EgoOptimizerNode()
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // ---- 参数 ----
    pnh.param("edt_hard_min", edt_hard_min_, 0.5);
    pnh.param("speed_u_per_sec", speed_u_per_sec_, 1.5);
    pnh.param("yaw_lookahead_u", yaw_lookahead_u_, 0.15);
    pnh.param("ignore_while_exec", ignore_while_exec_, true);
    pnh.param("max_yaw_rate", max_yaw_rate_, 0.6);
    pnh.param("yaw_align_tol", yaw_align_tol_, 0.20);
    pnh.param("yaw_align_timeout", yaw_align_timeout_, 2.0);

    // 优化参数
    pnh.param("opt_iters", opt_iters_, 40);
    pnh.param("sample_du", sample_du_, 0.05);
    pnh.param("step_size", step_size_, 0.08);
    pnh.param("w_smooth", w_smooth_, 1.0);
    pnh.param("w_ref", w_ref_, 1.5);
    pnh.param("w_safe", w_safe_, 2.5);
    pnh.param("safe_band", safe_band_, 0.8);
    pnh.param("ctrl_z", ctrl_z_, 1.0);
    pnh.param("max_ctrl_shift", max_ctrl_shift_, 0.3);

    // ---- 输入 ----
    ctrl_pts_sub_ = nh.subscribe(
        "/optimizer/ctrl_points", 1,
        &EgoOptimizerNode::ctrlPointsCallback, this);

    edt_sub_ = nh.subscribe(
        "/optimizer/edt_map", 1,
        &EgoOptimizerNode::edtCallback, this);

    // ---- 输出 ----
    cmd_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
        "/ardrone_1/command/pose", 10);

    cost_pub_ = nh.advertise<geometry_msgs::Vector3>(
        "/optimizer/cost", 10);

    edt_pub_ = nh.advertise<geometry_msgs::Vector3>(
        "/optimizer/edt", 10);

    traj_info_pub_ = nh.advertise<geometry_msgs::Vector3>(
        "/optimizer/traj_info", 10);

    // ---- 定时器 ----
    timer_ = nh.createTimer(
        ros::Duration(CMD_DT),
        &EgoOptimizerNode::cmdTimer, this);

    ROS_INFO("[ego_optimizer_node] Ready. edt_hard_min=%.3f, opt_iters=%d, step_size=%.3f, speed_u_per_sec=%.3f",
             edt_hard_min_, opt_iters_, step_size_, speed_u_per_sec_);
  }

private:
  // ============================================================
  // EDT 地图回调
  // ============================================================
  void edtCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    edt_map_ = *msg;
    has_edt_ = true;
  }

  // ============================================================
  // 控制点回调
  // ============================================================
  void ctrlPointsCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
    if (!has_edt_)
    {
      ROS_WARN("[ego_optimizer_node] No EDT yet.");
      return;
    }

    if (msg->poses.size() < 2)
    {
      ROS_WARN("[ego_optimizer_node] Need >= 2 reference points.");
      return;
    }

    if (ignore_while_exec_ && executing_)
    {
      ROS_WARN("[ego_optimizer_node] Executing, ignore new points.");
      return;
    }

    std::vector<Eigen::Vector3d> ref_pts;
    ref_pts.reserve(msg->poses.size());
    for (const auto& p : msg->poses)
    {
      ref_pts.emplace_back(p.position.x, p.position.y, ctrl_z_);
    }

    // 调试输出首尾点 EDT
    {
      const Eigen::Vector3d& p_start = ref_pts.front();
      const Eigen::Vector3d& p_goal  = ref_pts.back();

      double edt_s = 0.0, edt_g = 0.0;
      int raw_s = -1, raw_g = -1;
      int gsx = 0, gsy = 0, ggx = 0, ggy = 0;

      bool ok_s = queryEDT(p_start.x(), p_start.y(), edt_s, raw_s, gsx, gsy);
      bool ok_g = queryEDT(p_goal.x(),  p_goal.y(),  edt_g, raw_g, ggx, ggy);

      ROS_WARN_STREAM("[ego_optimizer_node][REF] START pos=(" << p_start.x() << "," << p_start.y()
                      << ") grid=(" << gsx << "," << gsy << ") raw_v=" << raw_s
                      << " edt_m=" << edt_s << " ok=" << ok_s);

      ROS_WARN_STREAM("[ego_optimizer_node][REF] GOAL  pos=(" << p_goal.x() << "," << p_goal.y()
                      << ") grid=(" << ggx << "," << ggy << ") raw_v=" << raw_g
                      << " edt_m=" << edt_g << " ok=" << ok_g);
    }

    // 1) 构造初始 B-spline 控制点
    std::vector<Eigen::Vector3d> ctrl_pts = buildInitialCtrlPoints(ref_pts);
    if (ctrl_pts.size() < 4)
    {
      ROS_WARN("[ego_optimizer_node] Failed to build initial control points.");
      return;
    }

    // 2) EDT 安全引导优化
    optimizeCtrlPoints(ctrl_pts);

    // 3) 构造 spline 并安全检查
    auto cand = std::make_unique<UniformCubicBSpline>(ctrl_pts);
    if (!cand->valid())
    {
      ROS_WARN("[ego_optimizer_node] Spline invalid after optimization.");
      return;
    }

    if (!checkTrajectorySafety(*cand))
    {
      ROS_WARN("[ego_optimizer_node] Optimized spline violates EDT hard constraint.");
      return;
    }

    spline_ = std::move(cand);

    // 4) yaw 对齐阶段
    hold_pos_ = spline_->evaluate(0.0);
    yaw_align_target_ = computeYawLookAhead(0.0);
    yaw_align_start_time_ = ros::Time::now();
    yaw_aligning_ = true;
    executing_ = false;

    publishCostsForSpline(*spline_);
    publishEdtStatsForSpline(*spline_);
    publishTrajInfoForSpline(*spline_);

    ROS_INFO_STREAM("[ego_optimizer_node] New local trajectory accepted. ctrl_pts="
                    << spline_->controlPoints().size() << ", totalU=" << spline_->totalU());
  }

  // ============================================================
  // 用参考点构造初始控制点
  // 这里采用简单扩边的方式：首尾重复一次
  // ============================================================
  std::vector<Eigen::Vector3d> buildInitialCtrlPoints(const std::vector<Eigen::Vector3d>& ref_pts) const
  {
    std::vector<Eigen::Vector3d> ctrl;
    if (ref_pts.empty()) return ctrl;

    if (ref_pts.size() == 2)
    {
      ctrl.push_back(ref_pts.front());
      ctrl.push_back(ref_pts.front());
      ctrl.push_back(ref_pts.back());
      ctrl.push_back(ref_pts.back());
      return ctrl;
    }

    ctrl.reserve(ref_pts.size() + 2);
    ctrl.push_back(ref_pts.front());
    for (const auto& p : ref_pts) ctrl.push_back(p);
    ctrl.push_back(ref_pts.back());
    return ctrl;
  }

  // ============================================================
  // 核心优化：简化 EGO 风格
  // 代价 = 平滑项 + 参考保持项 + EDT 安全项
  // ============================================================
  void optimizeCtrlPoints(std::vector<Eigen::Vector3d>& ctrl_pts)
  {
    if (ctrl_pts.size() < 4) return;

    const std::vector<Eigen::Vector3d> anchor_ctrl = ctrl_pts;

    for (int iter = 0; iter < opt_iters_; ++iter)
    {
      for (size_t i = 1; i + 1 < ctrl_pts.size(); ++i)
      {
        Eigen::Vector2d grad = Eigen::Vector2d::Zero();

        // ---- 1) 平滑项：二阶差分 ----
        {
          const Eigen::Vector2d p_im1(ctrl_pts[i - 1].x(), ctrl_pts[i - 1].y());
          const Eigen::Vector2d p_i  (ctrl_pts[i].x(),     ctrl_pts[i].y());
          const Eigen::Vector2d p_ip1(ctrl_pts[i + 1].x(), ctrl_pts[i + 1].y());
          grad += w_smooth_ * (2.0 * p_i - p_im1 - p_ip1);
        }

        // ---- 2) 参考保持项 ----
        {
          const Eigen::Vector2d pref(anchor_ctrl[i].x(), anchor_ctrl[i].y());
          const Eigen::Vector2d pi(ctrl_pts[i].x(), ctrl_pts[i].y());
          grad += w_ref_ * (pi - pref);
        }

        // ---- 3) EDT 安全项 ----
        {
          const double edt = queryEDTValue(ctrl_pts[i].x(), ctrl_pts[i].y());
          if (edt < safe_band_)
          {
            Eigen::Vector2d g_safe = computeEDTGradient(ctrl_pts[i].x(), ctrl_pts[i].y());
            if (g_safe.norm() > 1e-9)
            {
              // 梯度下降时，加上负号，使控制点沿“远离障碍”的方向移动
              grad += (-w_safe_ * std::max(0.0, safe_band_ - edt)) * g_safe;
            }
          }
        }

        Eigen::Vector2d pi(ctrl_pts[i].x(), ctrl_pts[i].y());
        Eigen::Vector2d new_xy = pi - step_size_ * grad;

        // 限制相对初始点的最大偏移
        {
          const Eigen::Vector2d pref(anchor_ctrl[i].x(), anchor_ctrl[i].y());
          Eigen::Vector2d delta = new_xy - pref;
          const double dn = delta.norm();
          if (dn > max_ctrl_shift_)
          {
            new_xy = pref + (max_ctrl_shift_ / std::max(dn, EPS)) * delta;
          }
        }

        ctrl_pts[i].x() = new_xy.x();
        ctrl_pts[i].y() = new_xy.y();
      }
    }
  }

  // ============================================================
  // 轨迹安全检查
  // ============================================================
  bool checkTrajectorySafety(const UniformCubicBSpline& spline) const
  {
    const double U = spline.totalU();
    if (U <= EPS) return false;

    for (double u = 0.0; u <= U + 1e-9; u += sample_du_)
    {
      Eigen::Vector3d p = spline.evaluate(u);
      if (!isSafeByEDT(p.x(), p.y()))
      {
        ROS_ERROR_STREAM("[ego_optimizer_node][SAFETY FAIL] u=" << u
                         << " pos=(" << p.x() << "," << p.y() << ")"
                         << " totalU=" << U);
        return false;
      }
    }
    return true;
  }

  // ============================================================
  // EDT 查询与梯度
  // ============================================================
  bool queryEDT(double x, double y, double& edt_m, int& raw_v, int& gx, int& gy) const
  {
    edt_m = 0.0;
    raw_v = -1;

    const double ox  = edt_map_.info.origin.position.x;
    const double oy  = edt_map_.info.origin.position.y;
    const double res = edt_map_.info.resolution;

    gx = int((x - ox) / res);
    gy = int((y - oy) / res);

    if (gx < 0 || gy < 0 || gx >= static_cast<int>(edt_map_.info.width) || gy >= static_cast<int>(edt_map_.info.height))
      return false;

    const int idx = gy * static_cast<int>(edt_map_.info.width) + gx;
    raw_v = static_cast<int>(edt_map_.data[idx]);

    if (raw_v < 0) return false;

    const int v = std::min(100, std::max(0, raw_v));
    constexpr double kMaxRangeM = 5.0;
    edt_m = static_cast<double>(v) * (kMaxRangeM / 100.0);
    return true;
  }

  double queryEDTValue(double x, double y) const
  {
    double edt = 0.0;
    int raw = -1, gx = 0, gy = 0;
    if (!queryEDT(x, y, edt, raw, gx, gy)) return 0.0;
    return edt;
  }

  Eigen::Vector2d computeEDTGradient(double x, double y) const
  {
    const double h = std::max(static_cast<double>(edt_map_.info.resolution), 1e-3);

    const double dx1 = queryEDTValue(x + h, y);
    const double dx0 = queryEDTValue(x - h, y);
    const double dy1 = queryEDTValue(x, y + h);
    const double dy0 = queryEDTValue(x, y - h);

    Eigen::Vector2d g((dx1 - dx0) / (2.0 * h),
                      (dy1 - dy0) / (2.0 * h));

    if (g.norm() < 1e-9)
      return Eigen::Vector2d::Zero();

    return g.normalized();
  }

  bool isSafeByEDT(double x, double y) const
  {
    double edt = 0.0;
    int raw = -1, gx = 0, gy = 0;
    if (!queryEDT(x, y, edt, raw, gx, gy)) return false;
    return edt >= edt_hard_min_;
  }

  // ============================================================
  // yaw 前视
  // ============================================================
  double computeYawLookAhead(double u_now) const
  {
    if (!spline_) return last_yaw_;

    const double u2 = std::min(u_now + yaw_lookahead_u_, spline_->totalU());
    Eigen::Vector3d p0 = spline_->evaluate(u_now);
    Eigen::Vector3d p1 = spline_->evaluate(u2);

    Eigen::Vector2d d = (p1 - p0).head<2>();
    if (d.norm() < 1e-3) return last_yaw_;
    return std::atan2(d.y(), d.x());
  }

  static double normalizeAngle(double a)
  {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  double stepYawToward(double yaw_target)
  {
    double dyaw = normalizeAngle(yaw_target - last_yaw_);
    const double max_step = max_yaw_rate_ * CMD_DT;
    dyaw = std::max(-max_step, std::min(max_step, dyaw));
    last_yaw_ = normalizeAngle(last_yaw_ + dyaw);
    return last_yaw_;
  }

  // ============================================================
  // 定时执行
  // ============================================================
  void cmdTimer(const ros::TimerEvent&)
  {
    if ((!yaw_aligning_ && !executing_) || !spline_) return;

    ros::Time now = ros::Time::now();

    // ---- Stage 1: 原地对齐 yaw ----
    if (yaw_aligning_)
    {
      double yaw = stepYawToward(yaw_align_target_);
      publishPoseCommand(hold_pos_, yaw, now);

      const double yaw_err = std::fabs(normalizeAngle(yaw_align_target_ - last_yaw_));
      const double align_t = (now - yaw_align_start_time_).toSec();
      if (yaw_err <= yaw_align_tol_ || align_t >= yaw_align_timeout_)
      {
        yaw_aligning_ = false;
        executing_ = true;
        start_time_ = now;
      }
      return;
    }

    // ---- Stage 2: 沿轨迹执行 ----
    double u = speed_u_per_sec_ * (now - start_time_).toSec();
    const double U = spline_->totalU();

    if (u >= U)
    {
      u = U;
      executing_ = false;
    }

    Eigen::Vector3d p = spline_->evaluate(u);

    if (has_edt_ && !isSafeByEDT(p.x(), p.y()))
    {
      ROS_ERROR("[ego_optimizer_node] Unsafe point encountered during execution! Stop.");
      executing_ = false;
      return;
    }

    double yaw = stepYawToward(computeYawLookAhead(u));
    publishPoseCommand(p, yaw, now);
  }

  void publishPoseCommand(const Eigen::Vector3d& p, double yaw, const ros::Time& stamp)
  {
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    geometry_msgs::PoseStamped cmd;
    cmd.header.stamp = stamp;
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

  // ============================================================
  // 统计量发布
  // ============================================================
  void publishCostsForSpline(const UniformCubicBSpline& spline)
  {
    const double U = spline.totalU();
    if (U <= EPS) return;

    const double dt = CMD_DT;
    const double du = std::max(EPS, speed_u_per_sec_ * dt);
    const int N = std::max(2, static_cast<int>(std::ceil(U / du)) + 1);

    double cost_v = 0.0;
    double cost_a = 0.0;
    double cost_j = 0.0;

    Eigen::Vector3d p_prev = spline.evaluate(0.0);
    Eigen::Vector3d v_prev = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_prev = Eigen::Vector3d::Zero();

    for (int i = 1; i < N; ++i)
    {
      double u = std::min(U, i * du);
      Eigen::Vector3d p = spline.evaluate(u);

      Eigen::Vector3d v = (p - p_prev) / dt;
      Eigen::Vector3d a = (v - v_prev) / dt;
      Eigen::Vector3d j = (a - a_prev) / dt;

      cost_v += v.squaredNorm() * dt;
      cost_a += a.squaredNorm() * dt;
      cost_j += j.squaredNorm() * dt;

      p_prev = p;
      v_prev = v;
      a_prev = a;

      if (u >= U - 1e-9) break;
    }

    geometry_msgs::Vector3 msg;
    msg.x = cost_v;
    msg.y = cost_a;
    msg.z = cost_j;
    cost_pub_.publish(msg);
  }

  void publishEdtStatsForSpline(const UniformCubicBSpline& spline)
  {
    const double U = spline.totalU();
    if (U <= EPS) return;

    const double dt = CMD_DT;
    const double du = std::max(EPS, speed_u_per_sec_ * dt);
    const int N = std::max(2, static_cast<int>(std::ceil(U / du)) + 1);

    double edt_min = std::numeric_limits<double>::infinity();
    double edt_sum = 0.0;
    int edt_cnt = 0;

    for (int i = 0; i < N; ++i)
    {
      double u = std::min(U, i * du);
      Eigen::Vector3d p = spline.evaluate(u);
      double edt = queryEDTValue(p.x(), p.y());

      edt_min = std::min(edt_min, edt);
      edt_sum += edt;
      ++edt_cnt;

      if (u >= U - 1e-9) break;
    }

    geometry_msgs::Vector3 msg;
    msg.x = std::isfinite(edt_min) ? edt_min : 0.0;
    msg.y = edt_cnt > 0 ? edt_sum / static_cast<double>(edt_cnt) : 0.0;
    msg.z = 0.0;
    edt_pub_.publish(msg);
  }

  void publishTrajInfoForSpline(const UniformCubicBSpline& spline)
  {
    const double U = spline.totalU();
    if (U <= EPS) return;

    const double dt = CMD_DT;
    const double du = std::max(EPS, speed_u_per_sec_ * dt);
    const int N = std::max(2, static_cast<int>(std::ceil(U / du)) + 1);

    double length = 0.0;
    Eigen::Vector3d p_prev = spline.evaluate(0.0);
    for (int i = 1; i < N; ++i)
    {
      double u = std::min(U, i * du);
      Eigen::Vector3d p = spline.evaluate(u);
      length += (p - p_prev).norm();
      p_prev = p;
      if (u >= U - 1e-9) break;
    }

    geometry_msgs::Vector3 msg;
    msg.x = length;
    msg.y = static_cast<double>(std::max(0, N - 1)) * dt;
    msg.z = 0.0;
    traj_info_pub_.publish(msg);
  }

private:
  // ROS
  ros::Subscriber ctrl_pts_sub_;
  ros::Subscriber edt_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher cost_pub_;
  ros::Publisher edt_pub_;
  ros::Publisher traj_info_pub_;
  ros::Timer timer_;

  // EDT
  nav_msgs::OccupancyGrid edt_map_;
  bool has_edt_ = false;

  // 轨迹
  std::unique_ptr<UniformCubicBSpline> spline_;
  ros::Time start_time_;
  bool executing_ = false;

  // 参数
  double edt_hard_min_ = 0.5;
  double speed_u_per_sec_ = 1.5;
  double yaw_lookahead_u_ = 0.15;
  bool   ignore_while_exec_ = true;
  double max_yaw_rate_ = 0.6;
  double yaw_align_tol_ = 0.20;
  double yaw_align_timeout_ = 2.0;

  int    opt_iters_ = 40;
  double sample_du_ = 0.05;
  double step_size_ = 0.08;
  double w_smooth_ = 1.0;
  double w_ref_ = 1.5;
  double w_safe_ = 2.5;
  double safe_band_ = 0.8;
  double ctrl_z_ = 1.0;
  double max_ctrl_shift_ = 0.3;

  // yaw / 执行阶段
  double last_yaw_ = 0.0;
  bool yaw_aligning_ = false;
  double yaw_align_target_ = 0.0;
  ros::Time yaw_align_start_time_;
  Eigen::Vector3d hold_pos_ = Eigen::Vector3d::Zero();
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ego_optimizer_node");
  EgoOptimizerNode node;
  ros::spin();
  return 0;
}