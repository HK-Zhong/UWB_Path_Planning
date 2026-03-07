#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Vector3.h>

#include <Eigen/Dense>
#include <tf/tf.h>

#include <vector>
#include <cmath>
#include <memory>
#include <limits>
#include <string>

/**
 * =========================================================
 * Minimum-snap trajectory optimizer node
 *
 * Topics (fixed to match your "all /optimizer/"):
 *   Sub:
 *     /optimizer/ctrl_points   (geometry_msgs/PoseArray)  >=2 poses
 *     /optimizer/edt_map       (nav_msgs/OccupancyGrid)   int8 [-1..100]
 *   Pub:
 *     /ardrone_1/command/pose  (geometry_msgs/PoseStamped)
 *
 * Params (keep launch unchanged):
 *   edt_hard_min       (double)  safety threshold in meters
 *   edt_max_range_m    (double)  decode max range (MUST match python, default 5.0)
 *   speed_s_per_sec    (double)  speed along trajectory arc-length approx
 *   yaw_lookahead_s    (double)  yaw lookahead in seconds
 *   ignore_while_exec  (bool)    ignore new ctrl points while executing
 * =========================================================
 */

static constexpr double CMD_DT = 0.02;   // 50Hz publish
static constexpr double SAMPLE_DT = 0.05; // safety sampling
static constexpr int POLY_DEG = 7;       // 7th order (minimum snap standard)
static constexpr int COEF_N = POLY_DEG + 1; // 8 coefficients per segment
static constexpr int DERIV_MAX_CONT = 3; // continuity up to jerk (0..3)

/**
 * =========================================================
 * Helper: factorial / power
 * =========================================================
 */
static inline double fact(int n)
{
  double r = 1.0;
  for (int i = 2; i <= n; ++i) r *= double(i);
  return r;
}

static inline double powi(double x, int p)
{
  double r = 1.0;
  for (int i = 0; i < p; ++i) r *= x;
  return r;
}

/**
 * =========================================================
 * Polynomial basis:
 * p(t) = sum_{k=0..7} c_k t^k
 * p^(d)(t) = sum_{k=d..7} c_k * k*(k-1)*...*(k-d+1) * t^(k-d)
 * =========================================================
 */
static inline Eigen::RowVectorXd polyDerivRow(double t, int deriv)
{
  Eigen::RowVectorXd r(COEF_N);
  r.setZero();
  for (int k = deriv; k <= POLY_DEG; ++k)
  {
    double coeff = 1.0;
    for (int j = 0; j < deriv; ++j) coeff *= double(k - j);
    r(k) = coeff * powi(t, k - deriv);
  }
  return r;
}

/**
 * =========================================================
 * Minimum snap Q for one segment of duration T:
 * cost = ∫0^T (p''''(t))^2 dt = c^T Q c
 * =========================================================
 */
static inline Eigen::MatrixXd segmentSnapQ(double T)
{
  Eigen::MatrixXd Q(COEF_N, COEF_N);
  Q.setZero();

  const int d = 4; // snap derivative order
  for (int i = d; i <= POLY_DEG; ++i)
  {
    for (int j = d; j <= POLY_DEG; ++j)
    {
      double ci = 1.0, cj = 1.0;
      for (int k = 0; k < d; ++k) ci *= double(i - k);
      for (int k = 0; k < d; ++k) cj *= double(j - k);

      int p = i + j - 2 * d;
      // ∫0^T t^p dt = T^(p+1)/(p+1)
      double val = ci * cj * powi(T, p + 1) / double(p + 1);
      Q(i, j) = val;
    }
  }
  return Q;
}

/**
 * =========================================================
 * Simple trajectory container
 * - per segment 7th poly coeffs for x,y,z
 * - durations per segment
 * =========================================================
 */
struct MinSnapTraj
{
  std::vector<double> T;               // segment durations (size M)
  std::vector<Eigen::VectorXd> cx;     // size M, each 8x1
  std::vector<Eigen::VectorXd> cy;
  std::vector<Eigen::VectorXd> cz;

  double totalDuration() const
  {
    double s = 0.0;
    for (double t : T) s += t;
    return s;
  }

  // evaluate at global time t
  Eigen::Vector3d eval(double t) const
  {
    if (T.empty()) return Eigen::Vector3d::Zero();
    double acc = 0.0;
    int seg = 0;
    for (; seg < (int)T.size(); ++seg)
    {
      if (t <= acc + T[seg] || seg == (int)T.size() - 1) break;
      acc += T[seg];
    }
    double local = std::min(std::max(t - acc, 0.0), T[seg]);

    auto evalPoly = [&](const Eigen::VectorXd& c)->double{
      double v = 0.0;
      double tt = 1.0;
      for (int k = 0; k <= POLY_DEG; ++k)
      {
        v += c(k) * tt;
        tt *= local;
      }
      return v;
    };

    return Eigen::Vector3d(evalPoly(cx[seg]), evalPoly(cy[seg]), evalPoly(cz[seg]));
  }

  // evaluate velocity (for yaw)
  Eigen::Vector3d evalVel(double t) const
  {
    if (T.empty()) return Eigen::Vector3d::Zero();
    double acc = 0.0;
    int seg = 0;
    for (; seg < (int)T.size(); ++seg)
    {
      if (t <= acc + T[seg] || seg == (int)T.size() - 1) break;
      acc += T[seg];
    }
    double local = std::min(std::max(t - acc, 0.0), T[seg]);

    auto evalPolyD1 = [&](const Eigen::VectorXd& c)->double{
      double v = 0.0;
      double tt = 1.0;
      for (int k = 1; k <= POLY_DEG; ++k)
      {
        v += c(k) * double(k) * tt;
        tt *= local;
      }
      return v;
    };

    return Eigen::Vector3d(evalPolyD1(cx[seg]), evalPolyD1(cy[seg]), evalPolyD1(cz[seg]));
  }

  // evaluate acceleration
  Eigen::Vector3d evalAcc(double t) const
  {
    if (T.empty()) return Eigen::Vector3d::Zero();
    double acc = 0.0;
    int seg = 0;
    for (; seg < (int)T.size(); ++seg)
    {
      if (t <= acc + T[seg] || seg == (int)T.size() - 1) break;
      acc += T[seg];
    }
    double local = std::min(std::max(t - acc, 0.0), T[seg]);

    auto evalPolyD2 = [&](const Eigen::VectorXd& c)->double{
      double v = 0.0;
      double tt = 1.0;
      for (int k = 2; k <= POLY_DEG; ++k)
      {
        v += c(k) * double(k) * double(k - 1) * tt;
        tt *= local;
      }
      return v;
    };

    return Eigen::Vector3d(evalPolyD2(cx[seg]), evalPolyD2(cy[seg]), evalPolyD2(cz[seg]));
  }

  // evaluate jerk
  Eigen::Vector3d evalJerk(double t) const
  {
    if (T.empty()) return Eigen::Vector3d::Zero();
    double acc = 0.0;
    int seg = 0;
    for (; seg < (int)T.size(); ++seg)
    {
      if (t <= acc + T[seg] || seg == (int)T.size() - 1) break;
      acc += T[seg];
    }
    double local = std::min(std::max(t - acc, 0.0), T[seg]);

    auto evalPolyD3 = [&](const Eigen::VectorXd& c)->double{
      double v = 0.0;
      double tt = 1.0;
      for (int k = 3; k <= POLY_DEG; ++k)
      {
        v += c(k) * double(k) * double(k - 1) * double(k - 2) * tt;
        tt *= local;
      }
      return v;
    };

    return Eigen::Vector3d(evalPolyD3(cx[seg]), evalPolyD3(cy[seg]), evalPolyD3(cz[seg]));
  }
};

/**
 * =========================================================
 * Node
 * =========================================================
 */
class MinimumSnapOptimizerNode
{
public:
  MinimumSnapOptimizerNode()
  {
    ros::NodeHandle nh("~");

    nh.param("edt_hard_min", edt_hard_min_, 0.5);
    nh.param("edt_max_range_m", edt_max_range_m_, 5.0);
    nh.param("speed_s_per_sec", speed_s_per_sec_, 2.0);
    nh.param("yaw_lookahead_s", yaw_lookahead_s_, 0.3);
    nh.param("ignore_while_exec", ignore_while_exec_, true);
    nh.param("max_yaw_rate", max_yaw_rate_, 0.6);
    nh.param("yaw_align_tol", yaw_align_tol_, 0.20);
    nh.param("yaw_align_timeout", yaw_align_timeout_, 2.0);

    ctrl_sub_ = nh.subscribe("/optimizer/ctrl_points", 1,
                             &MinimumSnapOptimizerNode::ctrlCb, this);
    edt_sub_  = nh.subscribe("/optimizer/edt_map", 1,
                             &MinimumSnapOptimizerNode::edtCb, this);

    cmd_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
        "/ardrone_1/command/pose", 10);
    cost_pub_ = nh.advertise<geometry_msgs::Vector3>("/optimizer/cost", 10, true);
    edt_pub_  = nh.advertise<geometry_msgs::Vector3>("/optimizer/edt", 10, true);
    traj_info_pub_ = nh.advertise<geometry_msgs::Vector3>("/optimizer/traj_info", 10, true);

    timer_ = nh.createTimer(ros::Duration(CMD_DT),
                            &MinimumSnapOptimizerNode::timerCb, this);

    ROS_INFO("[MinimumSnap_optimizer] Ready. edt_hard_min=%.3f, edt_max_range_m=%.2f, speed=%.2f, yaw_lookahead=%.2f, ignore_while_exec=%s, max_yaw_rate=%.2f, yaw_align_tol=%.2f, yaw_align_timeout=%.2f",
             edt_hard_min_, edt_max_range_m_, speed_s_per_sec_, yaw_lookahead_s_,
             ignore_while_exec_ ? "true" : "false",
             max_yaw_rate_, yaw_align_tol_, yaw_align_timeout_);
  }

private:
  // ---------------------------
  // callbacks
  // ---------------------------
  void edtCb(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    edt_map_ = *msg;
    has_edt_ = true;
  }

  void ctrlCb(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
    if (msg->poses.size() < 2)
    {
      ROS_WARN("[MinimumSnap_optimizer] Need >=2 ctrl points.");
      return;
    }

    if (ignore_while_exec_ && executing_)
    {
      ROS_WARN("[MinimumSnap_optimizer] executing, ignore new ctrl points.");
      return;
    }

    if (!has_edt_)
    {
      ROS_WARN("[MinimumSnap_optimizer] No EDT map yet.");
      return;
    }

    std::vector<Eigen::Vector3d> wp;
    wp.reserve(msg->poses.size());
    for (const auto& p : msg->poses)
    {
      wp.emplace_back(p.position.x, p.position.y, p.position.z);
    }

    // --- log: start/end + EDT ---
    Eigen::Vector3d p0 = wp.front();
    Eigen::Vector3d pN = wp.back();
    double edt0 = queryEDT(p0);
    double edtN = queryEDT(pN);
    ROS_WARN("[MinimumSnap_optimizer][CTRL] start=(%.2f,%.2f,%.2f) edt=%.3f | goal=(%.2f,%.2f,%.2f) edt=%.3f | K=%zu",
             p0.x(), p0.y(), p0.z(), edt0,
             pN.x(), pN.y(), pN.z(), edtN,
             wp.size());

    // build durations
    std::vector<double> T = allocateTime(wp, speed_s_per_sec_);

    // generate min-snap
    MinSnapTraj traj;
    if (!solveMinimumSnap(wp, T, traj))
    {
      ROS_WARN("[MinimumSnap_optimizer] solveMinimumSnap failed.");
      return;
    }

    // safety check
    if (!checkTrajSafety(traj))
    {
      ROS_WARN("[MinimumSnap_optimizer] Trajectory violates EDT constraint.");
      return;
    }

    traj_ = std::make_shared<MinSnapTraj>(traj);

    // Stage 1: first align yaw toward the initial path direction while holding position.
    hold_pos_ = traj_->eval(0.0);
    yaw_align_target_ = computeYawLookAhead(0.0);
    yaw_align_start_time_ = ros::Time::now();
    yaw_aligning_ = true;
    executing_ = false;

    publishTrajectoryCost(*traj_);
    publishEdtStatsForTraj(*traj_);
    publishTrajInfoForTraj(*traj_);

    ROS_INFO("[MinimumSnap_optimizer] Trajectory accepted. segments=%zu, duration=%.2fs",
             traj_->T.size(), traj_->totalDuration());
  }

  // ---------------------------
  // Yaw helpers (for two-stage execution)
  // ---------------------------
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

  double computeYawLookAhead(double t_now) const
  {
    if (!traj_) return last_yaw_;

    double T = traj_->totalDuration();
    double t2 = std::min(t_now + yaw_lookahead_s_, T);
    Eigen::Vector3d p0 = traj_->eval(t_now);
    Eigen::Vector3d p1 = traj_->eval(t2);

    Eigen::Vector2d d = (p1 - p0).head<2>();
    if (d.norm() < 1e-3) return last_yaw_;

    return std::atan2(d.y(), d.x());
  }

  void timerCb(const ros::TimerEvent&)
  {
    if ((!yaw_aligning_ && !executing_) || !traj_) return;

    ros::Time now = ros::Time::now();

    // Stage 1: hold position and rotate the nose toward the initial path direction.
    if (yaw_aligning_)
    {
      double yaw = stepYawToward(yaw_align_target_);

      tf::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);

      geometry_msgs::PoseStamped cmd;
      cmd.header.stamp = now;
      cmd.header.frame_id = "map";
      cmd.pose.position.x = hold_pos_.x();
      cmd.pose.position.y = hold_pos_.y();
      cmd.pose.position.z = hold_pos_.z();
      cmd.pose.orientation.x = q.x();
      cmd.pose.orientation.y = q.y();
      cmd.pose.orientation.z = q.z();
      cmd.pose.orientation.w = q.w();
      cmd_pub_.publish(cmd);

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

    // Stage 2: move along the trajectory normally.
    double t = (now - start_time_).toSec();
    double T = traj_->totalDuration();
    if (t >= T)
    {
      t = T;
      executing_ = false;
    }

    Eigen::Vector3d p = traj_->eval(t);

    double yaw_des = computeYawLookAhead(t);
    double yaw = stepYawToward(yaw_des);

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

  // ---------------------------
  // EDT decode exactly matching your python publisher
  // python:
  //   v = int(min(d * (100/max_range_m), 100))
  // so decode:
  //   d = (v/100) * max_range_m
  // ---------------------------
  double decodeEDT(int8_t raw_v) const
  {
    if (raw_v <= 0) return 0.0;
    if (raw_v > 100) raw_v = 100;
    return (double(raw_v) / 100.0) * edt_max_range_m_;
  }

  double queryEDT(const Eigen::Vector3d& p) const
  {
    if (!has_edt_) return 0.0;

    const auto& info = edt_map_.info;
    int gx = int((p.x() - info.origin.position.x) / info.resolution);
    int gy = int((p.y() - info.origin.position.y) / info.resolution);

    if (gx < 0 || gy < 0 || gx >= (int)info.width || gy >= (int)info.height)
      return 0.0;

    int idx = gy * info.width + gx;
    if (idx < 0 || idx >= (int)edt_map_.data.size()) return 0.0;

    return decodeEDT(edt_map_.data[idx]);
  }

  bool isSafeByEDT(const Eigen::Vector3d& p) const
  {
    return queryEDT(p) >= edt_hard_min_;
  }

  // ---------------------------
  // time allocation
  // ---------------------------
  std::vector<double> allocateTime(const std::vector<Eigen::Vector3d>& wp, double speed) const
  {
    int M = int(wp.size()) - 1;
    std::vector<double> T(M, 0.5);

    speed = std::max(speed, 0.1);
    for (int i = 0; i < M; ++i)
    {
      double d = (wp[i+1] - wp[i]).head<2>().norm();
      double ti = d / speed;
      // keep a minimum duration to avoid numerical issues
      T[i] = std::max(0.25, ti);
    }
    return T;
  }

  // ---------------------------
  // QP solver: min 0.5 x^T Q x  s.t. A x = b
  // via KKT:
  // [Q A^T; A 0] [x;lambda] = [0;b]
  // ---------------------------
  bool solveQP(const Eigen::MatrixXd& Q,
               const Eigen::MatrixXd& A,
               const Eigen::VectorXd& b,
               Eigen::VectorXd& x) const
  {
    int n = Q.rows();
    int m = A.rows();
    if (Q.cols() != n) return false;
    if (A.cols() != n) return false;
    if (b.size() != m) return false;

    Eigen::MatrixXd KKT(n + m, n + m);
    KKT.setZero();

    // small regularization (helps numerics)
    Eigen::MatrixXd Qr = Q;
    Qr.diagonal().array() += 1e-9;

    KKT.topLeftCorner(n, n) = Qr;
    KKT.topRightCorner(n, m) = A.transpose();
    KKT.bottomLeftCorner(m, n) = A;

    Eigen::VectorXd rhs(n + m);
    rhs.setZero();
    rhs.tail(m) = b;

    Eigen::VectorXd sol = KKT.fullPivLu().solve(rhs);
    if ((KKT * sol - rhs).norm() > 1e-5)
    {
      // still accept in many cases; but warn
      ROS_WARN("[MinimumSnap_optimizer] KKT residual=%.3e", (KKT * sol - rhs).norm());
    }

    x = sol.head(n);
    return true;
  }

  // ---------------------------
  // build constraints for one axis
  // ---------------------------
  bool solveMinimumSnap1D(const std::vector<double>& w,     // waypoint positions (K)
                          const std::vector<double>& T,     // segment durations (M=K-1)
                          std::vector<Eigen::VectorXd>& c_out) const
  {
    const int K = (int)w.size();
    const int M = K - 1;
    const int nvar = M * COEF_N;

    // Q
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(nvar, nvar);
    for (int i = 0; i < M; ++i)
    {
      Eigen::MatrixXd Qi = segmentSnapQ(T[i]);
      Q.block(i*COEF_N, i*COEF_N, COEF_N, COEF_N) = Qi;
    }

    // Constraints:
    // 1) Position at each segment start/end:
    //    p_i(0)=w_i, p_i(Ti)=w_{i+1}  => 2M eq
    // 2) Continuity at internal waypoints for deriv 1..3:
    //    p_{i-1}^{(d)}(Ti-1) = p_i^{(d)}(0)  for i=1..M-1, d=1..3 => (M-1)*3 eq
    // 3) Endpoint derivatives at start and end set to 0 for d=1..3:
    //    p_0^{(d)}(0)=0, p_{M-1}^{(d)}(T)=0 => 2*3 eq
    //
    // Total eq:
    //   2M + 3(M-1) + 6 = 5M + 3
    const int neq = 2*M + (M-1)*DERIV_MAX_CONT + 2*DERIV_MAX_CONT;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(neq, nvar);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(neq);

    int row = 0;

    // pos constraints
    for (int i = 0; i < M; ++i)
    {
      // p_i(0) = w_i
      A.block(row, i*COEF_N, 1, COEF_N) = polyDerivRow(0.0, 0);
      b(row) = w[i];
      row++;

      // p_i(Ti) = w_{i+1}
      A.block(row, i*COEF_N, 1, COEF_N) = polyDerivRow(T[i], 0);
      b(row) = w[i+1];
      row++;
    }

    // continuity for d=1..3 at internal joints
    for (int i = 1; i < M; ++i)
    {
      for (int d = 1; d <= DERIV_MAX_CONT; ++d)
      {
        // p_{i-1}^{(d)}(T_{i-1}) - p_i^{(d)}(0) = 0
        A.block(row, (i-1)*COEF_N, 1, COEF_N) = polyDerivRow(T[i-1], d);
        A.block(row, i*COEF_N,     1, COEF_N) = -polyDerivRow(0.0, d);
        b(row) = 0.0;
        row++;
      }
    }

    // endpoint derivatives
    for (int d = 1; d <= DERIV_MAX_CONT; ++d)
    {
      // start: p0^{(d)}(0)=0
      A.block(row, 0, 1, COEF_N) = polyDerivRow(0.0, d);
      b(row) = 0.0;
      row++;
    }
    for (int d = 1; d <= DERIV_MAX_CONT; ++d)
    {
      // end: p_{M-1}^{(d)}(T)=0
      A.block(row, (M-1)*COEF_N, 1, COEF_N) = polyDerivRow(T[M-1], d);
      b(row) = 0.0;
      row++;
    }

    if (row != neq)
    {
      ROS_WARN("[MinimumSnap_optimizer] constraint row mismatch row=%d neq=%d", row, neq);
    }

    Eigen::VectorXd x;
    if (!solveQP(Q, A, b, x)) return false;

    // extract coefficients
    c_out.clear();
    c_out.resize(M);
    for (int i = 0; i < M; ++i)
      c_out[i] = x.segment(i*COEF_N, COEF_N);

    return true;
  }

  bool solveMinimumSnap(const std::vector<Eigen::Vector3d>& wp,
                        const std::vector<double>& T,
                        MinSnapTraj& traj) const
  {
    if (wp.size() < 2) return false;
    if ((int)T.size() != (int)wp.size()-1) return false;

    std::vector<double> wx, wy, wz;
    wx.reserve(wp.size());
    wy.reserve(wp.size());
    wz.reserve(wp.size());
    for (const auto& p : wp)
    {
      wx.push_back(p.x());
      wy.push_back(p.y());
      wz.push_back(p.z());
    }

    std::vector<Eigen::VectorXd> cx, cy, cz;
    if (!solveMinimumSnap1D(wx, T, cx)) return false;
    if (!solveMinimumSnap1D(wy, T, cy)) return false;
    if (!solveMinimumSnap1D(wz, T, cz)) return false;

    traj.T  = T;
    traj.cx = cx;
    traj.cy = cy;
    traj.cz = cz;
    return true;
  }

  bool checkTrajSafety(const MinSnapTraj& traj) const
  {
    double T = traj.totalDuration();
    if (T <= 0.0) return false;

    for (double t = 0.0; t <= T; t += SAMPLE_DT)
    {
      Eigen::Vector3d p = traj.eval(t);
      if (!isSafeByEDT(p))
      {
        double edt = queryEDT(p);
        ROS_ERROR("[MinimumSnap_optimizer][EDT TOO SMALL] t=%.2f pos=(%.2f,%.2f) edt=%.3f thr=%.3f",
                  t, p.x(), p.y(), edt, edt_hard_min_);
        return false;
      }
    }
    return true;
  }

private:
  ros::Subscriber ctrl_sub_;
  ros::Subscriber edt_sub_;
  ros::Publisher  cmd_pub_;
  ros::Publisher  cost_pub_;
  ros::Publisher  edt_pub_;
  ros::Publisher  traj_info_pub_;
  ros::Timer      timer_;

  nav_msgs::OccupancyGrid edt_map_;
  bool has_edt_ = false;

  // params
  double edt_hard_min_ = 0.5;
  double edt_max_range_m_ = 5.0;
  double speed_s_per_sec_ = 2.0;
  double yaw_lookahead_s_ = 0.3;
  bool ignore_while_exec_ = true;
  double max_yaw_rate_ = 0.6;
  double yaw_align_tol_ = 0.20;
  double yaw_align_timeout_ = 2.0;

  // state
  std::shared_ptr<MinSnapTraj> traj_;
  ros::Time start_time_;
  bool executing_ = false;
  double last_yaw_ = 0.0;
  bool yaw_aligning_ = false;
  double yaw_align_target_ = 0.0;
  ros::Time yaw_align_start_time_;
  Eigen::Vector3d hold_pos_ = Eigen::Vector3d::Zero();

  // Publish trajectory cost (velocity, acceleration, jerk integrals)
  void publishTrajectoryCost(const MinSnapTraj& traj)
  {
    // Costs are simple integral of squared derivatives over time
    // cost_v   = ∫ ||v||^2 dt
    // cost_a   = ∫ ||a||^2 dt
    // cost_jerk= ∫ ||j||^2 dt
    const double T = traj.totalDuration();
    if (T <= 0.0) return;

    double cost_v = 0.0;
    double cost_a = 0.0;
    double cost_j = 0.0;

    // Use the same step as command publish for stable/consistent numbers
    for (double t = 0.0; t <= T; t += CMD_DT)
    {
      Eigen::Vector3d v = traj.evalVel(t);
      Eigen::Vector3d a = traj.evalAcc(t);
      Eigen::Vector3d j = traj.evalJerk(t);

      cost_v += v.squaredNorm() * CMD_DT;
      cost_a += a.squaredNorm() * CMD_DT;
      cost_j += j.squaredNorm() * CMD_DT;
    }

    geometry_msgs::Vector3 msg;
    msg.x = cost_v;
    msg.y = cost_a;
    msg.z = cost_j;

    cost_pub_.publish(msg);

    ROS_INFO("[MinimumSnap_optimizer][COST] v=%.6f a=%.6f jerk=%.6f", cost_v, cost_a, cost_j);
  }

    // Publish EDT stats (min/mean) along trajectory
  void publishEdtStatsForTraj(const MinSnapTraj& traj)
  {
    // Sample EDT along the trajectory using the same discretization as cost:
    // sample at controller dt.

    const double T = traj.totalDuration();
    if (T <= 0.0) return;

    double edt_min = std::numeric_limits<double>::infinity();
    double edt_sum = 0.0;
    int    edt_cnt = 0;

    for (double t = 0.0; t <= T; t += CMD_DT)
    {
      Eigen::Vector3d p = traj.eval(t);

      // queryEDT() already handles out-of-map/unknown as 0.0
      double edt_m = queryEDT(p);

      edt_min = std::min(edt_min, edt_m);
      edt_sum += edt_m;
      edt_cnt += 1;
    }

    if (edt_cnt <= 0) return;
    const double edt_mean = edt_sum / double(edt_cnt);
    if (!std::isfinite(edt_min)) return;

    geometry_msgs::Vector3 msg;
    msg.x = edt_min;   // min EDT along trajectory (m)
    msg.y = edt_mean;  // mean EDT along trajectory (m)
    msg.z = 0.0;
    edt_pub_.publish(msg);

    ROS_INFO("[MinimumSnap_optimizer][EDT] edt_min=%.6f edt_mean=%.6f samples=%d",
             edt_min, edt_mean, edt_cnt);
  }

  // Publish trajectory info (length and execution time)
  void publishTrajInfoForTraj(const MinSnapTraj& traj)
  {
    const double T = traj.totalDuration();
    if (T <= 0.0) return;

    // We execute the trajectory by publishing commands every CMD_DT.
    // Therefore, the EXECUTION time can be estimated by how many CMD_DT ticks
    // are needed to reach total duration T.

    const double dt = CMD_DT;

    // Number of sampled points along the execution (including t=0).
    const int N = std::max(2, int(std::ceil(T / dt)) + 1);

    // Execution ticks are N-1 intervals of length dt.
    const double traj_time_exec = double(std::max(0, N - 1)) * dt;

    // Approximate geometric length by sampling positions along time using the same discretization.
    double length_m = 0.0;
    Eigen::Vector3d p_prev = traj.eval(0.0);

    for (int i = 1; i < N; ++i)
    {
      double t = std::min(T, i * dt);
      Eigen::Vector3d p = traj.eval(t);
      length_m += (p - p_prev).norm();
      p_prev = p;
      if (t >= T - 1e-9) break;
    }

    geometry_msgs::Vector3 msg;
    msg.x = length_m;       // trajectory length (m)
    msg.y = traj_time_exec; // trajectory execution time (s) estimated by CMD_DT ticks
    msg.z = 0.0;
    traj_info_pub_.publish(msg);

    ROS_INFO_STREAM("[MinimumSnap_optimizer][TRAJ] length_m=" << length_m
                    << " traj_time_exec=" << traj_time_exec
                    << " N=" << N
                    << " dt=" << dt
                    << " totalDuration=" << T);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MinimumSnap_optimizer_node");
  MinimumSnapOptimizerNode node;
  ros::spin();
  return 0;
}