// ================================================================
// centripetal_catmull_rom_optimizer_node.cpp
//
// 用 “Centripetal Catmull–Rom” 直接替换你现在的 Uniform B-spline 插值。
// 输入：
//   - /optimizer/ctrl_points   (geometry_msgs/PoseArray)  控制点序列（>=2）
//   - /optimizer/edt_map       (nav_msgs/OccupancyGrid)   EDT 地图（data: int8 [-1,100] -1未知；0..100为线性编码值
// 输出：
//   - /ardrone_1/command/pose (geometry_msgs/PoseStamped) 位置+姿态指令（50Hz）
//
// 特点：
//   - 必过给定点（插值），第一个点/最后一个点必过
//   - 采用 Centripetal 参数化（α=0.5），显著减少过冲
//   - yaw “超前飞”：朝向前视点
//   - 轨迹采样检查 EDT >= edt_hard_min
//   - 收到新控制点：若当前正在执行轨迹，则默认忽略（防抖）
// ================================================================

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Vector3.h>

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <memory>
#include <limits>
#include <algorithm>

#include <tf/tf.h>

// ----------------------- 参数 -----------------------
static constexpr double CMD_DT    = 0.02;  // 50 Hz 发布
static constexpr double SAMPLE_DT = 0.05;  // 安全检查采样
static constexpr double EPS       = 1e-6;

// ================================================================
// 1) Centripetal Catmull–Rom 样条（插值曲线）
//    - 输入：一串点 P0..Pn
//    - 输出：按弧长-ish 参数 s ∈ [0, total] 计算位置
//
// 注意：Catmull–Rom 常用 4 点定义一段曲线：
//   段 i 表示从 Pi 到 P(i+1)，需要用 P(i-1), Pi, P(i+1), P(i+2)
// 边界处理：端点处通过“复制端点”扩展。
// 参数化：centripetal => α=0.5
//   t0=0
//   t1=t0+|P1-P0|^α
//   t2=t1+|P2-P1|^α
//   t3=t2+|P3-P2|^α
// ================================================================
class CentripetalCatmullRom
{
public:
  explicit CentripetalCatmullRom(std::vector<Eigen::Vector3d> pts, double alpha = 0.5)
    : pts_(std::move(pts)), alpha_(alpha)
  {
    // 至少 2 个点才有意义；少于 2 个点就退化
    if (pts_.size() < 2) return;

    // 预计算每段的“参数长度”（用 centripetal 的 t2-t1 近似段时长）
    // 这里把整条曲线参数 s 定义为各段 (t2 - t1) 的累加
    buildSegmentParams();
  }

  bool valid() const { return pts_.size() >= 2 && !seg_s_.empty(); }

  // 返回整体“参数长度”（相当于 duration，但不是时间）
  double totalS() const
  {
    if (seg_prefix_.empty()) return 0.0;
    return seg_prefix_.back();
  }

  // 在 s∈[0,totalS] 上求位置
  Eigen::Vector3d evaluate(double s) const
  {
    if (pts_.empty()) return Eigen::Vector3d::Zero();
    if (pts_.size() == 1) return pts_.front();
    if (seg_s_.empty()) return pts_.front();

    double S = totalS();
    if (S <= EPS) return pts_.front();

    // clamp
    s = std::min(std::max(s, 0.0), S);

    // 找段：seg i 对应 [prefix(i), prefix(i+1))
    int i = findSegment(s);
    double s0 = (i == 0) ? 0.0 : seg_prefix_[i - 1];
    double ds = seg_s_[i];
    double u  = (ds > EPS) ? (s - s0) / ds : 0.0; // u∈[0,1]

    return evalSegment(i, u);
  }

private:
  // 构建段参数：seg_s_[i] ~ (t2 - t1)
  void buildSegmentParams()
  {
    // 段数 = n-1（从 P0->P1, P1->P2, ...）
    int n = static_cast<int>(pts_.size());
    int segN = n - 1;

    seg_s_.assign(segN, 0.0);
    seg_prefix_.assign(segN, 0.0);

    double acc = 0.0;
    for (int i = 0; i < segN; ++i)
    {
      // 取段 i 的四个点：P(i-1), Pi, P(i+1), P(i+2)（越界用端点复制）
      Eigen::Vector3d P0 = pts_[std::max(i - 1, 0)];
      Eigen::Vector3d P1 = pts_[i];
      Eigen::Vector3d P2 = pts_[i + 1];
      Eigen::Vector3d P3 = pts_[std::min(i + 2, n - 1)];

      // centripetal 参数化的 t 序列
      double t0 = 0.0;
      double t1 = t0 + tj(P0, P1);
      double t2 = t1 + tj(P1, P2);
      double t3 = t2 + tj(P2, P3);

      double ds = (t2 - t1);
      if (ds < EPS) ds = 1.0; // 避免零长度段

      seg_s_[i] = ds;
      acc += ds;
      seg_prefix_[i] = acc;
    }
  }

  // tj = |Pi - Pj|^α
  double tj(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const
  {
    double d = (b - a).norm();
    // centripetal α=0.5 => sqrt(d)
    return std::pow(std::max(d, 0.0), alpha_);
  }

  int findSegment(double s) const
  {
    // seg_prefix_[i] = sum_{k<=i} seg_s_[k]
    // 找最小 i 使 prefix[i] >= s
    auto it = std::lower_bound(seg_prefix_.begin(), seg_prefix_.end(), s);
    int i = static_cast<int>(std::distance(seg_prefix_.begin(), it));
    if (i < 0) i = 0;
    if (i >= static_cast<int>(seg_s_.size())) i = static_cast<int>(seg_s_.size()) - 1;
    return i;
  }

  // 计算某一段 i（Pi->P(i+1)）在 u∈[0,1] 的位置
  Eigen::Vector3d evalSegment(int i, double u) const
  {
    int n = static_cast<int>(pts_.size());

    Eigen::Vector3d P0 = pts_[std::max(i - 1, 0)];
    Eigen::Vector3d P1 = pts_[i];
    Eigen::Vector3d P2 = pts_[i + 1];
    Eigen::Vector3d P3 = pts_[std::min(i + 2, n - 1)];

    // centripetal 参数化
    double t0 = 0.0;
    double t1 = t0 + tj(P0, P1);
    double t2 = t1 + tj(P1, P2);
    double t3 = t2 + tj(P2, P3);

    // 该段的实际参数 t ∈ [t1, t2]
    double t = (1.0 - u) * t1 + u * t2;

    // 按 Catmull–Rom 的“递推线性插值”公式：
    // A1 = (t1-t)/(t1-t0)*P0 + (t-t0)/(t1-t0)*P1
    // A2 = (t2-t)/(t2-t1)*P1 + (t-t1)/(t2-t1)*P2
    // A3 = (t3-t)/(t3-t2)*P2 + (t-t2)/(t3-t2)*P3
    // B1 = (t2-t)/(t2-t0)*A1 + (t-t0)/(t2-t0)*A2
    // B2 = (t3-t)/(t3-t1)*A2 + (t-t1)/(t3-t1)*A3
    // C  = (t2-t)/(t2-t1)*B1 + (t-t1)/(t2-t1)*B2
    Eigen::Vector3d A1 = lerpSafe(P0, P1, t0, t1, t);
    Eigen::Vector3d A2 = lerpSafe(P1, P2, t1, t2, t);
    Eigen::Vector3d A3 = lerpSafe(P2, P3, t2, t3, t);

    Eigen::Vector3d B1 = lerpSafe(A1, A2, t0, t2, t);
    Eigen::Vector3d B2 = lerpSafe(A2, A3, t1, t3, t);

    Eigen::Vector3d C  = lerpSafe(B1, B2, t1, t2, t);

    return C;
  }

  // 在区间 [ta,tb] 上按参数 t 做线性插值，处理除零
  static Eigen::Vector3d lerpSafe(const Eigen::Vector3d& Pa,
                                  const Eigen::Vector3d& Pb,
                                  double ta, double tb, double t)
  {
    double denom = (tb - ta);
    if (std::fabs(denom) < EPS) return Pa;
    double w = (t - ta) / denom;
    return (1.0 - w) * Pa + w * Pb;
  }

private:
  std::vector<Eigen::Vector3d> pts_;
  double alpha_;

  std::vector<double> seg_s_;      // 每段参数长度
  std::vector<double> seg_prefix_; // 前缀和（累计到该段末尾）
};

// ================================================================
// 2) ROS Node：订阅控制点 + EDT，发布姿态指令
// ================================================================
class CatmullRomOptimizerNode
{
public:
  CatmullRomOptimizerNode()
  {
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;

    // ---- 参数 ----
    pnh.param("edt_hard_min",    edt_hard_min_,    0.5);
    pnh.param("yaw_lookahead_s", yaw_lookahead_s_, 0.20); // 在曲线参数 s 上的前视量
    pnh.param("speed_s_per_sec", speed_s_per_sec_, 2.0);  // s 参数推进速度（越大飞得越快）
    pnh.param("ignore_while_exec", ignore_while_exec_, true);

    // ---- 订阅控制点序列（>=2） ----
    ctrl_pts_sub_ = nh.subscribe(
        "/optimizer/ctrl_points", 1,
        &CatmullRomOptimizerNode::ctrlPointsCallback, this);

    // ---- 订阅 EDT ----
    edt_sub_ = nh.subscribe(
        "/optimizer/edt_map", 1,
        &CatmullRomOptimizerNode::edtCallback, this);

    // ---- 发布无人机指令 ----
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
        &CatmullRomOptimizerNode::cmdTimer, this);

    ROS_INFO("[CentripetalCatmullRom_optimizer] Ready. edt_hard_min=%.3f, speed_s_per_sec=%.2f",
             edt_hard_min_, speed_s_per_sec_);
  }

private:
  void edtCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    edt_map_ = *msg;
    has_edt_ = true;
  }

  void ctrlPointsCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
    if (msg->poses.size() < 2)
    {
      ROS_WARN("[CentripetalCatmullRom_optimizer] Need >=2 points.");
      return;
    }
    if (ignore_while_exec_ && executing_)
    {
      ROS_WARN("[CentripetalCatmullRom_optimizer] Executing, ignore new points.");
      return;
    }
    if (!has_edt_)
    {
      ROS_WARN("[CentripetalCatmullRom_optimizer] No EDT yet.");
      return;
    }

    std::vector<Eigen::Vector3d> pts;
    pts.reserve(msg->poses.size());
    for (const auto& p : msg->poses)
    {
      pts.emplace_back(p.position.x, p.position.y, p.position.z);
    }

    // ===================== DEBUG LOG: start/end + EDT =====================
    // 你现在发布的是 4 个控制点：我们把第一个点当起点，最后一个点当终点。
    if (msg->poses.size() >= 4)
    {
      const Eigen::Vector3d &p_start = pts.front();
      const Eigen::Vector3d &p_goal  = pts.back();

      double edt_s = 0.0, edt_g = 0.0;
      int raw_s = -1, raw_g = -1;
      int gsx = 0, gsy = 0, ggx = 0, ggy = 0;

      bool ok_s = queryEDT(p_start.x(), p_start.y(), edt_s, raw_s, gsx, gsy);
      bool ok_g = queryEDT(p_goal.x(),  p_goal.y(),  edt_g, raw_g, ggx, ggy);

      if (ok_s)
      {
        ROS_WARN_STREAM("[DEBUG][CTRL_PTS] START pos=(" << p_start.x() << "," << p_start.y()
                        << ") grid=(" << gsx << "," << gsy << ") raw_v=" << raw_s
                        << " edt_m=" << edt_s << " thr=" << edt_hard_min_);
      }
      else
      {
        ROS_WARN_STREAM("[DEBUG][CTRL_PTS] START pos=(" << p_start.x() << "," << p_start.y()
                        << ") EDT QUERY FAIL (out/unknown). raw_v=" << raw_s);
      }

      if (ok_g)
      {
        ROS_WARN_STREAM("[DEBUG][CTRL_PTS] GOAL  pos=(" << p_goal.x() << "," << p_goal.y()
                        << ") grid=(" << ggx << "," << ggy << ") raw_v=" << raw_g
                        << " edt_m=" << edt_g << " thr=" << edt_hard_min_);
      }
      else
      {
        ROS_WARN_STREAM("[DEBUG][CTRL_PTS] GOAL  pos=(" << p_goal.x() << "," << p_goal.y()
                        << ") EDT QUERY FAIL (out/unknown). raw_v=" << raw_g);
      }
    }
    else
    {
      // 兼容旧的 >=2 点输入：也打印首尾
      const Eigen::Vector3d &p_start = pts.front();
      const Eigen::Vector3d &p_goal  = pts.back();

      double edt_s = 0.0, edt_g = 0.0;
      int raw_s = -1, raw_g = -1;
      int gsx = 0, gsy = 0, ggx = 0, ggy = 0;

      bool ok_s = queryEDT(p_start.x(), p_start.y(), edt_s, raw_s, gsx, gsy);
      bool ok_g = queryEDT(p_goal.x(),  p_goal.y(),  edt_g, raw_g, ggx, ggy);

      ROS_WARN_STREAM("[DEBUG][CTRL_PTS] (legacy) pts=" << pts.size());
      ROS_WARN_STREAM("[DEBUG][CTRL_PTS] START pos=(" << p_start.x() << "," << p_start.y()
                      << ") grid=(" << gsx << "," << gsy << ") raw_v=" << raw_s
                      << " edt_m=" << edt_s << " thr=" << edt_hard_min_ << " ok=" << ok_s);
      ROS_WARN_STREAM("[DEBUG][CTRL_PTS] GOAL  pos=(" << p_goal.x() << "," << p_goal.y()
                      << ") grid=(" << ggx << "," << ggy << ") raw_v=" << raw_g
                      << " edt_m=" << edt_g << " thr=" << edt_hard_min_ << " ok=" << ok_g);
    }
    // ================================================================

    // 构造 centripetal Catmull–Rom（α=0.5）
    auto cand = std::make_unique<CentripetalCatmullRom>(pts, 0.5);
    if (!cand->valid() || cand->totalS() <= EPS)
    {
      ROS_WARN("[CentripetalCatmullRom_optimizer] Curve invalid.");
      return;
    }

    // 安全性检查
    if (!checkCurveSafety(*cand))
    {
      ROS_WARN("[CentripetalCatmullRom_optimizer] Curve violates EDT constraint.");
      return;
    }

    curve_ = std::move(cand);
    start_time_ = ros::Time::now();
    executing_ = true;

    ROS_INFO("[CentripetalCatmullRom_optimizer] Curve accepted. points=%zu, totalS=%.3f",
             pts.size(), curve_->totalS());
    publishCostsForCurve(*curve_);
    publishEdtStatsForCurve(*curve_);
    publishTrajInfoForCurve(*curve_);
  }

  // 查询某个世界坐标点的 EDT 值（米）。
  // 返回 true 表示查询成功（在地图内且非 unknown），并把 edt_m 写入 out。
  bool queryEDT(double x, double y, double &edt_m, int &raw_v, int &gx, int &gy) const
  {
    edt_m = 0.0;
    raw_v = -1;

    const double ox  = edt_map_.info.origin.position.x;
    const double oy  = edt_map_.info.origin.position.y;
    const double res = edt_map_.info.resolution;

    gx = int((x - ox) / res);
    gy = int((y - oy) / res);

    if (gx < 0 || gy < 0 || gx >= (int)edt_map_.info.width || gy >= (int)edt_map_.info.height)
      return false;

    const int idx = gy * (int)edt_map_.info.width + gx;
    raw_v = static_cast<int>(edt_map_.data[idx]);

    // unknown
    if (raw_v < 0)
      return false;

    // clamp [0,100]
    const int v = std::min(100, std::max(0, raw_v));

    // HARD-CODED max range to match Python encoder
    constexpr double kMaxRangeM = 5.0;
    constexpr double kMetersPerUnit = kMaxRangeM / 100.0; // 0.05m per unit

    edt_m = static_cast<double>(v) * kMetersPerUnit;
    return true;
  }

  bool checkCurveSafety(const CentripetalCatmullRom& c)
  {
    const double S = c.totalS();
    if (S <= EPS) return false;

    // 用 s 参数均匀采样（注意：s 是曲线参数，不是时间；这里只是做安全性覆盖采样）
    // 若你希望更严，可把步长改小一些
    const double ds = std::max(0.05, speed_s_per_sec_ * SAMPLE_DT);

    for (double s = 0.0; s <= S + 1e-9; s += ds)
    {
      Eigen::Vector3d p = c.evaluate(s);
      if (!isSafeByEDT(p.x(), p.y()))
      {
        ROS_ERROR_STREAM("[CentripetalCatmullRom_optimizer][SAFETY FAIL] s=" << s
                         << " pos=(" << p.x() << "," << p.y() << ")"
                         << " totalS=" << S);
        return false;
      }
    }
    return true;
  }

  bool isSafeByEDT(double x, double y) const
  {
    // 世界坐标 -> 栅格
    const double ox = edt_map_.info.origin.position.x;
    const double oy = edt_map_.info.origin.position.y;
    const double res = edt_map_.info.resolution;

    int gx = int((x - ox) / res);
    int gy = int((y - oy) / res);

    if (gx < 0 || gy < 0 || gx >= (int)edt_map_.info.width || gy >= (int)edt_map_.info.height)
    {
      ROS_ERROR_STREAM("[CentripetalCatmullRom_optimizer][EDT OUT OF MAP] pos=(" << x << "," << y
                       << ") grid=(" << gx << "," << gy << ")"
                       << " map_size=(" << edt_map_.info.width << "," << edt_map_.info.height << ")"
                       << " origin=(" << ox << "," << oy << ") res=" << res);
      return false;
    }

    const int idx = gy * (int)edt_map_.info.width + gx;

    // ===================== EDT decode (HARD-CODED to match Python) =====================
    // Python publish_edt_map():
    //   max_range_m = 5.0
    //   scale = 100.0 / max_range_m
    //   v = int(min(d * scale, 100.0))  where d is EDT in meters
    // So decode is:
    //   d(m) = v * (max_range_m / 100.0) = v * 0.05
    // OccupancyGrid.data is int8 [-1,100]; -1 means unknown.

    const int raw_v = static_cast<int>(edt_map_.data[idx]);

    // Unknown cell
    if (raw_v < 0)
    {
      ROS_ERROR_STREAM("[CentripetalCatmullRom_optimizer][EDT UNKNOWN] pos=(" << x << "," << y
                       << ") grid=(" << gx << "," << gy << ") raw_v=" << raw_v);
      return false;
    }

    // Clamp to [0,100]
    const int v = std::min(100, std::max(0, raw_v));

    // HARD-CODED max range to match Python encoder
    constexpr double kMaxRangeM = 5.0;
    constexpr double kMetersPerUnit = kMaxRangeM / 100.0; // 0.05m per unit

    const double edt_m = static_cast<double>(v) * kMetersPerUnit;

    if (edt_m < edt_hard_min_)
    {
      ROS_ERROR_STREAM("[CentripetalCatmullRom_optimizer][EDT TOO SMALL] pos=(" << x << "," << y
                       << ") grid=(" << gx << "," << gy << ") raw_v=" << raw_v
                       << " v=" << v << " edt_m=" << edt_m
                       << " threshold=" << edt_hard_min_);
      return false;
    }

    return true;
  }

  double computeYawLookAhead(double s_now) const
  {
    if (!curve_) return last_yaw_;

    double s2 = std::min(s_now + yaw_lookahead_s_, curve_->totalS());
    Eigen::Vector3d p0 = curve_->evaluate(s_now);
    Eigen::Vector3d p1 = curve_->evaluate(s2);

    Eigen::Vector2d d = (p1 - p0).head<2>();
    if (d.norm() < 1e-3) return last_yaw_;

    return std::atan2(d.y(), d.x());
  }

  void cmdTimer(const ros::TimerEvent&)
  {
    if (!executing_ || !curve_) return;

    ros::Time now = ros::Time::now();
    double t = (now - start_time_).toSec();

    // s 参数按“速度”推进：s = v_s * t
    double s = speed_s_per_sec_ * t;
    double S = curve_->totalS();

    if (s >= S)
    {
      s = S;
      executing_ = false; // 到终点就停止接受新点（或你也可以在这里悬停继续发布）
    }

    Eigen::Vector3d p = curve_->evaluate(s);

    // 如果运行中遇到不安全（地图更新导致）——可选择紧急停止
    if (has_edt_ && !isSafeByEDT(p.x(), p.y()))
    {
      ROS_ERROR("[CentripetalCatmullRom_optimizer] Unsafe point encountered during execution! Stop.");
      executing_ = false;
      return;
    }

    double yaw = computeYawLookAhead(s);
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
  ros::Subscriber ctrl_pts_sub_;
  ros::Subscriber edt_sub_;
  ros::Publisher  cmd_pub_;
  ros::Publisher  cost_pub_;
  ros::Publisher  edt_pub_;
  ros::Publisher  traj_info_pub_;
  ros::Timer      timer_;

  // Helper to compute and publish trajectory length (meters) and execution time (seconds)
  // for the curve under the current execution parametrization s(t)=speed_s_per_sec_*t.
  void publishTrajInfoForCurve(const CentripetalCatmullRom& c)
  {
    const double S = c.totalS();
    if (S <= EPS) return;

    // We execute the curve by advancing the curve parameter with:
    //   s(t) = speed_s_per_sec_ * t
    // and we publish commands every CMD_DT.
    // Therefore, the EXECUTION time can be estimated by how many CMD_DT ticks
    // are needed to reach S with step ds = speed_s_per_sec_ * CMD_DT.

    const double dt = CMD_DT;
    const double ds = std::max(EPS, speed_s_per_sec_ * dt);

    // Number of sampled points along the execution (including s=0).
    const int N = std::max(2, int(std::ceil(S / ds)) + 1);

    // Execution ticks are N-1 intervals of length dt.
    const double traj_time_exec = double(std::max(0, N - 1)) * dt;

    // Approximate geometric length by sampling positions along s using the same discretization.
    double length_m = 0.0;
    Eigen::Vector3d p_prev = c.evaluate(0.0);

    for (int i = 1; i < N; ++i)
    {
      double s = std::min(S, i * ds);
      Eigen::Vector3d p = c.evaluate(s);
      length_m += (p - p_prev).norm();
      p_prev = p;
      if (s >= S - 1e-9) break;
    }

    geometry_msgs::Vector3 msg;
    msg.x = length_m;       // trajectory length (m)
    msg.y = traj_time_exec; // trajectory execution time (s) estimated by CMD_DT ticks
    msg.z = 0.0;
    traj_info_pub_.publish(msg);

    ROS_INFO_STREAM("[CentripetalCatmullRom_optimizer][TRAJ] length_m=" << length_m
                    << " traj_time_exec=" << traj_time_exec
                    << " N=" << N
                    << " dt=" << dt
                    << " ds=" << ds
                    << " totalS=" << S
                    << " speed_s_per_sec=" << speed_s_per_sec_);
  }

  // Helper to compute and publish costs for a curve
  void publishCostsForCurve(const CentripetalCatmullRom& c)
  {
    // Compute simple discrete costs along execution time:
    // s(t) = speed_s_per_sec_ * t
    // v, a, jerk derived from finite differences of position.

    const double S = c.totalS();
    if (S <= EPS) return;

    // Use controller dt as the sampling dt for cost.
    const double dt = CMD_DT;
    const double ds = std::max(EPS, speed_s_per_sec_ * dt);

    // Need enough samples for velocity/acc/jerk.
    const int N = std::max(2, int(std::ceil(S / ds)) + 1);

    double cost_v = 0.0;
    double cost_a = 0.0;
    double cost_j = 0.0;

    Eigen::Vector3d p_prev = c.evaluate(0.0);
    Eigen::Vector3d v_prev = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_prev = Eigen::Vector3d::Zero();

    for (int i = 1; i < N; ++i)
    {
      double s = std::min(S, i * ds);
      Eigen::Vector3d p = c.evaluate(s);

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

      if (s >= S - 1e-9) break;
    }

    geometry_msgs::Vector3 msg;
    msg.x = cost_v;
    msg.y = cost_a;
    msg.z = cost_j;
    cost_pub_.publish(msg);

    ROS_INFO_STREAM("[CentripetalCatmullRom_optimizer][COST] cost_v=" << cost_v
                    << " cost_a=" << cost_a
                    << " cost_jerk=" << cost_j);
  }

  // Helper to compute and publish EDT stats (min / mean) along the curve
  void publishEdtStatsForCurve(const CentripetalCatmullRom& c)
  {
    // Sample EDT along the curve using the same discretization as cost:
    // s(t) = speed_s_per_sec_ * t, sample at controller dt.

    const double S = c.totalS();
    if (S <= EPS) return;

    const double dt = CMD_DT;
    const double ds = std::max(EPS, speed_s_per_sec_ * dt);
    const int N = std::max(2, int(std::ceil(S / ds)) + 1);

    double edt_min = std::numeric_limits<double>::infinity();
    double edt_sum = 0.0;
    int    edt_cnt = 0;

    for (int i = 0; i < N; ++i)
    {
      double s = std::min(S, i * ds);
      Eigen::Vector3d p = c.evaluate(s);

      double edt_m = 0.0;
      int raw_v = -1;
      int gx = 0, gy = 0;

      // Safety should already guarantee query success, but keep it robust.
      if (!queryEDT(p.x(), p.y(), edt_m, raw_v, gx, gy))
      {
        // Treat unknown/out as 0 for stats (also indicates inconsistency)
        edt_m = 0.0;
      }

      edt_min = std::min(edt_min, edt_m);
      edt_sum += edt_m;
      edt_cnt += 1;

      if (s >= S - 1e-9) break;
    }

    if (edt_cnt <= 0) return;
    const double edt_mean = edt_sum / double(edt_cnt);
    if (!std::isfinite(edt_min)) return;

    geometry_msgs::Vector3 msg;
    msg.x = edt_min;   // min EDT along curve (m)
    msg.y = edt_mean;  // mean EDT along curve (m)
    msg.z = 0.0;
    edt_pub_.publish(msg);

    ROS_INFO_STREAM("[CentripetalCatmullRom_optimizer][EDT] edt_min=" << edt_min
                    << " edt_mean=" << edt_mean
                    << " samples=" << edt_cnt);
  }

  // EDT
  nav_msgs::OccupancyGrid edt_map_;
  bool has_edt_ = false;

  // 曲线
  std::unique_ptr<CentripetalCatmullRom> curve_;
  ros::Time start_time_;
  bool executing_ = false;

  // 参数
  double edt_hard_min_ = 0.5;
  double yaw_lookahead_s_ = 0.2;
  double speed_s_per_sec_ = 2.0;
  bool   ignore_while_exec_ = true;

  // yaw
  double last_yaw_ = 0.0;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "centripetal_catmull_rom_optimizer_node");
  CatmullRomOptimizerNode node;
  ros::spin();
  return 0;
}
