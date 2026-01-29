#include "activity_control_pkg/waypoint_navigator.h"
#include <tf/transform_datatypes.h>
#include <cmath>

namespace {
inline double meterToCm(double m) { return m * 100.0; }
inline double radToDeg(double r) { return r * 180.0 / M_PI; }
inline double normalizeAngleDeg(double a) {
  while (a > 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}
}

namespace activity_control_pkg {

// 构造函数
WaypointNavigator::WaypointNavigator(ros::NodeHandle &nh,
                                     const std::string &map_frame,
                                     const std::string &laser_link_frame,
                                     const std::string &output_topic)
    : nh_(nh),
      output_topic_(output_topic),
      map_frame_(map_frame), 
      laser_link_frame_(laser_link_frame)
{
  // 发布器：目标点
  target_position_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(output_topic_, 1, /*latch=*/true);
  // 订阅高度（绝对话题 /height，单位 cm）
  height_sub_ = nh_.subscribe<std_msgs::Int16>("/height", 10, &WaypointNavigator::heightCallback, this);
  
  ROS_INFO("[WaypointNavigator] 初始化完成, map=%s, laser_link=%s, topic=%s",
           map_frame_.c_str(), laser_link_frame_.c_str(), output_topic_.c_str());
}

// 添加目标点
void WaypointNavigator::addTarget(double x, double y, double z, double yaw) {
  Target t;
  t.x_cm = x;
  t.y_cm = y;
  t.z_cm = z;
  t.yaw_deg = yaw;

  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_empty = targets_.empty();
  targets_.push_back(t);
  if (was_empty) {
    current_idx_ = 0;
    publishCurrent();
  }
}

// 清除所有航点
void WaypointNavigator::clearTargets() {
  std::lock_guard<std::mutex> lock(mutex_);
  targets_.clear();
  current_idx_ = static_cast<std::size_t>(-1);
}

// 获取当前索引
std::size_t WaypointNavigator::currentIndex() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_idx_;
}

// 当前目标总数
std::size_t WaypointNavigator::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return targets_.size();
}

// 获取当前目标点
bool WaypointNavigator::getCurrentTarget(Target &t) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_idx_ != static_cast<std::size_t>(-1) && current_idx_ < targets_.size()) {
    t = targets_[current_idx_];
    return true;
  }
  return false;
}

// 推进到下一个航点
bool WaypointNavigator::advanceToNext() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_idx_ == static_cast<std::size_t>(-1) || current_idx_ >= targets_.size()) {
    return false;
  }
  
  ROS_INFO("[WaypointNavigator] 目标 %zu 达成，准备发布下一个", current_idx_);
  current_idx_++;
  
  if (current_idx_ < targets_.size()) {
    publishTarget(targets_[current_idx_], false);
    return true;
  } else {
    // 所有目标已完成
    Target landing;
    landing.x_cm = -1;
    landing.y_cm = -1;
    landing.z_cm = -1;
    landing.yaw_deg = -1;
    publishTarget(landing, false);
    ROS_INFO("[WaypointNavigator] 所有目标已完成");
    return false;
  }
}

// 发布当前目标
void WaypointNavigator::publishCurrent() {
  if (current_idx_ != static_cast<std::size_t>(-1) && current_idx_ < targets_.size()) {
    publishTarget(targets_[current_idx_], current_idx_ == 0);
  }
}

// 发布降落/停止命令
void WaypointNavigator::publishFinal() {
  Target landing;
  landing.x_cm = -1;
  landing.y_cm = -1;
  landing.z_cm = -1;
  landing.yaw_deg = -1;
  publishTarget(landing, false);
}

// 实际发布到话题
void WaypointNavigator::publishTarget(const Target &t, bool init_flag) {
  std_msgs::Float32MultiArray out;
  out.data.resize(4);
  out.data[0] = static_cast<float>(t.x_cm);
  out.data[1] = static_cast<float>(t.y_cm);
  out.data[2] = static_cast<float>(t.z_cm);
  out.data[3] = static_cast<float>(t.yaw_deg);
  target_position_pub_.publish(out);
  ROS_INFO("[WaypointNavigator] 发布目标: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg%s",
           t.x_cm, t.y_cm, t.z_cm, t.yaw_deg, init_flag ? " (首个)" : "");
}

// 高度回调
void WaypointNavigator::heightCallback(const std_msgs::Int16::ConstPtr &msg) {
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

// 获取实时位姿
bool WaypointNavigator::getCurrentPose(double &x_cm, double &y_cm, double &z_cm, double &yaw_deg) {
  tf::StampedTransform transform;
  try {
    listener_.lookupTransform(map_frame_, laser_link_frame_, ros::Time(0), transform);
    x_cm = meterToCm(transform.getOrigin().x());
    y_cm = meterToCm(transform.getOrigin().y());
    z_cm = has_height_ ? current_height_cm_ : 0.0;
    tf::Quaternion q = transform.getRotation();
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_deg = radToDeg(yaw);
    return true;
  } catch (tf::TransformException &ex) {
    ROS_WARN_THROTTLE(2.0, "TF 查询失败 (%s->%s): %s", map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
    return false;
  }
}

// 判断是否到达目标
bool WaypointNavigator::isReached(const Target &t, double x_cm, double y_cm, double z_cm, double yaw_deg) {
  double dx = t.x_cm - x_cm;
  double dy = t.y_cm - y_cm;
  double dxy = std::sqrt(dx * dx + dy * dy);
  double dz = t.z_cm - z_cm;
  double dyaw = normalizeAngleDeg(t.yaw_deg - yaw_deg);
  bool z_ok = has_height_ ? (std::fabs(dz) <= height_tol_cm_) : false;
  return z_ok && (dxy <= pos_tol_cm_) && (std::fabs(dyaw) <= yaw_tol_deg_);
}

// 分段降落
void WaypointNavigator::startSegmentedLanding() {
  double x, y, z, yaw;
  std::lock_guard<std::mutex> lock(mutex_);

  // 获取当前无人机实时位置
  if (!getCurrentPose(x, y, z, yaw)) {
    ROS_WARN("[WaypointNavigator] 无法获取当前位姿（TF缺失），忽略降落请求！");
    return;
  }
  //添加降落航点到末尾

  ROS_INFO("[WaypointNavigator] 启动分段降落! 基准位置: x=%.1f, y=%.1f, z=%.1f", x, y, z);

  // 构建分段降落路径（保持 x, y, yaw 不变）
  if (z > 55.0) {
    addTarget(x, y, 50.0, yaw);
  }
  if(z > 10.0) {
    addTarget(x, y, 10.0, yaw);
  }
}

} // namespace activity_control_pkg
