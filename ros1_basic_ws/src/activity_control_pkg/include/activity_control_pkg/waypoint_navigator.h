#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <tf/transform_listener.h>
#include <string>
#include <vector>
#include <mutex>

namespace activity_control_pkg {

// 目标点（单位：cm 与 度）- 通用结构，不含任务语义
struct Target {
  double x_cm;     // 目标 X（cm）
  double y_cm;     // 目标 Y（cm）
  double z_cm;     // 目标 Z（cm）
  double yaw_deg;  // 航向角（度）
  bool skip_processing{false};  // 是否跳过后续处理（到达即下一个）
};

/**
 * @brief 通用航点导航器
 * 
 * 负责航点管理、发布、到达判定与分段降落
 * 不包含任何任务特定逻辑
 */
class WaypointNavigator {
public:
  /**
   * @brief 构造函数
   * @param nh 节点句柄
   * @param map_frame 地图坐标系名称
   * @param laser_link_frame 雷达坐标系名称
   * @param output_topic 发布目标位置的话题
   */
  explicit WaypointNavigator(ros::NodeHandle &nh,
                             const std::string &map_frame = "map",
                             const std::string &laser_link_frame = "laser_link",
                             const std::string &output_topic = "/target_position");

  // ========== 航点管理接口 ==========
  
  /**
   * @brief 添加目标点
   * @param x X坐标(cm)
   * @param y Y坐标(cm)
   * @param z Z坐标(cm)
   * @param yaw 航向角(度)
   * @param skip_processing 是否跳过后续处理(默认false)
   */
  void addTarget(double x, double y, double z, double yaw, bool skip_processing = false);

  /**
   * @brief 清除所有航点
   */
  void clearTargets();

  /**
   * @brief 获取当前正在执行的目标索引（0起始）
   * @return 当前索引，无目标时返回 size_t(-1)
   */
  std::size_t currentIndex() const;

  /**
   * @brief 当前目标总数
   */
  std::size_t size() const;

  /**
   * @brief 获取当前目标点（如果存在）
   * @param t 输出目标点
   * @return 是否存在当前目标
   */
  bool getCurrentTarget(Target &t) const;

  /**
   * @brief 推进到下一个航点
   * @return 是否还有下一个航点
   */
  bool advanceToNext();

  // ========== 位姿与到达判定 ==========

  /**
   * @brief 获取当前位姿
   * @param x_cm 输出X(cm)
   * @param y_cm 输出Y(cm)
   * @param z_cm 输出Z(cm)
   * @param yaw_deg 输出航向(度)
   * @return 是否成功获取
   */
  bool getCurrentPose(double &x_cm, double &y_cm, double &z_cm, double &yaw_deg);

  /**
   * @brief 判断是否到达目标
   * @param t 目标点
   * @param x_cm 当前X(cm)
   * @param y_cm 当前Y(cm)
   * @param z_cm 当前Z(cm)
   * @param yaw_deg 当前航向(度)
   * @return 是否到达
   */
  bool isReached(const Target &t, double x_cm, double y_cm, double z_cm, double yaw_deg);

  /**
   * @brief 等待 TF 可用
   * @param timeout 超时
   * @return 是否在超时内可用
   */
  bool waitForTransform(const ros::Duration &timeout);

  // ========== 发布接口 ==========

  /**
   * @brief 发布当前目标点
   */
  void publishCurrent();

  /**
   * @brief 发布降落/停止命令
   */
  void publishFinal();

  /**
   * @brief 分段降落
   * 获取当前位置并规划分段降落路径
   */
  void startSegmentedLanding();

  // ========== 容差设置 ==========
  void setPositionTolerance(double tol_cm) { pos_tol_cm_ = tol_cm; }
  void setYawTolerance(double tol_deg) { yaw_tol_deg_ = tol_deg; }
  void setHeightTolerance(double tol_cm) { height_tol_cm_ = tol_cm; }

protected:
  // 发布目标点到话题
  void publishTarget(const Target &t, bool init_flag);

  // 高度回调
  void heightCallback(const std_msgs::Int16::ConstPtr &msg);

  // ROS 资源
  ros::NodeHandle nh_;
  ros::Publisher target_position_pub_;
  ros::Subscriber height_sub_;
  tf::TransformListener listener_;

  // 坐标系
  std::string output_topic_;
  std::string map_frame_;
  std::string laser_link_frame_;

  // 航点队列
  std::vector<Target> targets_;
  mutable std::mutex mutex_;
  std::size_t current_idx_{static_cast<std::size_t>(-1)};

  // 容差（单位：cm 与 度）
  double pos_tol_cm_{4.0};
  double yaw_tol_deg_{5.0};
  double height_tol_cm_{3.0};

  // 高度状态
  bool has_height_{false};
  double current_height_cm_{0.0};
};

} // namespace activity_control_pkg
