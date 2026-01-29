#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include "activity_control_pkg/waypoint_navigator.h"

namespace activity_control_pkg {

// 任务层状态机
enum class InspectionState {
  NAVIGATE_TO_WAYPOINT = 0,
  PREPARE_VISION_SERVO = 1,
  VISUAL_ALIGNMENT = 2,
  LASER_SHOOTING = 3,
  NEXT_WAYPOINT = 4
};

/**
 * @brief 二维码对准任务
 * 
 * 任务层负责：二维码订阅、对准判断、PID/激光控制、任务航点硬编码
 * 构造即运行：初始化任务航点并启动定时器
 */
class QrAlignmentTask {
public:
  QrAlignmentTask(ros::NodeHandle &nh, WaypointNavigator &navigator);

private:
  // 任务航点硬编码
  void buildTaskPath();

  // 定时检查与状态机
  void monitorTimerCallback(const ros::TimerEvent &);

  // 二维码偏移回调
  void qrCodeOffsetCallback(const geometry_msgs::Point::ConstPtr &msg);

  // 发送激光控制
  void publishLaserControl(bool enable);

  // 发送PID模式
  void publishPIDMode(uint8_t mode);

  // 判断二维码对准
  bool isQRCodeAligned();

private:
  ros::NodeHandle nh_;
  WaypointNavigator &navigator_;

  // ROS资源
  ros::Subscriber qr_code_sub_;
  ros::Publisher pid_mode_pub_;
  ros::Publisher laser_control_pub_;
  ros::Timer monitor_timer_;

  // 任务状态机
  InspectionState inspection_state_{InspectionState::NAVIGATE_TO_WAYPOINT};

  // 二维码对准
  double qr_offset_x_{0.0};
  double qr_offset_y_{0.0};
  double qr_target_x_{320.0};
  double qr_target_y_{170.0};
  double qr_align_error_{20.0};
};

} // namespace activity_control_pkg
