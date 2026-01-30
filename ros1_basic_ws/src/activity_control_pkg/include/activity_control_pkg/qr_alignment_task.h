#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <deque>
#include "activity_control_pkg/waypoint_navigator.h"
#include "activity_control_pkg/qr_constants.h"  // 统一的二维码常量

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

  // 误差模长趋势判定（宽松）
  bool isErrorTrendDecreasing() const;

  // 检查视觉伺服是否超时
  bool checkVisualServoTimeout();

  // 检查二维码数据是否有效
  bool isQRCodeDataValid();

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

  // 二维码对准（使用统一常量）
  double qr_offset_x_{0.0};
  double qr_offset_y_{0.0};
  double qr_target_x_{qr_constants::QR_TARGET_X};
  double qr_target_y_{qr_constants::QR_TARGET_Y};
  double qr_align_error_{qr_constants::QR_ALIGN_ERROR};

  // 二维码平滑与趋势判定
  std::deque<double> qr_offset_x_history_{};
  std::deque<double> qr_offset_y_history_{};
  std::deque<double> qr_error_history_{};
  
  // 视觉伺服超时控制
  ros::Time visual_servo_start_time_;
  double visual_servo_timeout_{qr_constants::QR_VISUAL_SERVO_TIMEOUT};  // 视觉伺服超时时间（秒）
  bool visual_servo_started_{false};
  
  // 二维码数据有效性
  ros::Time last_qr_update_time_;
  double qr_data_timeout_{qr_constants::QR_DATA_TIMEOUT};  // 二维码数据超时时间（秒）
  bool qr_data_valid_{false};

  // 激光射击等待（系统时钟）
  ros::WallTime laser_shot_start_time_;
  bool laser_shot_in_progress_{false};
  double laser_shot_wait_{qr_constants::QR_LASER_SHOT_WAIT};

  // 二维码平滑与趋势判定
  std::size_t smooth_window_size_{static_cast<std::size_t>(qr_constants::QR_SMOOTH_WINDOW_SIZE)};
  std::size_t trend_window_size_{static_cast<std::size_t>(qr_constants::QR_TREND_WINDOW_SIZE)};
  double trend_percent_threshold_{qr_constants::QR_TREND_PERCENT_THRESHOLD};
};

} // namespace activity_control_pkg
