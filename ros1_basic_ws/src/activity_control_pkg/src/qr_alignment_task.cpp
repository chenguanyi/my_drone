#include "activity_control_pkg/qr_alignment_task.h"
#include <cmath>

namespace activity_control_pkg {

QrAlignmentTask::QrAlignmentTask(ros::NodeHandle &nh, WaypointNavigator &navigator)
    : nh_(nh), navigator_(navigator) {
  // 发布器
  pid_mode_pub_ = nh_.advertise<std_msgs::UInt8>("/pid_mode", 1, true);
  laser_control_pub_ = nh_.advertise<std_msgs::Bool>("/laser_control", 1, true);

  // 订阅二维码偏移
  qr_code_sub_ = nh_.subscribe<geometry_msgs::Point>("/qr_code_position", 10,
                                                    &QrAlignmentTask::qrCodeOffsetCallback, this);

  // 构造即运行：构建任务航点
  buildTaskPath();

  // 启动定时器，周期检查是否达到当前目标
  monitor_timer_ = nh_.createTimer(ros::Duration(0.05), &QrAlignmentTask::monitorTimerCallback, this);

  ROS_INFO("[QrAlignmentTask] 任务已启动");
}

// 任务航点硬编码
void QrAlignmentTask::buildTaskPath() {
  navigator_.addTarget(0.0, 0.0, 150.0, 0.0);
  navigator_.addTarget(100.0, 0.0, 150.0, 0.0);
  navigator_.addTarget(100.0, 100.0, 150.0, 0.0);
  navigator_.addTarget(100.0, 100.0, 150.0, 90.0);
  navigator_.startSegmentedLanding();
  ROS_INFO("[QrAlignmentTask] 航点已加载，共 %zu 个", navigator_.size());
}

// 定时器：检查状态转换
void QrAlignmentTask::monitorTimerCallback(const ros::TimerEvent &) {
  Target t;
  if (!navigator_.getCurrentTarget(t)) {
    return;
  }

  double x_cm, y_cm, z_cm, yaw_deg;
  if (!navigator_.getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) return;

  switch (inspection_state_) {
    case InspectionState::NAVIGATE_TO_WAYPOINT:
      if (navigator_.isReached(t, x_cm, y_cm, z_cm, yaw_deg)) {
        inspection_state_ = InspectionState::PREPARE_VISION_SERVO;
      }
      break;
    case InspectionState::PREPARE_VISION_SERVO:
      // 任务层决策：需要二维码对准则切换模式
      publishPIDMode(1);
      inspection_state_ = InspectionState::VISUAL_ALIGNMENT;
      break;
    case InspectionState::VISUAL_ALIGNMENT:
      if (isQRCodeAligned()) {
        inspection_state_ = InspectionState::LASER_SHOOTING;
      }
      break;
    case InspectionState::LASER_SHOOTING:
      publishLaserControl(true);
      publishPIDMode(0);
      inspection_state_ = InspectionState::NEXT_WAYPOINT;
      break;
    case InspectionState::NEXT_WAYPOINT:
      if (!navigator_.advanceToNext()) {
        // 已无下一目标
        return;
      }
      inspection_state_ = InspectionState::NAVIGATE_TO_WAYPOINT;
      break;
  }

  ROS_INFO_THROTTLE(1.0, "[QrAlignmentTask] mode=%d", static_cast<int>(inspection_state_));
}

// 二维码偏移回调
void QrAlignmentTask::qrCodeOffsetCallback(const geometry_msgs::Point::ConstPtr &msg) {
  qr_offset_x_ = msg->x;
  qr_offset_y_ = msg->y;
}

// 发送激光控制
void QrAlignmentTask::publishLaserControl(bool enable) {
  std_msgs::Bool msg;
  msg.data = enable;
  laser_control_pub_.publish(msg);
}

// 发送PID模式
void QrAlignmentTask::publishPIDMode(uint8_t mode) {
  std_msgs::UInt8 msg;
  msg.data = mode;
  pid_mode_pub_.publish(msg);
}

// 判断二维码对准
bool QrAlignmentTask::isQRCodeAligned() {
  return (std::abs(qr_offset_x_ - qr_target_x_) < qr_align_error_ &&
          std::abs(qr_offset_y_ - qr_target_y_) < qr_align_error_);
}

} // namespace activity_control_pkg
