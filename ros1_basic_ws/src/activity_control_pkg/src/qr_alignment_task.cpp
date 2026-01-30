#include "activity_control_pkg/qr_alignment_task.h"
#include <cmath>
#include <algorithm>

namespace activity_control_pkg {

QrAlignmentTask::QrAlignmentTask(ros::NodeHandle &nh, WaypointNavigator &navigator)
    : nh_(nh), navigator_(navigator) {
  // 参数：平滑窗口与趋势阈值（宽松判定）
  int smooth_window_size_param = qr_constants::QR_SMOOTH_WINDOW_SIZE;
  int trend_window_size_param = qr_constants::QR_TREND_WINDOW_SIZE;
  nh_.param("qr/smooth_window_size", smooth_window_size_param, qr_constants::QR_SMOOTH_WINDOW_SIZE);
  nh_.param("qr/trend_window_size", trend_window_size_param, qr_constants::QR_TREND_WINDOW_SIZE);
  nh_.param("qr/trend_percent_threshold", trend_percent_threshold_, qr_constants::QR_TREND_PERCENT_THRESHOLD);
  nh_.param("qr/align_error_px", qr_align_error_, qr_constants::QR_ALIGN_ERROR);
  nh_.param("qr/laser_shot_wait_s", laser_shot_wait_, qr_constants::QR_LASER_SHOT_WAIT);
  smooth_window_size_ = std::max(1, smooth_window_size_param);
  trend_window_size_ = std::max(1, trend_window_size_param);

  // 发布器
  pid_mode_pub_ = nh_.advertise<std_msgs::UInt8>("/pid_mode", 1, true);
  laser_control_pub_ = nh_.advertise<std_msgs::Bool>("/laser_control", 1, true);

  // 订阅二维码偏移
  qr_code_sub_ = nh_.subscribe<geometry_msgs::Point>("/qr_code_position", 10,
                                                    &QrAlignmentTask::qrCodeOffsetCallback, this);

  // 初始化时间戳
  last_qr_update_time_ = ros::Time::now();
  visual_servo_start_time_ = ros::Time::now();

  // 构造即运行：构建任务航点
  buildTaskPath();
  ROS_INFO("[QrAlignmentTask] 航点数量确认: %zu", navigator_.size());

  // 启动前等待 TF 就绪，避免启动瞬间 TF 不存在
  navigator_.waitForTransform(ros::Duration(3.0));

  // 启动定时器，周期检查是否达到当前目标
  monitor_timer_ = nh_.createTimer(ros::Duration(0.02), &QrAlignmentTask::monitorTimerCallback, this);

  ROS_INFO("[QrAlignmentTask] 任务已启动");
}

// 任务航点硬编码
void QrAlignmentTask::buildTaskPath() {
  ROS_INFO("[QrAlignmentTask] 进入任务");
  //0,0,150
  navigator_.addTarget(0.0, 0.0, 130.0, 0.0, true);  // 跳过处理
  navigator_.addTarget(-50, 100.0, 127.0, 0.0,true);

  navigator_.addTarget(-102, 100.0, 127.0, 0.0);
  navigator_.addTarget(-102, 100.0, 92.0, 0.0);
  navigator_.addTarget(-152, 100.0, 92.0, 0.0);
  navigator_.addTarget(-152, 100.0, 127.0, 0.0);
  navigator_.addTarget(-202, 100.0, 127.0, 0.0);
  navigator_.addTarget(-202, 100.0, 92.0, 0.0);

  navigator_.addTarget(-202, 100.0, 0.0, 0.0, true);
  // navigator_.addTarget(-300, 100.0, 92.0, 0.0, true);   // 跳过处理
  // navigator_.addTarget(-300, 300.0, 92.0, 0.0, true);   // 跳过处理
  // navigator_.addTarget(-300, 300.0, 92.0, 180.0, true); // 跳过处理
  
  // navigator_.addTarget(-202, 300.0, 92.0, 180.0);
  // navigator_.addTarget(-202, 300.0, 127.0, 180.0);
  // navigator_.addTarget(-152, 300.0, 127.0, 180.0);
  // navigator_.addTarget(-152, 300.0, 92.0, 180.0);
  // navigator_.addTarget(-102, 300.0, 92.0, 180.0);
  // navigator_.addTarget(-102, 300.0, 127.0, 180.0);
  
  // navigator_.addTarget(-293, 376.0, 127.0, 180.0);
  // navigator_.addTarget(-293.0, 376.0, 92.0, 180.0,true);
  // // navigator_.startSegmentedLanding();
  navigator_.addTarget(-293.0, 376.0, 0.0, 180.0, true); // 跳过处理

  ROS_INFO("[QrAlignmentTask] 航点已加载，共 %zu 个", navigator_.size());
}

// 定时器：检查状态转换
void QrAlignmentTask::monitorTimerCallback(const ros::TimerEvent &) {
  static InspectionState last_state = InspectionState::NAVIGATE_TO_WAYPOINT;
  Target t;
  if (!navigator_.getCurrentTarget(t)) {
    return;
  }

  double x_cm, y_cm, z_cm, yaw_deg;
  if (!navigator_.getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
    ROS_WARN_THROTTLE(2.0, "[QrAlignmentTask] 无法获取当前位姿，跳过本次检查");
    return;
  }
  
  // 添加状态机调试日志
  ROS_INFO_THROTTLE(2.0, "[QrAlignmentTask] 状态=%d 当前位置: x=%.1f y=%.1f z=%.1f yaw=%.1f",
                    static_cast<int>(inspection_state_), x_cm, y_cm, z_cm, yaw_deg);

  if (inspection_state_ != last_state) {
    if (inspection_state_ == InspectionState::NAVIGATE_TO_WAYPOINT) {
      publishPIDMode(0);
    }
    last_state = inspection_state_;
  }

  switch (inspection_state_) {
    case InspectionState::NAVIGATE_TO_WAYPOINT: {
      // 极速模式限制：必须进入目标 30cm 范围内才检查视觉
      double dist_xy = std::hypot(x_cm - t.x_cm, y_cm - t.y_cm);
      double dist_z = std::abs(z_cm - t.z_cm);

      if (dist_xy < 5.0 && dist_z < 5.0) {
        // 如果不是“跳过点”，且看到二维码并对准，则立即射击
          if (!t.skip_processing && isQRCodeDataValid() && isQRCodeAligned()) {
            ROS_INFO("[QrAlignmentTask] 极速模式触发: dist_xy=%.1f (lim 30.0). 视觉已对准 -> 立即射击", dist_xy);
            visual_servo_started_ = false;
            laser_shot_in_progress_ = false;
            inspection_state_ = InspectionState::LASER_SHOOTING;
            break; // 退出 switch
          }
      }

      if (navigator_.isReached(t, x_cm, y_cm, z_cm, yaw_deg)) {
        // 检查是否跳过后续处理
        if (t.skip_processing) {
          ROS_INFO("[QrAlignmentTask] 航点标记为跳过，直接进入下一个");
          inspection_state_ = InspectionState::NEXT_WAYPOINT;
        } else {
          inspection_state_ = InspectionState::PREPARE_VISION_SERVO;
        }
      }
      break;
    }
    case InspectionState::PREPARE_VISION_SERVO:
      ROS_INFO("[QrAlignmentTask] 准备视觉伺服");
      // 任务层决策：需要二维码对准则切换模式
      publishPIDMode(1);
      visual_servo_start_time_ = ros::Time::now();
      visual_servo_started_ = true;
      inspection_state_ = InspectionState::VISUAL_ALIGNMENT;
      break;
    case InspectionState::VISUAL_ALIGNMENT:
      if (checkVisualServoTimeout()) {
        break;
      }
      if (!isQRCodeDataValid()) {
        break;
      }
      // 对准检查：只要在运动过程中进入了误差范围，立即射击
      if (isQRCodeAligned()) {
        ROS_INFO("[QrAlignmentTask] 二维码已进入误差范围，执行射击并前往下一航点");
        visual_servo_started_ = false;
        laser_shot_in_progress_ = false;
        inspection_state_ = InspectionState::LASER_SHOOTING;
      }
      break;
    case InspectionState::LASER_SHOOTING:
      if (!laser_shot_in_progress_) {
        publishLaserControl(true);
        publishPIDMode(0);
        laser_shot_start_time_ = ros::WallTime::now();
        laser_shot_in_progress_ = true;
      }
      if ((ros::WallTime::now() - laser_shot_start_time_).toSec() >= laser_shot_wait_) {
        laser_shot_in_progress_ = false;
        inspection_state_ = InspectionState::NEXT_WAYPOINT;
      }
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
  qr_offset_x_history_.push_back(msg->x);
  qr_offset_y_history_.push_back(msg->y);
  if (qr_offset_x_history_.size() > smooth_window_size_) {
    qr_offset_x_history_.pop_front();
  }
  if (qr_offset_y_history_.size() > smooth_window_size_) {
    qr_offset_y_history_.pop_front();
  }

  double sum_x = 0.0;
  for (double v : qr_offset_x_history_) sum_x += v;
  double sum_y = 0.0;
  for (double v : qr_offset_y_history_) sum_y += v;

  if (!qr_offset_x_history_.empty()) {
    qr_offset_x_ = sum_x / static_cast<double>(qr_offset_x_history_.size());
  }
  if (!qr_offset_y_history_.empty()) {
    qr_offset_y_ = sum_y / static_cast<double>(qr_offset_y_history_.size());
  }

  double error_x = qr_offset_x_ - qr_target_x_;
  double error_y = qr_offset_y_ - qr_target_y_;
  double error_mag = std::sqrt(error_x * error_x + error_y * error_y);
  qr_error_history_.push_back(error_mag);
  std::size_t max_history = trend_window_size_ * 2;
  while (qr_error_history_.size() > max_history) {
    qr_error_history_.pop_front();
  }

  last_qr_update_time_ = ros::Time::now();
  qr_data_valid_ = true;
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
  double error_x = qr_offset_x_ - qr_target_x_;
  double error_y = qr_offset_y_ - qr_target_y_;
  double error_mag = std::sqrt(error_x * error_x + error_y * error_y);
  
  // 仅判断当前误差是否在圈内，移除趋势判断，实现最宽松条件
  bool in_window = (error_mag < qr_align_error_);
  
  // 降低日志频率
  ROS_INFO_THROTTLE(0.5,
                    "[QrAlignmentTask] QR对准: err=%.1f thres=%.1f aligned=%d",
                     error_mag, qr_align_error_, in_window ? 1 : 0);

  return in_window;
}

bool QrAlignmentTask::isErrorTrendDecreasing() const {
  std::size_t required = trend_window_size_ * 2;
  if (qr_error_history_.size() < required || trend_window_size_ == 0) {
    return true;  // 宽松策略：历史不足时直接放行
  }

  double recent_sum = 0.0;
  double prev_sum = 0.0;
  std::size_t start_prev = qr_error_history_.size() - required;
  std::size_t start_recent = qr_error_history_.size() - trend_window_size_;

  for (std::size_t i = 0; i < trend_window_size_; ++i) {
    prev_sum += qr_error_history_[start_prev + i];
    recent_sum += qr_error_history_[start_recent + i];
  }

  double prev_avg = prev_sum / static_cast<double>(trend_window_size_);
  double recent_avg = recent_sum / static_cast<double>(trend_window_size_);
  double denom = std::max(prev_avg, 1e-6);
  double percent_drop = (prev_avg - recent_avg) / denom;

  return percent_drop >= trend_percent_threshold_;
}

// 检查视觉伺服是否超时
bool QrAlignmentTask::checkVisualServoTimeout() {
  double elapsed = (ros::Time::now() - visual_servo_start_time_).toSec();
  if (elapsed > visual_servo_timeout_) {
    ROS_WARN("[QrAlignmentTask] 视觉伺服超时(%.1fs)，尝试打激光后跳过此航点", elapsed);
    visual_servo_started_ = false;
    laser_shot_in_progress_ = false;
    inspection_state_ = InspectionState::LASER_SHOOTING;
    return true;
  }
  return false;
}

// 检查二维码数据是否有效
bool QrAlignmentTask::isQRCodeDataValid() {
  double qr_age = (ros::Time::now() - last_qr_update_time_).toSec();
  if (!qr_data_valid_ || qr_age > qr_data_timeout_) {
    ROS_WARN_THROTTLE(1.0, "[QrAlignmentTask] 二维码数据无效或超时(%.1fs)，等待...", qr_age);
    return false;
  }
  return true;
}

} // namespace activity_control_pkg
