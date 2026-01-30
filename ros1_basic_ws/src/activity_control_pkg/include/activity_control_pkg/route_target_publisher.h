#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <string>
#include <vector>
#include <mutex>

namespace activity_control_pkg {

// 目标点（单位：cm 与 度）
struct Target {
  double x_cm;     // 目标 X（cm）
  double y_cm;     // 目标 Y（cm）
  double z_cm;     // 目标 Z（cm）
  double yaw_deg;  // 航向角（度）
  bool qr_valid;
};

enum InspectionState{
  NAVIGATE_TO_WAYPOINT = 0,   //导航到下一个点
  PREPARE_VISION_SERVO = 1,   //准备视觉伺服
  VISUAL_ALTGNMENT = 2,       //视觉对准
  LASER_SHOOTING = 3,         //激光开火
  NEXT_WAYPOINT =4            //前往下一个点
};

class RouteTargetPublisher {
public:
  // 构造：需要节点句柄与（可选）坐标系、输出话题
  // - nh 可以是私有句柄（ros::NodeHandle nh("~")）或全局句柄
  // - map_frame 与 laser_link_frame 用于 TF 查询实时位姿
  // - output_topic 为发布目标位置的话题（单位：cm/度）
  explicit RouteTargetPublisher(ros::NodeHandle &nh,
                                const std::string &map_frame = "map",
                                const std::string &laser_link_frame = "laser_link",
                                const std::string &output_topic = "/target_position");

  // 在运行过程中追加新的目标点；若这是第一个目标，会立即发布
  void addTarget(double x, double y, double z, double yaw);

  // 获取当前正在执行的目标索引（0 起始）；当尚无目标时返回 size_t(-1)
  std::size_t currentIndex() const;

  // 当前目标总数
  std::size_t size() const;

  //路径规划
  void cgyTest();//测试用例
  void Pathplan_24();//24点巡航路径规划

  // 分段降落功能
  void startSegmentedLanding();

private:

  void publishTarget(const Target &t, bool init_flag);//发布目标点
  void publishCurrent();//发布当前目标点
  void publishFinal();//巡航完发布降落命令

  // 位姿获取与到达判断
  bool getCurrentPose(double &x_cm, double &y_cm, double &z_cm, double &yaw_deg);
  bool isReached(const Target &t, double x_cm, double y_cm, double z_cm, double yaw_deg);

  // 定时器：周期检查状态转换
  void monitorTimerCallback(const ros::TimerEvent &);

  // ROS 资源
  ros::NodeHandle nh_;
  ros::Publisher target_position_pub_;//目标点发送
  ros::Publisher pid_mode_pub_;//pid模式发送
  ros::Publisher laser_control_pub_;//激光控制
  ros::Subscriber height_sub_;//高度获取
  ros::Subscriber qr_code_sub_;//二维码像素坐标获取
  tf::TransformListener listener_;//tf监听器
  ros::Timer monitor_timer_;//定时器

  //地图数据
  int map_size_x_ = 7;//x轴格数
  int map_size_y_ =9;//y轴格数
  double square_length_ = 50;//方格长度
  float lowest_vol = 15.5;//最低电压

  // 状态机状态
  InspectionState inspection_state_;
  // 二维码
  bool has_qr_detection_;//是否有二维码检测
  double qr_offset_x_;//x偏移
  double qr_offset_y_;//y偏移
  double qr_target_x_;//目标中心x
  double qr_target_y_;//目标中心y
  double qr_align_error_{20.0};//二维码对准容差

  // 状态
  std::string output_topic_;//发送目标点话题名称
  std::string map_frame_;//地图坐标系名称
  std::string laser_link_frame_;//雷达坐标系名称
  std::vector<Target> targets_;//目标点存储缓冲区
  mutable std::mutex mutex_;//互斥锁
  std::size_t current_idx_{static_cast<std::size_t>(-1)}; //当前目标索引

  // 阈值（单位：cm 与 度）
  double pos_tol_cm_{4.0};
  double yaw_tol_deg_{5.0};
  double height_tol_cm_{3.0};

  // 高度状态
  bool has_height_{false};
  double current_height_cm_{0.0};

  // 订阅高度
  void heightCallback(const std_msgs::Int16::ConstPtr &msg);
  // 订阅二维码偏移
  void qrCodeOffsetCallback(const geometry_msgs::Point::ConstPtr& msg);
  // 发送激光控制
  void publishLaserControl(bool enable);
  // 发送PID模式
  void publishPIDMode(uint8_t mode);
  // 判断二维码对准
  bool isQRCodeAligned();

};

} // namespace activity_control_pkg
 
