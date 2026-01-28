#include "activity_control_pkg/route_target_publisher.h"
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

// 构造：需要 nh、TF 帧名和输出话题
RouteTargetPublisher::RouteTargetPublisher(ros::NodeHandle &nh,
                                           const std::string &map_frame,
                                           const std::string &laser_link_frame,
                                           const std::string &output_topic)
    : nh_(nh),
      output_topic_(output_topic),
      map_frame_(map_frame), 
      laser_link_frame_(laser_link_frame), 
      inspection_state_(NAVIGATE_TO_WAYPOINT), 
      has_qr_detection_(false),
      qr_offset_x_(0.0),
      qr_offset_y_(0.0),
      qr_target_x_(320.0),
      qr_target_y_(170.0)
      {
  // 发布器目标点
  target_position_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(output_topic_, 1, /*latch=*/true);
  // 发布器PID模式
  pid_mode_pub_ = nh_.advertise<std_msgs::UInt8>("/pid_mode", 1,true);
  // 发布器激光控制
  laser_control_pub_ = nh_.advertise<std_msgs::Bool>("/laser_control",1,true);
  // 订阅高度（绝对话题 /height，单位 cm）
  height_sub_ = nh_.subscribe<std_msgs::Int16>("/height", 10, &RouteTargetPublisher::heightCallback, this);
  // 订阅二维码偏移
  qr_code_sub_ = nh_.subscribe<geometry_msgs::Point>("/qr_code_position", 10, &RouteTargetPublisher::qrCodeOffsetCallback, this);

  // 启动定时器，周期检查是否达到当前目标
  monitor_timer_ = nh_.createTimer(ros::Duration(0.05), &RouteTargetPublisher::monitorTimerCallback, this);
}

void RouteTargetPublisher::startsnakepath(int map_size, double square_length, double altitude) { 
    int idx = 0;
    int idy = 0;
    int direction_flag = 0; // 0 -->  1 <--
    
    for (int row = 0; row < map_size; row++) {
      for (int col = 0; col < 2; col++) {
        Target t;
        if (direction_flag == 0) {
          // 从左到右
          if(col == 0)
          {
            idx = 0;
          }else {
            idx = map_size - 1;
          }
        } else {
          // 从右到左
          if(col == 0)
          {
            idx = map_size - 1;
          }else {
            idx = 0;
          }
        }
        idy = row;
        t.x_cm = idx * square_length;
        t.y_cm = idy * square_length;
        t.z_cm = altitude;
        t.qr_valid = true;
        t.yaw_deg = 0.0;
        
        addTarget(t);
        ROS_INFO("[RouteTargetPublisher] 目标点: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg",t.x_cm,t.y_cm,t.z_cm,t.yaw_deg);
      }
      // 切换方向
      direction_flag = 1 - direction_flag;
    }
    
    // 添加下降到地面的目标点
    Target landing1, landing2, landing3;
    landing1.x_cm = (0) * square_length;
    landing1.y_cm = (map_size - 1) * square_length;
    landing1.z_cm = altitude * 0.5;
    landing1.yaw_deg = 0.0;
    landing1.qr_valid = false;
    addTarget(landing1);
    
    landing2.x_cm = (0) * square_length;
    landing2.y_cm = (map_size - 1) * square_length;
    landing2.z_cm = altitude * 0.25;
    landing2.yaw_deg = 0.0;
    landing2.qr_valid = false;
    addTarget(landing2);
    
    landing3.x_cm = (0) * square_length;
    landing3.y_cm = (map_size - 1) * square_length;
    landing3.z_cm = 0.0;
    landing3.yaw_deg = 0.0;
    landing3.qr_valid = false;
    addTarget(landing3);
    
    ROS_INFO("[RouteTargetPublisher] 蛇形巡检路径已生成，共 %zu 个目标点", targets_.size());

}

void RouteTargetPublisher::startplanpath(int num,int map_size_x,int map_size_y,double square_length,double altitude) { 
  int number = std::min(num,map_size_x * map_size_y);
  ROS_INFO("[RouteTargetPublisher] 规划路径，格子数 %d",number);
  int row = number / map_size_x;
  int finalx = number % map_size_x;
  int direction_flag = 0; // 0 -->  1 <--
  int idx = 0;
  int idy = 0;
  for(int i = 0; i < row; i++)
  {
    for(int j = 0; j < 2; j++)
    {
      Target t;
      if (direction_flag == 0) {
        // 从左到右
        if(j == 0)
        {
          idx = 0;
        }else {
          idx = map_size_x - 1;
        }
      }else {
        // 从右到左
        if(j == 0)
        {
          idx = map_size_x - 1;
        }else {
          idx = 0;
        }
      }
      idy = i;
      t.x_cm = idx * square_length;
      t.y_cm = idy * square_length;
      t.z_cm = altitude;
      t.yaw_deg = 0.0;
      t.qr_valid = false;
      addTarget(t);
      ROS_INFO("[RouteTargetPublisher] 目标点: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg",t.x_cm,t.y_cm,t.z_cm,t.yaw_deg);
    }
    direction_flag = 1 - direction_flag;
  }
  if(row % 2 == 0)
  {
    Target t;
    t.x_cm = 0;
    t.y_cm = row * square_length;
    t.z_cm = altitude;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    ROS_INFO("[RouteTargetPublisher] 目标点: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg",t.x_cm,t.y_cm,t.z_cm,t.yaw_deg);
    t.x_cm = (finalx - 1) * square_length;
    t.y_cm = row * square_length;
    t.z_cm = altitude;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    ROS_INFO("[RouteTargetPublisher] 目标点: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg",t.x_cm,t.y_cm,t.z_cm,t.yaw_deg);
  }else 
  {
    Target t;
    t.x_cm = (map_size_x - 1) * square_length;
    t.y_cm = row * square_length;
    t.z_cm = altitude;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    ROS_INFO("[RouteTargetPublisher] 目标点: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg",t.x_cm,t.y_cm,t.z_cm,t.yaw_deg);
    t.x_cm = (map_size_x - finalx + 1) * square_length;
    t.y_cm = row * square_length;
    t.z_cm = altitude;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    ROS_INFO("[RouteTargetPublisher] 目标点: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg",t.x_cm,t.y_cm,t.z_cm,t.yaw_deg);
  }

  // 添加下降到地面的目标点
  Target landing1, landing2, landing3, landing4;
  landing1.x_cm = 0;
  landing1.y_cm = 0;
  landing1.z_cm = altitude;
  landing1.yaw_deg = 0.0;
  landing1.qr_valid = false;
  addTarget(landing1);//返航
  ROS_INFO("[RouteTargetPublisher] 目标点: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg",landing1.x_cm,landing1.y_cm,landing1.z_cm,landing1.yaw_deg);

  landing2.x_cm = 0;
  landing2.y_cm = 0;
  landing2.z_cm = altitude * 0.5;
  landing2.yaw_deg = 0.0;
  landing2.qr_valid = false;
  addTarget(landing2);
  ROS_INFO("[RouteTargetPublisher] 目标点: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg",landing2.x_cm,landing2.y_cm,landing2.z_cm,landing2.yaw_deg);

  landing3.x_cm = 0;
  landing3.y_cm = 0;
  landing3.z_cm = altitude * 0.25;
  landing3.yaw_deg = 0.0;
  landing3.qr_valid = false;
  addTarget(landing3);
  ROS_INFO("[RouteTargetPublisher] 目标点: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg",landing3.x_cm,landing3.y_cm,landing3.z_cm,landing3.yaw_deg);

  landing4.x_cm = 0;
  landing4.y_cm = 0;
  landing4.z_cm = altitude * 0;
  landing4.yaw_deg = 0.0;
  landing4.qr_valid = false;
  addTarget(landing4);
  ROS_INFO("[RouteTargetPublisher] 目标点: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg",landing4.x_cm,landing4.y_cm,landing4.z_cm,landing4.yaw_deg);
  ROS_INFO("[RouteTargetPublisher] 蛇形巡检路径已生成，共 %zu 个目标点", targets_.size());
   
}

void RouteTargetPublisher::Pathplan_24Race()
{
    Target t;
    t.x_cm = 0.0;
    t.y_cm = 0.0;
    t.z_cm = 125.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = -80.0;
    t.y_cm = 0.0;
    t.z_cm = 125.0;
    t.yaw_deg = 0.0;
    t.qr_valid = true;
    addTarget(t);
    t.x_cm = -140.0;
    t.y_cm = 0.0;
    t.z_cm = 125.0;
    t.yaw_deg = 0.0;
    t.qr_valid = true;
    addTarget(t);
    t.x_cm = -190.0;
    t.y_cm = 0.0;
    t.z_cm = 125.0;
    t.yaw_deg = 0.0;
    t.qr_valid = true;
    addTarget(t);
    t.x_cm = -190.0;
    t.y_cm = 0.0;
    t.z_cm = 85.0;
    t.yaw_deg = 0.0;
    t.qr_valid = true;
    addTarget(t);
    t.x_cm = -140.0;
    t.y_cm = 0.0;
    t.z_cm = 85.0;
    t.yaw_deg = 0.0;
    t.qr_valid = true;
    addTarget(t);
    t.x_cm = -80.0;
    t.y_cm = 0.0;
    t.z_cm = 85.0;
    t.yaw_deg = 0.0;
    t.qr_valid = true;
    addTarget(t);
    t.x_cm = 0.0;
    t.y_cm = 0.0;
    t.z_cm = 30.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    ROS_INFO("[RouteTargetPublisher] 24Race路径已生成，共 %zu 个目标点", targets_.size());
}

void RouteTargetPublisher::Pathplan_24Race2()
{
    Target t;
    t.x_cm = 0.0;
    t.y_cm = 0.0;
    t.z_cm = 133.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    
    t.x_cm = -125.0;//A面
    t.y_cm = 0.0;
    t.z_cm = 132.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = -174.0;
    t.y_cm = 0.0;
    t.z_cm = 132.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = -222.0;
    t.y_cm = 0.0;
    t.z_cm = 132.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    
    t.x_cm = -222.0;
    t.y_cm = 0.0;
    t.z_cm = 92.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = -165.0;
    t.y_cm = 0.0;
    t.z_cm = 92.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = -110.0;
    t.y_cm = 0.0;
    t.z_cm = 92.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);

    t.x_cm = 0.0;
    t.y_cm = 0.0;
    t.z_cm = 92.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = 0.0;
    t.y_cm = 135.0;
    t.z_cm = 83.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);

    t.x_cm = -115.0;//C面
    t.y_cm = 135.0;
    t.z_cm = 83.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = -170.0;
    t.y_cm = 135.0;
    t.z_cm = 83.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = -213.0;
    t.y_cm = 135.0;
    t.z_cm = 83.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);

    t.x_cm = -213.0;
    t.y_cm = 135.0;
    t.z_cm = 126.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = -158.0;
    t.y_cm = 135.0;
    t.z_cm = 126.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = -110.0;
    t.y_cm = 135.0;
    t.z_cm = 126.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;    
    addTarget(t);

    t.x_cm = -110.0;
    t.y_cm = 150.0;
    t.z_cm = 132.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);

    t.x_cm = -110.0;//B面
    t.y_cm = 150.0;
    t.z_cm = 132.0;
    t.yaw_deg = 180.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = -170.0;
    t.y_cm = 150.0;
    t.z_cm = 132.0;
    t.yaw_deg = 180.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = -222.0;
    t.y_cm = 150.0;
    t.z_cm = 132.0;
    t.yaw_deg = 180.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = -222.0;
    t.y_cm = 150.0;
    t.z_cm = 91.0;
    t.yaw_deg = 180.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = -160.0;
    t.y_cm = 150.0;
    t.z_cm = 91.0;
    t.yaw_deg = 180.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = -105.0;
    t.y_cm = 150.0;
    t.z_cm = 91.0;
    t.yaw_deg = 180.0;
    t.qr_valid = false;
    addTarget(t);

    t.x_cm = 0.0;
    t.y_cm = 150.0;
    t.z_cm = 91.0;
    t.yaw_deg = 180.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = 0.0;
    t.y_cm = 150.0;
    t.z_cm = 40.0;
    t.yaw_deg = 180.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = 0.0;
    t.y_cm = 150.0;
    t.z_cm = 4.0;
    t.yaw_deg = 180.0;
    t.qr_valid = false;
    addTarget(t);

    // t.x_cm = 0.0;
    // t.y_cm = 300.0;
    // t.z_cm = 83.0;
    // t.yaw_deg = 180.0;
    // t.qr_valid = false;
    // addTarget(t);

    // t.x_cm = -115.0;//D面
    // t.y_cm = 300.0;
    // t.z_cm = 83.0;
    // t.yaw_deg = 180.0;
    // t.qr_valid = false;
    // addTarget(t);
    // t.x_cm = -170.0;
    // t.y_cm = 300.0;
    // t.z_cm = 83.0;
    // t.yaw_deg = 180.0;
    // t.qr_valid = false;
    // addTarget(t);
    // t.x_cm = -222.0;
    // t.y_cm = 300.0;
    // t.z_cm = 83.0;
    // t.yaw_deg = 180.0;
    // t.qr_valid = false;
    // addTarget(t);
    // t.x_cm = -222.0;
    // t.y_cm = 300.0;
    // t.z_cm = 124.0;
    // t.yaw_deg = 180.0;
    // t.qr_valid = false;
    // addTarget(t);
    // t.x_cm = -160.0;
    // t.y_cm = 300.0;
    // t.z_cm = 124.0;
    // t.yaw_deg = 180.0;
    // t.qr_valid = false;
    // addTarget(t);
    // t.x_cm = -105.0;
    // t.y_cm = 300.0;
    // t.z_cm = 124.0;
    // t.yaw_deg = 180.0;
    // t.qr_valid = false;
    // addTarget(t);

    // t.x_cm = -220.0;
    // t.y_cm = 300.0;
    // t.z_cm = 124.0;
    // t.yaw_deg = 180.0;
    // t.qr_valid = false;
    // addTarget(t);
    // t.x_cm = -220.0;
    // t.y_cm = 300.0;
    // t.z_cm = 40.0;
    // t.yaw_deg = 180.0;
    // t.qr_valid = false;
    // addTarget(t);
    // t.x_cm = -220.0;
    // t.y_cm = 300.0;
    // t.z_cm = 6.0;
    // t.yaw_deg = 180.0;
    // t.qr_valid = false;
    // addTarget(t);


    ROS_INFO("[RouteTargetPublisher] 24Race路径已生成，共 %zu 个目标点", targets_.size());
}

void RouteTargetPublisher::PathTest()
{
    Target t;
    t.x_cm = 0.0;
    t.y_cm = 0.0;
    t.z_cm = 125.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = -220.0;
    t.y_cm = 0.0;
    t.z_cm = 125.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = -220.0;
    t.y_cm = 0.0;
    t.z_cm = 50.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    t.x_cm = -220.0;
    t.y_cm = 0.0;
    t.z_cm = 5.0;
    t.yaw_deg = 0.0;
    t.qr_valid = false;
    addTarget(t);
    ROS_INFO("finish plan");
}

void RouteTargetPublisher::zixuanTest()
{
  Target t;
  t.x_cm = 0.0;
  t.y_cm = 0.0;
  t.z_cm = 30.0;
  t.yaw_deg = 0.0;
  t.qr_valid = false;
  addTarget(t);
  t.x_cm = 0.0;
  t.y_cm = 0.0;
  t.z_cm = 30.0;
  t.yaw_deg = 180.0;
  t.qr_valid = false;
  addTarget(t);
  t.x_cm = 0.0;
  t.y_cm = 0.0;
  t.z_cm = 5.0;
  t.yaw_deg = 180.0;
  t.qr_valid = false;
  addTarget(t);
  ROS_INFO("[RouteTargetPublisher] 测试路径已生成，共 %zu 个目标点", targets_.size());
}

// 运行时追加新目标；若这是第一个目标，立即发布
void RouteTargetPublisher::addTarget(const Target &t) {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_empty = targets_.empty();
  targets_.push_back(t);//添加到向量末尾
    if (was_empty) {
    current_idx_ = 0;
    publishCurrent();
  }
}

// 发布当前目标（如果存在）
void RouteTargetPublisher::publishCurrent() {
  if (current_idx_ != static_cast<std::size_t>(-1) && current_idx_ < targets_.size()) {
    publishTarget(targets_[current_idx_], current_idx_ == 0);
  }
}

void RouteTargetPublisher::publishFinal()//巡航完了发送停止命令
{
  Target landing1;
  landing1.x_cm = -1;
  landing1.y_cm = -1;
  landing1.z_cm = -1;
  landing1.yaw_deg = -1;
  publishTarget(landing1,current_idx_ == 0);
}

// 实际发布到话题
void RouteTargetPublisher::publishTarget(const Target &t, bool init_flag) {
  std_msgs::Float32MultiArray out;
  out.data.resize(4);
  out.data[0] = static_cast<float>(t.x_cm);
  out.data[1] = static_cast<float>(t.y_cm);
  out.data[2] = static_cast<float>(t.z_cm);
  out.data[3] = static_cast<float>(t.yaw_deg);
  target_position_pub_.publish(out);
  ROS_INFO("[RouteTargetPublisher] 发布目标: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg%s",
           t.x_cm, t.y_cm, t.z_cm, t.yaw_deg, init_flag ? " (首个)" : "");
}

// 查询当前索引（0 起始）；无目标时返回 size_t(-1)
std::size_t RouteTargetPublisher::currentIndex() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_idx_;
}

// 当前目标总数
std::size_t RouteTargetPublisher::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return targets_.size();
}

// 高度回调
void RouteTargetPublisher::heightCallback(const std_msgs::Int16::ConstPtr &msg) {
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

// 获取实时位姿：x/y/yaw 来自 TF(map->laser_link)，z 来自 /height
bool RouteTargetPublisher::getCurrentPose(double &x_cm, double &y_cm, double &z_cm, double &yaw_deg) {
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

// 判断是否到达目标（xy 平面距离、z 高度、yaw 航向）
bool RouteTargetPublisher::isReached(const Target &t, double x_cm, double y_cm, double z_cm, double yaw_deg) {
  double dx = t.x_cm - x_cm;
  double dy = t.y_cm - y_cm;
  double dxy = std::sqrt(dx * dx + dy * dy);
  double dz = t.z_cm - z_cm;
  double dyaw = normalizeAngleDeg(t.yaw_deg - yaw_deg);
  bool z_ok = has_height_ ? (std::fabs(dz) <= height_tol_cm_) : false;
  return z_ok && (dxy <= pos_tol_cm_) && (std::fabs(dyaw) <= yaw_tol_deg_);
}

// 定时检查：到达则推进到下一个目标
void RouteTargetPublisher::monitorTimerCallback(const ros::TimerEvent &) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_idx_ == static_cast<std::size_t>(-1) || current_idx_ >= targets_.size()) return;
  double x_cm, y_cm, z_cm, yaw_deg;
  if (!getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) return;
  const Target &t = targets_[current_idx_];
  switch (inspection_state_)
  {
  case NAVIGATE_TO_WAYPOINT:
    if (isReached(t, x_cm, y_cm, z_cm, yaw_deg))
    {
      inspection_state_ = PREPARE_VISION_SERVO;
    }
    break;
  case PREPARE_VISION_SERVO:
    if(t.qr_valid == true)
    {
      publishPIDMode(1);
      inspection_state_ = VISUAL_ALTGNMENT;
    }else 
    {
      inspection_state_ = NEXT_WAYPOINT;
    }
    break;
  case VISUAL_ALTGNMENT:
     if(isQRCodeAligned())
    {
      inspection_state_ = LASER_SHOOTING;
    }
    break;
  case LASER_SHOOTING:
    {
      publishLaserControl(true);
      inspection_state_ = NEXT_WAYPOINT;
      publishPIDMode(0);
    }
    break;
  case NEXT_WAYPOINT:
    ROS_INFO("[RouteTargetPublisher] 目标 %zu 达成，准备发布下一个", current_idx_);
    current_idx_++;
    if (current_idx_ < targets_.size()) {
      publishCurrent();
    } else {
      publishFinal();
      ROS_INFO("[RouteTargetPublisher] 所有目标已完成");
    }
    inspection_state_ = NAVIGATE_TO_WAYPOINT;
    break;
  }
  ROS_INFO_THROTTLE(1.0,"Current mode %d",inspection_state_);
  ROS_INFO("Current pos %.1fcm %.1fcm %.1fcm %.1fdeg",x_cm,y_cm,z_cm,yaw_deg);
  ROS_INFO_THROTTLE(1.0,"Current idx %ld",current_idx_);
  
  // if (isReached(t, x_cm, y_cm, z_cm, yaw_deg)) {
  //   ROS_INFO("[RouteTargetPublisher] 目标 %zu 达成，准备发布下一个", current_idx_);
  //   current_idx_++;
  //   if (current_idx_ < targets_.size()) {
  //     publishCurrent();
  //   } else {
  //     publisFinal();
  //     ROS_INFO("[RouteTargetPublisher] 所有目标已完成");
  //   }
  // }
}

void RouteTargetPublisher::qrCodeOffsetCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  qr_offset_x_ = msg->x;
  qr_offset_x_ = msg->y;
}

void RouteTargetPublisher::publishLaserControl(bool enable)
{
  std_msgs::Bool msg;
  msg.data = enable;
  laser_control_pub_.publish(msg);
}

void RouteTargetPublisher::publishPIDMode(uint8_t mode)
{
  std_msgs::UInt8 msg;
  msg.data = mode;
  pid_mode_pub_.publish(msg);
}

bool RouteTargetPublisher::isQRCodeAligned()
{
  bool ret = false;
  if(abs(qr_offset_x_ - qr_target_x_) < qr_align_error_ && abs(qr_offset_y_ - qr_target_y_) < qr_align_error_)
  {
    ret = true; 
  }
  return ret;
}

} // namespace activity_control_pkg
