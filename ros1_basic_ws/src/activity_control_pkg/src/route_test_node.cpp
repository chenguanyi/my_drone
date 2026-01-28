// 示例 main：随时追加目标点，并可查询当前执行到哪个目标
#include <ros/ros.h>
#include "activity_control_pkg/route_target_publisher.h"
// 新增：设置进程locale以支持中文日志
#include <clocale>
// 新增：订阅 is_st_ready
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "route_node");

  // 关键一行：跟随环境设置locale（需确保环境是 zh_CN.UTF-8 或 en_US.UTF-8）
  setlocale(LC_ALL, "");  // 只需这一行

  ros::NodeHandle nh("~");

  const std::string map_frame = "map";
  const std::string laser_link_frame = "laser_link";

  activity_control_pkg::RouteTargetPublisher node(nh, map_frame, laser_link_frame);
  
  bool started = false;  // 是否已开始
  uint8_t is_ready;      // 飞控是否准备就绪
  uint8_t square_num;    // 航点个数 

  // 订阅 /is_st_ready：当收到 1 时，添加首个目标并启动定时器
  ros::Subscriber ready_sub = nh.subscribe<std_msgs::UInt8>("/is_st_ready", 10,
    [&](const std_msgs::UInt8::ConstPtr& msg_ready)
     {
      // is_ready = msg_ready->data;
      if(msg_ready->data > 0 && !started) {
        node.Pathplan_24Race2();
        started = true;
      }
     }
  );
  // node.Pathplan_24Race2();
  // ros::Subscriber square_num_sub = nh.subscribe<std_msgs::UInt8>("/square_num", 10,
  // [&](const std_msgs::UInt8::ConstPtr& msg)
  // {
  //     if(msg->data > 0 && !started && is_ready == 1) {
  //       node.Pathplan_24Race2();
  //       started = true;
  //     }
  // }
  // );

  ros::spin();
  return 0;
}