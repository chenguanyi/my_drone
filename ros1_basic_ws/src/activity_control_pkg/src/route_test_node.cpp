// 示例 main：基础导航 + 二维码任务
#include <ros/ros.h>
#include "activity_control_pkg/waypoint_navigator.h"
#include "activity_control_pkg/qr_alignment_task.h"
// 设置进程locale以支持中文日志
#include <clocale>

int main(int argc, char **argv) {
  ros::init(argc, argv, "route_node");

  // 关键一行：跟随环境设置locale
  setlocale(LC_ALL, ""); 

  // 使用私有命名空间句柄
  ros::NodeHandle nh("~");

  const std::string map_frame = "map";
  const std::string laser_link_frame = "laser_link";

  activity_control_pkg::WaypointNavigator navigator(nh, map_frame, laser_link_frame);
  // 构造即运行：任务层内部硬编码航点并启动
  activity_control_pkg::QrAlignmentTask task(nh, navigator);

  ros::spin();
  return 0;
}