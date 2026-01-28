#include <ros/ros.h>
#include "uart_to_stm32.h"

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "uart_to_stm32_node");
    ros::NodeHandle nh("~");  // 使用私有命名空间
    
    ROS_INFO("Starting uart_to_stm32 node...");
    
    try {
        // 创建主对象
        uart_to_stm32::UartToStm32 node(nh);

        // 初始化（参数：更新频率，源坐标系，目标坐标系）
        if (!node.initialize(100.0, "map", "laser_link")) {
            ROS_ERROR("Failed to initialize UartToStm32");
            return -1;
        }

        // 运行主循环
        node.run();
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return -1;
    }
    
    ROS_INFO("uart_to_stm32 node shutting down");
    return 0;
}