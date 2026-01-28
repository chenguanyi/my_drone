/*
这个文件主要做了什么：
实现和地面站的双向数据同行
同时发送的数据和接收的数据处理也在这个文件中完成
对于现在这个项目具体做了什么：
接受来自地面站的航点数量信息并发布
订阅电压信息并计算自己的巡航里程发送给地面站
订阅yolo识别信息和，高度信息和tf树计算目标在地图中的位置发送给地面站
*/
#include <ros/ros.h>
#include <serial_comm/serial_comm.h>
// 添加缺失的头文件
#include <iomanip>
#include <sstream>
#include <limits>
// 添加智能指针头文件
#include <memory>
// 添加消息类型头文件
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <yolo/yolo.h>

struct cma_inparam{
    double fx;//焦距x
    double fy;//焦距y
    double cx;//光学中心x
    double cy;//光学中心y
};

struct RobotPosition {
    float x;  // x坐标（厘米）
    float y;  // y坐标（厘米）
    double yaw; // 角度
    bool valid; // 位置是否有效
};

struct WorldPosition {
    double x;  // x坐标（厘米）
    double y;  // y坐标（厘米）
};

void pixelToWorldWithDepth(int u, int v, double Z, cma_inparam params,RobotPosition* pos) {
    // 计算归一化坐标（相对于主点的偏移）
    double dx = (u - params.cx) / params.fx;  // (u - cx) / fx
    double dy = (v - params.cy) / params.fy;  // (v - cy) / fy
    
    // 通过深度Z计算真实坐标并进行简单的坐标轴转化
    pos->x = dx * Z;
    pos->y = -dy * Z;
    pos->valid = true;
}

void protocol_handler(ros::Publisher& num_pub,uint8_t id, const std::vector<uint8_t>& data)
{
    std::stringstream ss;
    ss << "Received protocol frame - ID: 0x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(id) 
    << std::dec << ", Data Length: " << data.size() << ", Data: ";
    for (size_t i = 0; i < data.size(); ++i) {
        ss << static_cast<int>(data[i]) << " ";
    }
    ROS_INFO("%s", ss.str().c_str());
    
    switch(id)
    {
        case 0xF1:
        {
            // num_square
            std_msgs::UInt8 msg;
            msg.data = data[0];
            ROS_INFO("Received data: %d", msg.data);
            num_pub.publish(msg);
            break;
        }
    }
}
double meterToCm(double meter) { return meter * 100.0; }
void timer_callback(const ros::TimerEvent& event, tf::TransformListener& listener,RobotPosition* robot_position)
{
    try
    {
        tf::StampedTransform transform;
        listener.lookupTransform("map", "laser_link", ros::Time(0), transform);
        robot_position->x = meterToCm(transform.getOrigin().x());
        robot_position->y = meterToCm(transform.getOrigin().y());
        tf::Quaternion q = transform.getRotation();
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        robot_position->yaw = yaw;
        robot_position->valid = true;
        ROS_INFO_THROTTLE(1.0,"x_cm:%f, y_cm %f",robot_position->x,robot_position->y);
    }
    catch(const std::exception& e)
    {
        ROS_WARN("Failed to get transform");
        robot_position->valid = false; // 确保在失败时标记为无效
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "bluetooth_uart_test");
    ros::NodeHandle nh;
    // 创建发布者，用于发布接收到的数据
    ros::Publisher bluetooth_data_pub = nh.advertise<std_msgs::UInt8>("/bluetooth_data", 10);
    ros::Publisher num_pub = nh.advertise<std_msgs::UInt8>("/square_num", 10);
    ROS_INFO("Starting BluetoothUart test...");

    cma_inparam came1;
    came1.fx = 1103;
    came1.fy = 1100;
    came1.cx = 299.162;
    came1.cy = 207.625;

    // 修改: 正确初始化二维数组，确保所有元素都被设置为0
    uint8_t whole_map[9][7] = {{0}};
    // 修改: 初始化高度值，避免未定义行为
    int16_t high1 = 0; // 高度

    uint8_t index_x;
    uint8_t index_y;
    // bool send_flag = false;
    bool battle_flag = true;

    RobotPosition current_pos;//tf「当前位置」lidar位置
    RobotPosition current_pos2;//相机计算出的偏差
    
    WorldPosition final_pos;
    current_pos.valid = false;
    current_pos2.valid = false; // 初始化为无效状态

    tf::TransformListener listener;
    // 使用智能指针初始化serial类
    auto bluetooth_uart = std::make_unique<serial_comm::SerialComm>();
    //创建定时器实现tf查询
    ros::Timer timer = nh.createTimer(ros::Duration(0.01), [&listener,&current_pos](const ros::TimerEvent& event){
     timer_callback(event,listener,&current_pos);
     });

    
    try
    {
        if(!bluetooth_uart->initialize("/dev/ttyS2", 115200))
        {
            ROS_ERROR("Failed to initialize BluetoothUart");
            return -1;
        }
        
        if (!bluetooth_uart->is_open()) {
            ROS_ERROR("Bluetooth UART is not connected");
            return -1;
        }
        

        //订阅yolo识别信息
        // ros::Subscriber yolo_sub = nh.subscribe<yolo::yolo>("/yolo",1,
        // [&](const yolo::yolo::ConstPtr& msg){
        //     uint16_t cen_x = msg->x;
        //     uint16_t cen_y = msg->y;
        //     // 修改: 添加对高度值的检查
        //     if (high1 <= 0) {
        //         ROS_WARN_THROTTLE(1.0, "Invalid height value: %d", high1);
        //         return;
        //     }
            
        //     pixelToWorldWithDepth(cen_x, cen_y, high1, came1, &current_pos2);
            
        //     // 修改: 确保所有位置数据都有效才进行计算
        //     if(current_pos.valid && current_pos2.valid && high1 > 0)
        //     {
        //         final_pos.x = current_pos2.x + current_pos.x;
        //         final_pos.y = current_pos2.y + current_pos.y;

        //         index_x = final_pos.x / 50;
        //         index_y = final_pos.y / 50;

        //         if(whole_map[index_y][index_x] == 0)
        //         {
        //             // whole_map[index_y][index_x] = 1;//这个一定要放在前面
        //             // send_flag = true;
        //             for(int i; i< 1000;i++)
        //             {
        //                 bluetooth_uart->send_protocol_data(0xF2, 2, {index_x,index_y});//串口发给地面站目标位置
        //                 usleep(1000); // 添加1毫秒延时
        //             }
        //             ROS_INFO("send_zuobiao_x:%.f,y:%.f",final_pos.x,final_pos.y);
        //         }
        //     } else {
        //         ROS_WARN_THROTTLE(1.0, "Position data not valid. current_pos.valid: %d, current_pos2.valid: %d, high1: %d", 
        //                           current_pos.valid, current_pos2.valid, high1);
        //     }
            
        //     // ROS_INFO("lidar_x:%.f,lidar_y:%.f,camera_x:%.f,camera_y:%.f",
        //     //                   current_pos.x, current_pos.y, 
        //     //                   current_pos2.valid ? current_pos2.x : std::numeric_limits<double>::quiet_NaN(),
        //     //                   current_pos2.valid ? current_pos2.y : std::numeric_limits<double>::quiet_NaN());
        // }
        // );
        //订阅电压数据
        ros::Subscriber battle = nh.subscribe<std_msgs::Float32>("/battle_voltage",1,
        [&](const std_msgs::Float32::ConstPtr& msg){
            // 修改: 添加对电压值的边界检查
            if (msg->data < 15.5 || msg->data > 17.5) { // 假设合理电压范围是15.5-17.5V
                ROS_WARN("Voltage out of expected range: %.2fV", msg->data);
                return;
            }

            // if(battle_flag == true)
            // {
                uint8_t num = static_cast<uint8_t>((msg->data - 15.5) / 0.02);
                // for(uint64_t i = 0;i< 5000;i++)
                // {
                    bluetooth_uart->send_protocol_data(0xF1, 1, {num});//串口发给地面站可巡航的格数
                    ROS_INFO_THROTTLE(1.0,"num:%d",num);
                //     usleep(1000); // 添加1毫秒延时
                // }
                // battle_flag = false;
            // }
        }
        );      
        //订阅高度信息
        ros::Subscriber heigh = nh.subscribe<std_msgs::Int16>("/height",1,
        [&](const std_msgs::Int16::ConstPtr& msg){
            high1 = msg->data;
        }
        );
        //地面站发送的数据解析数据
        bluetooth_uart->start_protocol_receive([&num_pub](uint8_t id, const std::vector<uint8_t>& data){
            protocol_handler(num_pub,id, data);
        },
        [](const std::string& error) {
            ROS_WARN("Protocol receive error: %s", error.c_str());
        }
        ); 

        ros::spin();
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Exception in main: %s", e.what());
        return -1;
    }
    // 添加: 确保正确关闭连接
    catch (...) {
        ROS_ERROR("Unknown exception occurred");
        return -1;
    }
    
    ROS_INFO("bluetooth_uart_test node shutting down"); // 修正节点名称
    return 0;
}