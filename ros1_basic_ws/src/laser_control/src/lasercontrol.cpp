#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>
#include <unencoder/decode.h>
#include <wiringPi.h>
#include <set>

#define LASER_PIN 10

bool laser_state = false;
geometry_msgs::Point current_qr_position;
uint8_t previous_qr_data = 0;

// 添加QR码数据发布者
ros::Publisher qr_data_pub;


// 添加周期性切换IO口电平的函数测试用
void periodicToggle(const ros::TimerEvent&)
{
    laser_state = !laser_state;
    if(laser_state) {
        digitalWrite(LASER_PIN, HIGH);
    } else {
        digitalWrite(LASER_PIN, LOW);
    }
}

// QR码数据回调函数
void qrDataCallback(const unencoder::decode::ConstPtr& msg)
{
    uint8_t current_qr_data = msg->data;
    bool qr_valid = msg->is_valid;

     // 检查该qr_data是否已经发布过
        if(current_qr_position.x > 180 && current_qr_position.x < 460 &&  current_qr_position.y < 400) {
            if(current_qr_data != previous_qr_data && qr_valid)
            {
                // 拉低引脚0.5秒
                digitalWrite(LASER_PIN, LOW);
                delay(500);
                digitalWrite(LASER_PIN, HIGH);
                qr_data_pub.publish(*msg);
                ROS_INFO("Published QR data: %d", current_qr_data);
                previous_qr_data = current_qr_data;
            }
        }
    

}

// QR码位置回调函数
void qrPositionCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    current_qr_position = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_control_node");
    ros::NodeHandle nh;
    
    // 初始化wiringPi
    if(wiringPiSetup() == -1) {
        ROS_ERROR("Failed to setup wiringPi");
        return 1;
    }
    
    // 设置引脚模式为输出，默认高电平
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, HIGH);
    
    // 创建QR码数据发布者
    qr_data_pub = nh.advertise<std_msgs::UInt8>("/processed_qr_data", 10);
    
    
    // 订阅QR码数据和位置话题 这个直接订阅decorder节点
    ros::Subscriber qr_data_sub = nh.subscribe("/qr_code_data", 1, qrDataCallback);
    ros::Subscriber qr_position_sub = nh.subscribe("/qr_code_position", 1, qrPositionCallback);
    
    // 创建2秒周期的定时器
    // ros::Timer timer = nh.createTimer(ros::Duration(2.0), periodicToggle);
    
    ros::spin();
    
    return 0;
}