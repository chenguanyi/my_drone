#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>
#include <unencoder/decode.h>
#include <gpiod.h>
#include <set>

// libgpiod 配置
// GPIO1_A4 = gpiochip1, line 4 (物理引脚36, wiringPi编号10)
#define GPIO_CHIP "gpiochip1"
#define LASER_LINE 4
#define LASER_PULSE_DURATION 0.5  // 激光脉冲持续时间（秒）

// libgpiod 全局变量
struct gpiod_chip *chip = nullptr;
struct gpiod_line *laser_line = nullptr;

bool laser_state = false;
geometry_msgs::Point current_qr_position;
uint8_t previous_qr_data = 0;
bool laser_pulse_pending = false;
ros::Timer laser_off_timer;

// 添加QR码数据发布者
ros::Publisher qr_data_pub;

// GPIO设置函数
void gpioSetValue(int value)
{
    if (laser_line) {
        gpiod_line_set_value(laser_line, value);
    }
}

// 判断是否应该发布QR码数据
bool shouldPublishQr(uint8_t current_qr_data, bool qr_valid)
{
    // 只有在数据有效且与上次不同时才发布
    return qr_valid && (current_qr_data != previous_qr_data);
}

// 添加周期性切换IO口电平的函数测试用
void periodicToggle(const ros::TimerEvent&)
{
    laser_state = !laser_state;
    if(laser_state) {
        gpioSetValue(1);
    } else {
        gpioSetValue(0);
    }
}

// 激光拉低后恢复高电平的定时回调（非阻塞）
void laserOffCallback(const ros::TimerEvent&)
{
    gpioSetValue(1);
    laser_pulse_pending = false;
    ROS_INFO("Laser off timer triggered, GPIO set to HIGH");
}

// 触发激光脉冲（非阻塞，自动恢复高电平）
// 返回true表示成功触发，false表示已有脉冲进行中（防重入）
bool triggerLaserPulse()
{
    if (laser_pulse_pending) {
        ROS_WARN("Laser pulse already pending, ignoring trigger");
        return false;
    }
    gpioSetValue(0);
    laser_pulse_pending = true;
    laser_off_timer.stop();
    laser_off_timer.setPeriod(ros::Duration(LASER_PULSE_DURATION));
    laser_off_timer.start();
    return true;
}

// 清理GPIO资源
void cleanupGpio()
{
    if (laser_line) {
        gpiod_line_release(laser_line);
        laser_line = nullptr;
    }
    if (chip) {
        gpiod_chip_close(chip);
        chip = nullptr;
    }
}

// QR码数据回调函数
void qrDataCallback(const unencoder::decode::ConstPtr& msg)
{
    uint8_t current_qr_data = msg->data;
    bool qr_valid = msg->is_valid;

    // 检查该qr_data是否已经发布过
    if(shouldPublishQr(current_qr_data, qr_valid))
    {
        // 注释掉自动射击，改由任务层 QrAlignmentTask 根据对准情况统一控制射击时机
        // if (triggerLaserPulse()) {
            qr_data_pub.publish(*msg);
            ROS_INFO("Detected QR data: %d, (Auto-laser disabled, waiting for alignment)", current_qr_data);
            previous_qr_data = current_qr_data;
        // }
    }
    

}

// QR码位置回调函数
void qrPositionCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    current_qr_position = *msg;
}

// 手动激光控制回调（从任务层触发）
void laserControlCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data && triggerLaserPulse()) {
        ROS_INFO("Manual laser trigger from /laser_control");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_control_node");
    ros::NodeHandle nh;
    
    // 初始化libgpiod
    chip = gpiod_chip_open_by_name(GPIO_CHIP);
    if (!chip) {
        ROS_ERROR("Failed to open GPIO chip: %s", GPIO_CHIP);
        return 1;
    }
    
    laser_line = gpiod_chip_get_line(chip, LASER_LINE);
    if (!laser_line) {
        ROS_ERROR("Failed to get GPIO line %d", LASER_LINE);
        gpiod_chip_close(chip);
        return 1;
    }
    
    // 配置为输出模式，默认高电平
    if (gpiod_line_request_output(laser_line, "laser_control", 1) < 0) {
        ROS_ERROR("Failed to request GPIO line as output");
        gpiod_chip_close(chip);
        return 1;
    }
    
    ROS_INFO("GPIO initialized successfully: chip=%s, line=%d", GPIO_CHIP, LASER_LINE);
    
    // 创建QR码数据发布者
    qr_data_pub = nh.advertise<std_msgs::UInt8>("/processed_qr_data", 10);
    
    
    // 订阅QR码数据和位置话题 这个直接订阅decorder节点
    ros::Subscriber qr_data_sub = nh.subscribe("/qr_code_data", 1, qrDataCallback);
    ros::Subscriber qr_position_sub = nh.subscribe("/qr_code_position", 1, qrPositionCallback);
    
    // 订阅手动激光控制话题（从任务层触发）
    ros::Subscriber laser_control_sub = nh.subscribe("/laser_control", 1, laserControlCallback);
    
    // 创建0.5秒一次性定时器，用于恢复激光高电平
    laser_off_timer = nh.createTimer(ros::Duration(0.5), laserOffCallback, true, false);
    // 创建2秒周期的定时器
    // ros::Timer timer = nh.createTimer(ros::Duration(2.0), periodicToggle);
    
    ros::spin();
    
    // 清理GPIO资源
    cleanupGpio();
    
    return 0;
}