#ifndef UART_TO_STM32_H
#define UART_TO_STM32_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <serial_comm/serial_comm.h>
#include <Eigen/Dense>
#include <memory>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>
#include <map>

namespace uart_to_stm32 {

/**
 * @brief UART到STM32通信类
 * 集成ROS话题订阅、坐标转换和串口通信功能
 */
class UartToStm32 {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    explicit UartToStm32(ros::NodeHandle& nh);

    /**
     * @brief 析构函数 用来清理资源
     */
    ~UartToStm32();

    /**
     * @brief 初始化所有订阅器
     * @param update_rate TF更新频率(Hz)
     * @param source_frame 源坐标系
     * @param target_frame 目标坐标系
     * @return true 初始化成功，false 初始化失败
     */
    bool initialize(double update_rate, const std::string& source_frame, const std::string& target_frame);

    /**
     * @brief 运行主循环，持续监听各种话题
     */
    void run();

private:
    /**
     * @brief 查找并处理map到laser_link的变换
     */
    void lookupTransform();

    /**
     * @brief 处理获取到的TF变换数据
     * @param transform 变换数据
     */
    void processTfTransform(const geometry_msgs::TransformStamped& transform);

    /**
     * @brief velocity_map话题回调函数
     * @param msg 接收到的Twist消息
     */
    void velocityMapCallback(const geometry_msgs::Twist::ConstPtr& msg);

    /**
     * @brief target_velocity话题回调函数
     * @param msg 接收到的Float32MultiArray消息 [vx_cm/s, vy_cm/s, vz_cm/s, vyaw_deg/s]
     */
    void targetVelocityCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

    /**
     * @brief 使用TF角度对线速度进行坐标变换
     * @param linear 原始线速度向量
     * @param yaw 从TF获取的yaw角度
     * @return 变换后的线速度向量
     */
    Eigen::Vector3d transformVelocity(const Eigen::Vector3d& linear, double yaw);

    /**
     * @brief 发送转换后的速度数据到串口
     * @param transformed_velocity 转换后的速度向量
     */
    void sendVelocityToSerial(const Eigen::Vector3d& transformed_velocity);

    /**
     * @brief 发送目标速度数据到串口 (四个int16_t值)
     * @param vx_cm_per_s X轴速度 (cm/s)
     * @param vy_cm_per_s Y轴速度 (cm/s)  
     * @param vz_cm_per_s Z轴速度 (cm/s)
     * @param vyaw_deg_per_s Yaw角速度 (deg/s)
     */
    void sendTargetVelocityToSerial(float vx_cm_per_s, float vy_cm_per_s, float vz_cm_per_s, float vyaw_deg_per_s);

    /**
     * @brief 发送 A2 应答帧（长度9，首字节0x01，其余0）
     */
    void sendA2ReadyResponse();

    void qrDataCallback(const std_msgs::UInt8::ConstPtr& msg);

    // 成员变量
    ros::NodeHandle& nh_;                           // ROS节点句柄
    
    // TF相关
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;    // TF缓冲区
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;  // TF监听器
    ros::Timer timer_;                              // TF查询定时器
    double update_rate_;                            // TF更新频率(Hz)
    std::string source_frame_;                      // 源坐标系
    std::string target_frame_;                      // 目标坐标系
    
    // 话题订阅器
    ros::Subscriber velocity_map_sub_;              // velocity_map话题订阅器
    ros::Subscriber target_velocity_sub_;           // target_velocity话题订阅器
    ros::Subscriber num_sq_sub_;
    ros::Subscriber qr_data_sub_;                   // QR数据订阅器
    // 串口通信
    std::unique_ptr<serial_comm::SerialComm> serial_comm_;  // 串口通信对象
    
    // 接收处理
    void protocolDataHandler(uint8_t id, const std::vector<uint8_t>& data);
    void sendPendingQrData();
    ros::Timer qr_send_timer_;               // 定时器对象
    std::vector<std::pair<int, int>> pending_qr_data_; 
    // 发布器
    ros::Publisher height_pub_;
    ros::Publisher is_st_ready_pub_;              // is_st_ready 话题发布器（std_msgs/UInt8）
    ros::Publisher mission_step_pub_;          // mission_step 话题发布器（std_msgs/UInt8）
    ros::Publisher battle_voltage_pub_;
    // 数据存储
    double current_yaw_;                              // 当前yaw角度
    bool yaw_valid_;                                  // yaw角度是否有效
    geometry_msgs::Twist current_velocity_;           // 当前速度数据
    bool velocity_valid_;                             // 速度数据是否有效
    bool has_st_ready_pub_;                           // 是否创建了 is_st_ready 发布器
    bool is_num_sq = false;                           // 是否有航点发布
    
    uint8_t sequence_number =1 ;

    // 协议配置
    static const uint8_t VELOCITY_FRAME_ID = 0x32;        // 速度数据的帧ID
    static const uint8_t TARGET_VELOCITY_FRAME_ID = 0x31; // 目标速度数据的帧ID
    static const uint8_t ST_READY_QUERY_ID = 0xF1;        // STM32 准备状态查询帧ID（接收）
    static const uint8_t A2_READY_RESP_ID = 0xA2;         // A2 应答帧ID（发送）
    static const uint8_t HEIGHT_ID = 0x05;                // 高度数据帧ID 
    static const uint8_t BATTEL_VOLTAGE_ID = 0x0d;        // 电池电压数据帧ID                 
};

} // namespace uart_to_stm32

#endif // UART_TO_STM32_H