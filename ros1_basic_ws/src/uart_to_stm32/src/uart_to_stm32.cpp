#include "uart_to_stm32.h"
#include <tf2/exceptions.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
namespace uart_to_stm32 {

// 定义QR数据映射表，按照指定规则
const std::map<uint8_t, uint8_t> qr_data_mapping = {
    {1,3}, {2, 2}, {3, 1}, {4, 4}, {5, 5}, {6, 6},
    {7, 18}, {8, 17}, {9, 16}, {10, 13}, {11, 14}, {12, 15},
    {13, 7}, {14, 8}, {15, 9}, {16, 12}, {17, 11}, {18, 10},
    {19, 22}, {20, 23}, {21, 24}, {22, 21}, {23, 20}, {24, 19}
};



UartToStm32::UartToStm32(ros::NodeHandle& nh) 
    : nh_(nh), current_yaw_(0.0), yaw_valid_(false), velocity_valid_(false) {
    
    ROS_INFO("UartToStm32 created");
}

UartToStm32::~UartToStm32() {
    if (timer_.isValid()) {
        timer_.stop();
    }
    
    // 停止协议接收并关闭串口
    if (serial_comm_) {
        serial_comm_->stop_protocol_receive();
        serial_comm_->close();
    }
}

bool UartToStm32::initialize(double update_rate, const std::string& source_frame, const std::string& target_frame) {
    try {
        // 保存参数
        update_rate_ = update_rate;
        source_frame_ = source_frame;
        target_frame_ = target_frame;
        

        ROS_INFO("UartToStm32 initialized with update rate: %.1f Hz", update_rate_);
        ROS_INFO("Looking for transform from '%s' to '%s'", source_frame_.c_str(), target_frame_.c_str());
        
        // 初始化串口通信
        serial_comm_ = std::make_unique<serial_comm::SerialComm>();
        if (!serial_comm_->initialize("/dev/ttyS6", 921600)) {
            ROS_ERROR("Failed to initialize serial port /dev/ttyS6 at 921600 baudrate");
            ROS_ERROR("Serial error: %s", serial_comm_->get_last_error().c_str());
            return false;
        }
        ROS_INFO("Serial port /dev/ttyS6 initialized at 921600 baudrate");
        
        // 创建TF缓冲区和监听器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 创建定时器，定期查找tf变换
        timer_ = nh_.createTimer(
            ros::Duration(1.0 / update_rate_),
            [this](const ros::TimerEvent&) { this->lookupTransform(); }
        );
        
        // 创建velocity_map话题订阅器
        velocity_map_sub_ = nh_.subscribe("/velocity_map", 10, 
            &UartToStm32::velocityMapCallback, this);
        
        // 创建target_velocity话题订阅器
        target_velocity_sub_ = nh_.subscribe("/target_velocity", 10, 
            &UartToStm32::targetVelocityCallback, this);

        //判断是否发送航点数据
        num_sq_sub_ =  nh_.subscribe<std_msgs::UInt8>("/square_num", 10,
        [this](const std_msgs::UInt8::ConstPtr& msg) {
            if(msg->data > 0)
            {
                is_num_sq = true;
            }
        });
        
        // 创建QR数据订阅器
        qr_data_sub_ = nh_.subscribe("/processed_qr_data", 10,
            &UartToStm32::qrDataCallback, this);
        
        // 创建高度发布器
        height_pub_ = nh_.advertise<std_msgs::Int16>("/height", 10);
        // 创建 is_st_ready 发布器（std_msgs/UInt8）
        is_st_ready_pub_ = nh_.advertise<std_msgs::UInt8>("/is_st_ready", 10, /*latch=*/true);
        mission_step_pub_ = nh_.advertise<std_msgs::UInt8>("/mission_step", 10);
        battle_voltage_pub_ = nh_.advertise<std_msgs::Float32>("/battle_voltage", 10);
        // 启动协议接收，回调到本类的处理函数
        serial_comm_->start_protocol_receive(
            [this](uint8_t id, const std::vector<uint8_t>& data) { this->protocolDataHandler(id, data); },
            [this](const std::string& err) { ROS_WARN("Serial protocol error: %s", err.c_str()); }
        );

        // 创建QR数据发送定时器，每秒发送一次
        // qr_send_timer_ = nh_.createTimer(
        //     ros::Duration(0.1),
        //     [this](const ros::TimerEvent&) { this->sendPendingQrData(); }
        // );

        ROS_INFO("UartToStm32 initialized successfully");
        ROS_INFO("Subscribed to /velocity_map and /target_velocity topics");
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to initialize topic subscriber: %s", e.what());
        return false;
    }
}

void UartToStm32::run() {
    ROS_INFO("UartToStm32 is running...");
    ros::spin();
}

void UartToStm32::lookupTransform() {
    try {
        // 查找从source_frame到target_frame的变换
        geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
            source_frame_, target_frame_, ros::Time(0), ros::Duration(0.1)
        );
        
        // 处理获取到的变换
        processTfTransform(transform);
        
    } catch (tf2::TransformException& ex) {
        // 静默处理TF异常，避免日志刷屏
        // 只在调试模式下输出
        ROS_DEBUG("Transform lookup failed: %s", ex.what());
    }
}

void UartToStm32::processTfTransform(const geometry_msgs::TransformStamped& transform) {
    // 提取位置信息
    double x = transform.transform.translation.x;
    double y = transform.transform.translation.y;
    double z = transform.transform.translation.z;
    
    // 提取旋转信息
    double qx = transform.transform.rotation.x;
    double qy = transform.transform.rotation.y;
    double qz = transform.transform.rotation.z;
    double qw = transform.transform.rotation.w;
    
    // 计算欧拉角
    tf2::Quaternion q(qx, qy, qz, qw);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // 更新当前yaw角度
    current_yaw_ = yaw;
    yaw_valid_ = true;
    
    // 输出变换信息（调试用）
    // ROS_INFO_THROTTLE(2.0, "Transform %s -> %s: pos(%.3f, %.3f, %.3f) rot(%.3f, %.3f, %.3f)", 
    //                   source_frame_.c_str(), target_frame_.c_str(),
    //                   x, y, z, roll, pitch, yaw);
    
    // 如果有有效的速度数据，立即进行坐标转换并发送
    if (velocity_valid_ && yaw_valid_) {
        Eigen::Vector3d linear_vel(current_velocity_.linear.x, 
                                   current_velocity_.linear.y, 
                                   current_velocity_.linear.z);
        
        Eigen::Vector3d transformed_vel = transformVelocity(linear_vel, current_yaw_);
        sendVelocityToSerial(transformed_vel);
    }
}

void UartToStm32::velocityMapCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // 保存当前速度数据
    current_velocity_ = *msg;
    velocity_valid_ = true;
    
    // 提取线速度
    double linear_x = msg->linear.x;
    double linear_y = msg->linear.y;
    double linear_z = msg->linear.z;
    
    // 提取角速度
    double angular_x = msg->angular.x;
    double angular_y = msg->angular.y;
    double angular_z = msg->angular.z;
    
    // 输出速度信息（调试用）原始速度
    // ROS_INFO("Velocity: linear(%.3f, %.3f, %.3f) angular(%.3f, %.3f, %.3f)", 
    //                   linear_x, linear_y, linear_z, angular_x, angular_y, angular_z);
    
    // 如果有有效的yaw角度，进行坐标转换并发送
    if (yaw_valid_ && velocity_valid_) {
        Eigen::Vector3d linear_vel(linear_x, linear_y, linear_z);
        Eigen::Vector3d transformed_vel = transformVelocity(linear_vel, current_yaw_);
        sendVelocityToSerial(transformed_vel);
    }
}

Eigen::Vector3d UartToStm32::transformVelocity(const Eigen::Vector3d& linear, double yaw) {
    // 参考odom_tran.cpp中的旋转矩阵实现
    // 创建2D旋转矩阵 (3x3矩阵，z轴旋转)
    Eigen::Matrix3d Rz;
    Rz << cos(yaw),  sin(yaw), 0,
          -sin(yaw), cos(yaw), 0,
          0,         0,        1;
    
    // 应用旋转变换
    Eigen::Vector3d transformed = Rz * linear;
    //转换后的速度
    // ROS_INFO("Velocity transform: yaw=%.3f, original(%.3f,%.3f,%.3f) -> transformed(%.3f,%.3f,%.3f)",
    //                   yaw * 180.0 / M_PI, linear.x(), linear.y(), linear.z(),
    //                   transformed.x(), transformed.y(), transformed.z());
    
    return transformed;
}

void UartToStm32::sendVelocityToSerial(const Eigen::Vector3d& transformed_velocity) {
    if (!serial_comm_ || !serial_comm_->is_open()) {
        ROS_WARN_THROTTLE(5.0, "Serial port is not open, cannot send velocity data");
        return;
    }
    
    try {
        // 将三个float值转换为int16_t (可以根据实际需求调整缩放因子)
        const double scale_factor = 100.0;  // 将m/s转换为cm/s
        
        int16_t vel_x = static_cast<int16_t>(transformed_velocity.x() * scale_factor);
        int16_t vel_y = static_cast<int16_t>(transformed_velocity.y() * scale_factor);
        int16_t vel_z = static_cast<int16_t>(transformed_velocity.z() * scale_factor);
        
        // 构建数据包：3个int16_t = 6字节
        std::vector<uint8_t> data(6);
        
        // 小端序写入int16_t数据
        data[0] = static_cast<uint8_t>(vel_x & 0xFF);         // vel_x 低字节
        data[1] = static_cast<uint8_t>((vel_x >> 8) & 0xFF);  // vel_x 高字节
        data[2] = static_cast<uint8_t>(vel_y & 0xFF);         // vel_y 低字节
        data[3] = static_cast<uint8_t>((vel_y >> 8) & 0xFF);  // vel_y 高字节
        data[4] = static_cast<uint8_t>(vel_z & 0xFF);         // vel_z 低字节
        data[5] = static_cast<uint8_t>((vel_z >> 8) & 0xFF);  // vel_z 高字节
        
        // 发送协议数据
        if (serial_comm_->send_protocol_data(VELOCITY_FRAME_ID, 6, data)) {
            ROS_INFO_THROTTLE(1.0,"Sent velocity data: x=%d, y=%d, z=%d (cm/s)", vel_x, vel_y, vel_z);
        } else {
            ROS_WARN_THROTTLE(5.0, "Failed to send velocity data: %s", serial_comm_->get_last_error().c_str());
        }
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in sendVelocityToSerial: %s", e.what());
    }
}

void UartToStm32::targetVelocityCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 4) {
        ROS_WARN("Target velocity message should contain 4 float values [vx_cm/s, vy_cm/s, vz_cm/s, vyaw_deg/s]");
        return;
    }
    
    // 提取四个速度分量
    float vx_cm_per_s = msg->data[0];   // X轴线速度 (cm/s)
    float vy_cm_per_s = msg->data[1];   // Y轴线速度 (cm/s)
    float vz_cm_per_s = msg->data[2];   // Z轴线速度 (cm/s)
    float vyaw_deg_per_s = msg->data[3]; // Yaw角速度 (deg/s)
    
    // 输出速度信息（调试用）
    ROS_INFO_THROTTLE(2.0, "Target Velocity: linear(%.1f, %.1f, %.1f)cm/s angular(%.1f)deg/s", 
                      vx_cm_per_s, vy_cm_per_s, vz_cm_per_s, vyaw_deg_per_s);
    
    // 发送到串口
    sendTargetVelocityToSerial(vx_cm_per_s, vy_cm_per_s, vz_cm_per_s, vyaw_deg_per_s);
}

void UartToStm32::sendTargetVelocityToSerial(float vx_cm_per_s, float vy_cm_per_s, float vz_cm_per_s, float vyaw_deg_per_s) {
    if (!serial_comm_ || !serial_comm_->is_open()) {
        ROS_WARN_THROTTLE(5.0, "Serial port is not open, cannot send target velocity data");
        return;
    }
    
    try {
        // 将四个float值转换为int16_t
        // 线速度已经是cm/s，角速度已经是deg/s，直接转换
        int16_t vel_x = static_cast<int16_t>(std::round(vx_cm_per_s));
        int16_t vel_y = static_cast<int16_t>(std::round(vy_cm_per_s));
        int16_t vel_z = static_cast<int16_t>(std::round(vz_cm_per_s));
        int16_t vel_yaw = static_cast<int16_t>(std::round(vyaw_deg_per_s));
        
        // 构建数据包：4个int16_t = 8字节
        std::vector<uint8_t> data(8);
        
        // 小端序写入int16_t数据
        data[0] = static_cast<uint8_t>(vel_x & 0xFF);         // vel_x 低字节
        data[1] = static_cast<uint8_t>((vel_x >> 8) & 0xFF);  // vel_x 高字节
        data[2] = static_cast<uint8_t>(vel_y & 0xFF);         // vel_y 低字节
        data[3] = static_cast<uint8_t>((vel_y >> 8) & 0xFF);  // vel_y 高字节
        data[4] = static_cast<uint8_t>(vel_z & 0xFF);         // vel_z 低字节
        data[5] = static_cast<uint8_t>((vel_z >> 8) & 0xFF);  // vel_z 高字节
        data[6] = static_cast<uint8_t>(vel_yaw & 0xFF);       // vel_yaw 低字节
        data[7] = static_cast<uint8_t>((vel_yaw >> 8) & 0xFF); // vel_yaw 高字节
        
        // 发送协议数据，使用ID 0x31
        if (serial_comm_->send_protocol_data(TARGET_VELOCITY_FRAME_ID, 8, data)) {
            ROS_INFO_THROTTLE(1.0, "Sent target velocity data: x=%d, y=%d, z=%d, yaw=%d", vel_x, vel_y, vel_z, vel_yaw);
        } else {
            ROS_WARN_THROTTLE(5.0, "Failed to send target velocity data: %s", serial_comm_->get_last_error().c_str());
        }
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in sendTargetVelocityToSerial: %s", e.what());
    }
}

void UartToStm32::protocolDataHandler(uint8_t id, const std::vector<uint8_t>& data) {
    switch (id) {
        case ST_READY_QUERY_ID: { // 0xF1 帧：长度9，第二字节==1 时发布一次 1，并回发 0xA2 帧
            if (data.size() < 9) {
                ROS_WARN("protocolDataHandler: ID 0xF1 data too short, len=%zu", data.size());
                break;
            }
            uint8_t first = data[0];
            if (mission_step_pub_)
            {
                std_msgs::UInt8 msg;
                msg.data = first;
                mission_step_pub_.publish(msg);
                ROS_INFO_THROTTLE(2.0, "Published /mission_step: %u (from 0xF1 frame)", static_cast<unsigned>(first));
            }
            uint8_t second = data[1];
            if (second == 1) {
                if (is_st_ready_pub_ ) {
                    std_msgs::UInt8 msg;
                    msg.data = 1;
                    is_st_ready_pub_.publish(msg);
                    // ROS_INFO("Published /is_st_ready: 1 (from 0xF1 frame)");
                }
                if(!has_st_ready_pub_){
                // if(is_num_sq)
                // {
                    // 发送 A2 应答帧
                    sendA2ReadyResponse();
                    has_st_ready_pub_ = true; // 只发布一次
                // }
                }
            } else {
                ROS_DEBUG("0xF1 frame second byte != 1 (%u), ignoring", static_cast<unsigned>(second));
            }
            break;
        }
        case HEIGHT_ID: {
            // Height: int16_t
            if (data.size() < 2) {
                ROS_WARN("protocolDataHandler: ID 0x05 data too short");
                break;
            }
            int16_t value = static_cast<int16_t>(static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8));
            std_msgs::Int16 msg;
            msg.data = value;
            if (height_pub_) {
                height_pub_.publish(msg);
                ROS_INFO_THROTTLE(2.0, "Published /height: %d", value);
            } else {
                ROS_WARN("Height publisher not initialized");
            }
            break;
        }
        case BATTEL_VOLTAGE_ID: {
            if(data.size() < 2) {
                ROS_WARN("protocolDataHandler: ID 0x0d data too short");
                break;
            }
            int16_t value = static_cast<int16_t>(static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8));
            float value_m = static_cast<float>(value) / 100.0f;
            std_msgs::Float32 msg;
            msg.data = value_m;
            if(battle_voltage_pub_)
            {
                battle_voltage_pub_.publish(msg);
                ROS_INFO_THROTTLE(2.0, "Published /battle_voltage: %.2f", value_m);
            }
        }

        default: {
            ROS_DEBUG_THROTTLE(10.0, "Unhandled protocol ID: 0x%02X, len=%zu", id, data.size());
            break;
        }
    }
}

void UartToStm32::sendA2ReadyResponse() {
    if (!serial_comm_ || !serial_comm_->is_open()) {
        ROS_WARN_THROTTLE(5.0, "Serial port is not open, cannot send A2 ready response");
        return;
    }
    // 构建长度9的数据，首字节0x01，其余填0
    std::vector<uint8_t> data(9, 0x00);
    data[0] = 0x01;
    if (serial_comm_->send_protocol_data(A2_READY_RESP_ID, 9, data)) {
        ROS_INFO("Sent A2 ready response (len=9, first=0x01)");
    } else {
        ROS_WARN("Failed to send A2 ready response: %s", serial_comm_->get_last_error().c_str());
    }
}

void UartToStm32::qrDataCallback(const std_msgs::UInt8::ConstPtr& msg) {
    uint8_t received_data = msg->data;
    
    // 查找映射表中是否有对应的重映射编号
    auto it = qr_data_mapping.find(sequence_number);
    if (it != qr_data_mapping.end()) {
        
        uint8_t mapped_id = it->second;
        sequence_number++;
        
        // 存储数据用于定时发送
        pending_qr_data_.push_back(std::make_pair(mapped_id, received_data));
        
        ROS_INFO("Queued QR data: id=%u, data=%u, sequence=%u", mapped_id, received_data, sequence_number);
    } else {
        ROS_WARN("Received QR data with no mapping: sequence=%u, data=%u", sequence_number, received_data);
    }
}

void UartToStm32::sendPendingQrData() {
    // 检查是否有待发送的数据
    if (pending_qr_data_.empty()) {
        return;
    }
    
    // 发送队列中的所有数据
    for (const auto& qr_item : pending_qr_data_) {
        uint8_t mapped_id = qr_item.first;
        uint8_t received_data = qr_item.second;
        
        // 构建数据包：编号 + 数据
        std::vector<uint8_t> data(2);
        data[0] = mapped_id;           // 重映射后的编号
        data[1] = received_data;       // 原始数据
        
        // 发送协议数据，使用ID 0xF4
        if (serial_comm_ && serial_comm_->is_open()) {
            if (serial_comm_->send_protocol_data(0xF4, 2, data)) {
                ROS_INFO("Sent QR data: id=%u, data=%u", mapped_id, received_data);
            } else {
                ROS_WARN("Failed to send QR data: %s", serial_comm_->get_last_error().c_str());
            }
        } else {
            ROS_WARN("Serial port is not open, cannot send QR data");
        }
    }
    
}

} // namespace uart_to_stm32