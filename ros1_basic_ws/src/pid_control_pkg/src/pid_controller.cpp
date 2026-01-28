#include "pid_controller.h"
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <cmath>

// PID控制器类实现
PIDController::PIDController(double kp, double ki, double kd, double max_output, double min_output, 
                           double integral_limit, double deadzone)
    : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output), min_output_(min_output),
      integral_limit_(integral_limit), deadzone_(deadzone),
      prev_error_(0.0), current_error_(0.0), integral_(0.0), prev_derivative_(0.0),
      first_call_(true), derivative_filter_alpha_(0.8)
{
}

double PIDController::calculate(double setpoint, double measured_value, double dt)
{
    current_error_ = setpoint - measured_value;
    
    // 死区处理
    if (std::abs(current_error_) < deadzone_)
    {
        current_error_ = 0.0;
    }
    
    if (first_call_)
    {
        prev_error_ = current_error_;
        first_call_ = false;
    }
    
    // 比例项
    double proportional = kp_ * current_error_;
    
    // 积分项 - 添加积分限制
    integral_ += current_error_ * dt;
    // 积分限制
    if (integral_ > integral_limit_) integral_ = integral_limit_;
    else if (integral_ < -integral_limit_) integral_ = -integral_limit_;
    
    double integral_term = ki_ * integral_;
    
    // 微分项 - 添加低通滤波
    double derivative_raw = (dt > 0.0) ? (current_error_ - prev_error_) / dt : 0.0;
    double derivative_filtered = derivative_filter_alpha_ * prev_derivative_ + 
                                (1.0 - derivative_filter_alpha_) * derivative_raw;
    double derivative_term = kd_ * derivative_filtered;
    
    // PID输出
    double output = proportional + integral_term + derivative_term;
    
    // 输出限制
    if (output > max_output_)
        output = max_output_;
    else if (output < min_output_)
        output = min_output_;
    
    // 改进的防积分饱和 - 条件积分
    if ((output >= max_output_ && current_error_ > 0) || (output <= min_output_ && current_error_ < 0))
    {
        integral_ -= current_error_ * dt;  // 回退积分
    }
    
    prev_error_ = current_error_;
    prev_derivative_ = derivative_filtered;
    
    return output;
}

void PIDController::reset()
{
    prev_error_ = 0.0;
    current_error_ = 0.0;
    integral_ = 0.0;
    prev_derivative_ = 0.0;
    first_call_ = true;
}

void PIDController::setPID(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setOutputLimits(double max_output, double min_output)
{
    max_output_ = max_output;
    min_output_ = min_output;
}

void PIDController::setIntegralLimit(double integral_limit)
{
    integral_limit_ = integral_limit;
}

void PIDController::setDeadzone(double deadzone)
{
    deadzone_ = deadzone;
}

// 位置PID控制节点类实现
PositionPIDController::PositionPIDController()
    : private_nh_("~"),
      pid_x_(0.0, 0.0, 0.0),
      pid_y_(0.0, 0.0, 0.0),
      pid_z_(0.0, 0.0, 0.0),
      pid_yaw_(0.0, 0.0, 0.0),
      pid_xy_speed_(0.0, 0.0, 0.0),
      target_x_cm_(0.0), target_y_cm_(0.0), target_z_cm_(0.0), target_yaw_deg_(0.0),
      current_x_cm_(0.0), current_y_cm_(0.0), current_yaw_deg_(0.0), current_z_cm_(0.0),
      error_x_cm_(0.0), error_y_cm_(0.0), error_yaw_deg_(0.0), error_z_cm_(0.0),
      distance_xy_cm_(0.0),
      has_target_position_(false),
      has_target_height_(false),
      has_qr_offset_(false),
      has_current_pose_(false),
      pojiang_flag(false),
      control_frequency_(50.0),
      map_frame_("map"),
      laser_link_frame_("laser_link"),
      max_slow_vel_(20.0),
      control_mode_(NORMAL_MODE),
      current_pid_mode_(0),
      qr_target_x_(320),
      qr_target_y_(170)
{
    // 加载参数
    loadParameters();
    // 订阅目标位置
    target_position_sub_ = nh_.subscribe("/target_position", 1, 
        &PositionPIDController::targetPositionCallback, this);
    // 订阅高度
    height_sub_ = nh_.subscribe("/height", 1, 
        &PositionPIDController::heightCallback, this);
    // 订阅迫降信号
    pojiang_sub_ = nh_.subscribe<std_msgs::UInt8>("/pojiang",1,
    [&]( const std_msgs::UInt8::ConstPtr& msg){
        pojiang_flag = (msg->data ==1);
    }
    );
    // 订阅PID模式
    pid_mod_sub_ = nh_.subscribe("/pid_mode", 1, 
        &PositionPIDController::pidModeCallback, this);
    // 订阅二维码偏移
    qr_offset_sub_ = nh_.subscribe<geometry_msgs::Point>("/qr_code_position",1,
    [&](const geometry_msgs::Point::ConstPtr& msg){
        qr_offset_x_ = msg->x;
        qr_offset_y_ = msg->y;
        has_qr_offset_ = true;
    }    
    );
    // 初始化发布器 - 发布4个浮点数 [vx_cm/s, vy_cm/s, vz_cm/s, vyaw_deg/s]
    target_velocity_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/target_velocity", 1);
    
    ROS_INFO("Position PID Controller initialized");
    ROS_INFO("Control frequency: %.1f Hz", control_frequency_);
    ROS_INFO("Map frame: %s, Laser link frame: %s", map_frame_.c_str(), laser_link_frame_.c_str());
}

PositionPIDController::~PositionPIDController()
{
}

void PositionPIDController::loadParameters()
{
    // 控制频率参数
    private_nh_.param("control_frequency", control_frequency_, 50.0);
    // 坐标系参数
    private_nh_.param("map_frame", map_frame_, std::string("map"));
    private_nh_.param("laser_link_frame", laser_link_frame_, std::string("laser_link"));

    //导航模式参数设置
    private_nh_.param("navigate/kp_xy",navigate_params_.kp_xy,0.8);
    private_nh_.param("navigate/ki_xy",navigate_params_.ki_xy,0.0);
    private_nh_.param("navigate/kd_xy",navigate_params_.kd_xy,0.1);
    private_nh_.param("navigate/kp_yaw",navigate_params_.kp_yaw,1.0);
    private_nh_.param("navigate/ki_yaw",navigate_params_.ki_yaw,0.0);
    private_nh_.param("navigate/kd_yaw",navigate_params_.kd_yaw,0.2);
    private_nh_.param("navigate/kp_z",navigate_params_.kp_z,1.2);
    private_nh_.param("navigate/ki_z",navigate_params_.ki_z,0.0);
    private_nh_.param("navigate/kd_z",navigate_params_.kd_z,0.15);
    private_nh_.param("navigate/max_linear_velocity",navigate_params_.max_linear_vel,36.0);
    private_nh_.param("navigate/max_angular_velocity",navigate_params_.max_angular_vel,30.0);
    private_nh_.param("navigate/max_vertical_velocity",navigate_params_.max_vertical_vel,40.0);
    private_nh_.param("navigate/min_linear_velocity",navigate_params_.min_linear_vel,-36.0);
    private_nh_.param("navigate/min_angular_velocity",navigate_params_.min_angular_vel,-30.0);
    private_nh_.param("navigate/min_vertical_velocity",navigate_params_.min_vertical_vel,-40.0);
    private_nh_.param("navigate/Xpos_tolerance",navigate_params_.Xpos_tolerance,6.0);
    private_nh_.param("navigate/Ypos_tolerance",navigate_params_.Ypos_tolerance,6.0);
    private_nh_.param("navigate/yaw_tolerance",navigate_params_.yaw_tolerance,5.0);
    private_nh_.param("navigate/height_tolerance",navigate_params_.height_tolerance,6.0);
    ROS_INFO("Navigate PID Parameters loaded:");
    //精调模式参数集 
    private_nh_.param("fine_tune/kp_xy",fine_tune_params_.kp_xy,0.3);
    private_nh_.param("fine_tune/ki_xy",fine_tune_params_.ki_xy,0.0);
    private_nh_.param("fine_tune/kd_xy",fine_tune_params_.kd_xy,0.05);
    private_nh_.param("fine_tune/kp_yaw",fine_tune_params_.kp_yaw,1.0);
    private_nh_.param("fine_tune/ki_yaw",fine_tune_params_.ki_yaw,0.0);
    private_nh_.param("fine_tune/kd_yaw",fine_tune_params_.kd_yaw,0.2);
    private_nh_.param("fine_tune/kp_z",fine_tune_params_.kp_z,0.3);
    private_nh_.param("fine_tune/ki_z",fine_tune_params_.ki_z,0.0);
    private_nh_.param("fine_tune/kd_z",fine_tune_params_.kd_z,0.1);
    private_nh_.param("fine_tune/max_linear_velocity",fine_tune_params_.max_linear_vel,6.0);
    private_nh_.param("fine_tune/max_angular_velocity",fine_tune_params_.max_angular_vel,-6.0);
    private_nh_.param("fine_tune/max_vertical_velocity",fine_tune_params_.max_vertical_vel,6.0);
    private_nh_.param("fine_tune/min_linear_velocity",fine_tune_params_.min_linear_vel,-6.0);
    private_nh_.param("fine_tune/min_angular_velocity",fine_tune_params_.min_angular_vel,6.0);
    private_nh_.param("fine_tune/min_vertical_velocity",fine_tune_params_.min_vertical_vel,-6.0);
    private_nh_.param("fine_tune/Xpos_tolerance",fine_tune_params_.Xpos_tolerance,20.0);//单位像素
    private_nh_.param("fine_tune/Ypos_tolerance",fine_tune_params_.Ypos_tolerance,6.0);//单位cm
    private_nh_.param("fine_tune/yaw_tolerance",fine_tune_params_.yaw_tolerance,5.0);//单位度
    private_nh_.param("fine_tune/height_tolerance",fine_tune_params_.height_tolerance,20.0);//单位像素
    ROS_INFO("Fine tune PID Parameters loaded:");

    applyPIDParams(navigate_params_);
    ROS_INFO("kp_xy: %.1f,ki_xy: %.1f,kd_xy: %.1f,kp_yaw: %.1f,ki_yaw: %.1f,kd_yaw: %.1f,",navigate_params_.kp_xy,
    navigate_params_.ki_xy,navigate_params_.kd_xy,navigate_params_.kp_yaw,navigate_params_.ki_yaw,
    navigate_params_.kd_yaw);
}

void PositionPIDController::targetPositionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() >= 4)
    {
        target_x_cm_ = msg->data[0];   // x位置 (cm)
        target_y_cm_ = msg->data[1];   // y位置 (cm)  
        target_z_cm_ = msg->data[2];   // z位置 (cm)
        target_yaw_deg_ = msg->data[3]; // yaw角度 (度)
        
        has_target_position_ = true;
        
        // ROS_INFO("Received target position: x=%.1fcm, y=%.1fcm, z=%.1fcm, yaw=%.1f°", 
        //          target_x_cm_, target_y_cm_, target_z_cm_, target_yaw_deg_);
    }
    else
    {
        ROS_WARN("Target position message should contain 4 float values [x_cm, y_cm, z_cm, yaw_deg]");
    }
}

void PositionPIDController::heightCallback(const std_msgs::Int16::ConstPtr& msg)
{
    current_z_cm_ = msg->data;  // 直接使用cm为单位的高度
    has_target_height_ = true;
    
    // ROS_DEBUG("Received height: %dcm", msg->data);
}

bool PositionPIDController::getCurrentPose()
{
    tf::StampedTransform transform;
    try
    {
        tf_listener_.lookupTransform(map_frame_, laser_link_frame_, 
                                   ros::Time(0), transform);
        
        // 获取位置并转换为cm
        current_x_cm_ = meterToCm(transform.getOrigin().x());
        current_y_cm_ = meterToCm(transform.getOrigin().y());
        // 获取yaw角度并转换为度
        tf::Quaternion q = transform.getRotation();
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_yaw_deg_ = radToDeg(yaw);
        // ROS_INFO("Current pose: x=%.1fcm, y=%.1fcm, yaw=%.1f°", 
        //          current_x_cm_, current_y_cm_, current_yaw_deg_);
        has_current_pose_ = true;
        return true;
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("Could not get transform from %s to %s: %s", 
                 map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
        return false;
    }
}

// 角度归一化函数 (度)
double PositionPIDController::normalizeAngleDeg(double angle_deg)
{
    while (angle_deg > 180.0) angle_deg -= 360.0;
    while (angle_deg < -180.0) angle_deg += 360.0;
    return angle_deg;
}

// 误差计算函数 (单位: cm, 度)
void PositionPIDController::calculateErrors()
{
    if(current_pid_mode_ == 0)
    {
        error_x_cm_ = target_x_cm_ - current_x_cm_;
        error_y_cm_ = target_y_cm_ - current_y_cm_;

        distance_xy_cm_ = sqrt(error_x_cm_ * error_x_cm_ + error_y_cm_ * error_y_cm_);

        // 计算yaw误差（归一化到-180到180度范围）
        error_yaw_deg_ = normalizeAngleDeg(target_yaw_deg_ - current_yaw_deg_);

        // Z轴误差 (cm)
        if (has_target_height_)
        {
            error_z_cm_ = target_z_cm_ - current_z_cm_;
        }
        else
        {
            error_z_cm_ = 0.0;
        }
    }else if(current_pid_mode_ == 1)
    {
        error_x_cm_ = qr_target_x_ - qr_offset_x_;
        error_z_cm_ = -(qr_target_y_ - qr_offset_y_);

        error_y_cm_ = target_y_cm_ - current_y_cm_;
        error_yaw_deg_ = normalizeAngleDeg(target_yaw_deg_ - current_yaw_deg_);
    }
}

// 检查是否到达目标 (单位: cm, 度)
bool PositionPIDController::isTargetReached()
{
    return (std::abs(error_x_cm_) <= Xpos_tolerance_ && 
            std::abs(error_y_cm_) <= Ypos_tolerance_ && 
            std::abs(error_yaw_deg_) <= yaw_tolerance_ &&
            (std::abs(error_z_cm_) <= height_tolerance_ || !has_target_height_));
}

// PID处理函数 (输出单位: cm/s, 度/s)
std_msgs::Float32MultiArray PositionPIDController::processPID(double dt)
{
    std_msgs::Float32MultiArray cmd_vel;
    cmd_vel.data.resize(4);  // [vx_cm/s, vy_cm/s, vz_cm/s, vyaw_deg/s]
    
    calculateErrors();
    
    double vel_x_cm, vel_y_cm, vel_yaw_deg, vel_z_cm;
    
if (current_pid_mode_ == 0)//导航
{
    // 根据控制模式计算输出 (单位: cm/s)
    switch (control_mode_)
    {
        case NORMAL_MODE:
        case SLOW_MODE:
        {
            // 基于平面距离PID计算总线速度，再按角度分配到x/y
            if (distance_xy_cm_ > 0.1) // 避免除零
            {
                // setpoint=0, measured=distance，输出应为负，取反得到正速度
                double speed_cmd = -pid_xy_speed_.calculate(0.0, distance_xy_cm_, dt);
                if (speed_cmd < 0.0) speed_cmd = 0.0; // 保护：不允许负速度

                double cos_theta = error_x_cm_ / distance_xy_cm_;
                double sin_theta = error_y_cm_ / distance_xy_cm_;
                vel_x_cm = speed_cmd * cos_theta;
                vel_y_cm = speed_cmd * sin_theta;
            }
            else
            {
                vel_x_cm = 0.0;
                vel_y_cm = 0.0;
                // 可选：到点后复位距离PID，避免残余积分
                // pid_xy_speed_.reset();
            }
            break;
        }
        
        case LOCK_Y_MODE:
        {
            // 锁定Y轴模式：优先控制Y，保持X
            vel_y_cm = pid_y_.calculate(target_y_cm_, current_y_cm_, dt);
            vel_x_cm = pid_x_.calculate(target_x_cm_, current_x_cm_, dt) * 0.4; // 保持增益
            break;
        }
        
        case LOCK_X_MODE:
        {
            // 锁定X轴模式：优先控制X，保持Y
            vel_x_cm = pid_x_.calculate(target_x_cm_, current_x_cm_, dt);
            vel_y_cm = pid_y_.calculate(target_y_cm_, current_y_cm_, dt) * 0.4; // 保持增益
            break;
        }
        
        case HOVER_MODE:
        {
            // 悬停模式：微调位置
            vel_x_cm = pid_x_.calculate(target_x_cm_, current_x_cm_, dt);
            vel_y_cm = pid_y_.calculate(target_y_cm_, current_y_cm_, dt);
            break;
        }
        
        default:
            vel_x_cm = vel_y_cm = 0.0;
            break;
    }
    
    double vel_x_body,vel_y_body;//修改
    double yaw_rad = degToRad(current_yaw_deg_);
    vel_x_body = vel_x_cm * cos(yaw_rad) + vel_y_cm * sin(yaw_rad);
    vel_y_body = -vel_x_cm * sin(yaw_rad) + vel_y_cm * cos(yaw_rad);
    vel_x_cm = vel_x_body;
    vel_y_cm = vel_y_body;

    // Yaw控制 (度/s)
    vel_yaw_deg = pid_yaw_.calculate(0.0, -error_yaw_deg_, dt);
    

    // Z轴控制 (cm/s)
    if (has_target_height_)
    {
        vel_z_cm = pid_z_.calculate(target_z_cm_, current_z_cm_, dt);
    }
    else
    {
        vel_z_cm = 0.0;
    }
}else if (current_pid_mode_ == 1)//精调
{

    // X轴控制 (cm/s)
    vel_x_cm = pid_x_.calculate(0.0, -error_x_cm_, dt);

    // Y轴控制 (cm/s)
    vel_y_cm = pid_y_.calculate(0.0, -error_y_cm_, dt);

    // Yaw控制 (度/s)
    vel_yaw_deg = pid_yaw_.calculate(0.0, -error_yaw_deg_, dt);
    
    // Z轴控制 (cm/s)
    if (has_target_height_)
    {
        vel_z_cm = pid_z_.calculate(0.0, -error_z_cm_, dt);
    }
    else
    {
        vel_z_cm = 0.0;
    }
}


    // 填充消息 [vx_cm/s, vy_cm/s, vz_cm/s, vyaw_deg/s]
    cmd_vel.data[0] = vel_x_cm;
    cmd_vel.data[1] = vel_y_cm;
    cmd_vel.data[2] = vel_z_cm;
    cmd_vel.data[3] = vel_yaw_deg;
    ROS_INFO_THROTTLE(1, "PID output: [%.2f, %.2f, %.2f, %.2f]", vel_x_cm, vel_y_cm, vel_z_cm, vel_yaw_deg);
    return cmd_vel;
}

void PositionPIDController::controlLoop()
{
    ros::Rate rate(control_frequency_);
    
    while (ros::ok())
    {
        ros::spinOnce();
        
        // 检查是否有目标位置和当前位置
        if (!has_target_position_)
        {
            rate.sleep();
            continue;
        }
        
        if (!getCurrentPose())
        {
            rate.sleep();
            continue;
        }
        
        // 计算时间间隔
        static ros::Time last_time = ros::Time::now();
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        if (dt <= 0.0) dt = 1.0 / control_frequency_;
        last_time = current_time;
        
        // 处理PID控制
        std_msgs::Float32MultiArray cmd_vel = processPID(dt);
        if((target_x_cm_ == -1 && target_y_cm_ == -1 && target_z_cm_ == -1 && target_yaw_deg_ == -1) || pojiang_flag == true)
        {
            cmd_vel.data[0] = 0.0;
            cmd_vel.data[1] = 0.0;
            cmd_vel.data[2] = 100.0;
            cmd_vel.data[3] = 0.0;
        }
        target_velocity_pub_.publish(cmd_vel);
        
        // 打印调试信息
        if (isTargetReached())
        {
            ROS_INFO_THROTTLE(1.0, "Target reached! Distance, Yaw error: %.1f°", 
                              error_yaw_deg_);
        }
        
        // ROS_INFO("Current: [%.1f, %.1f, %.1f, %.1f°]",
        //          current_x_cm_, current_y_cm_, current_z_cm_, current_yaw_deg_);
        
        rate.sleep();
    }
}

void PositionPIDController::pidModeCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    uint8_t new_mode = msg->data;

    if(new_mode != current_pid_mode_)
    {
        current_pid_mode_ = new_mode;
        if(current_pid_mode_ == 0)
        {
            applyPIDParams(navigate_params_);
            ROS_INFO("Switch to Navigate Mode");
        }else if(current_pid_mode_ == 1)
        {
            applyPIDParams(fine_tune_params_);
            ROS_INFO("Switch to Fine Tune Mode");
        }
    }
}

void PositionPIDController::applyPIDParams(const PIDParams params)
{
    pid_x_.setPID(params.kp_xy, params.ki_xy, params.kd_xy);
    pid_y_.setPID(params.kp_xy, params.ki_xy, params.kd_xy);
    pid_z_.setPID(params.kp_z, params.ki_z, params.kd_z);
    pid_yaw_.setPID(params.kp_yaw, params.ki_yaw, params.kd_yaw);
    pid_xy_speed_.setPID(params.kp_xy, params.ki_xy, params.kd_xy);

    pid_x_.setOutputLimits(params.max_linear_vel, params.min_linear_vel);
    pid_y_.setOutputLimits(params.max_linear_vel, params.min_linear_vel);
    pid_z_.setOutputLimits(params.max_vertical_vel, params.min_vertical_vel);
    pid_yaw_.setOutputLimits(params.max_angular_vel, params.min_angular_vel);
    pid_xy_speed_.setOutputLimits(params.max_linear_vel, params.min_linear_vel);
}

// 主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "position_pid_controller");
    
    try
    {
        PositionPIDController controller;
        controller.controlLoop();
    }
    catch (std::exception& e)
    {
        ROS_ERROR("Exception in position PID controller: %s", e.what());
        return -1;
    }
    
    return 0;
}