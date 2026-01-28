#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

/**
 * @brief 增强型PID控制器类
 */
class PIDController
{
public:
    /**
     * @brief PID控制器构造函数
     * @param kp 比例增益
     * @param ki 积分增益  
     * @param kd 微分增益
     * @param max_output 最大输出限制
     * @param min_output 最小输出限制
     * @param integral_limit 积分限制值
     * @param deadzone 死区大小
     */
    PIDController(double kp, double ki, double kd, double max_output = 1.0, double min_output = -1.0, 
                 double integral_limit = 10.0, double deadzone = 0.0);
    
    /**
     * @brief 计算PID输出
     * @param setpoint 目标值
     * @param measured_value 当前测量值
     * @param dt 时间间隔
     * @return PID控制输出
     */
    double calculate(double setpoint, double measured_value, double dt);
    
    /**
     * @brief 重置PID控制器
     */
    void reset();
    
    /**
     * @brief 设置PID参数
     */
    void setPID(double kp, double ki, double kd);
    
    /**
     * @brief 设置输出限制
     */
    void setOutputLimits(double max_output, double min_output);
    
    /**
     * @brief 设置积分限制
     */
    void setIntegralLimit(double integral_limit);
    
    /**
     * @brief 设置死区
     */
    void setDeadzone(double deadzone);
    
    /**
     * @brief 获取当前误差
     */
    double getError() const { return current_error_; }
    
    /**
     * @brief 获取积分项
     */
    double getIntegral() const { return integral_; }

private:
    double kp_, ki_, kd_;           // PID参数
    double max_output_, min_output_; // 输出限制
    double integral_limit_;        // 积分限制
    double deadzone_;              // 死区
    double prev_error_;            // 上一次误差
    double current_error_;         // 当前误差
    double integral_;              // 积分项
    double prev_derivative_;       // 上一次微分项(用于滤波)
    bool first_call_;              // 是否首次调用标志
    
    // 滤波系数
    double derivative_filter_alpha_; // 微分项低通滤波系数
};

/**
 * @brief 控制模式枚举
 */
enum ControlMode {
    NORMAL_MODE = 0,     // 普通走点模式
    SLOW_MODE = 1,       // 慢速走点模式
    LOCK_Y_MODE = 2,     // 锁定Y轴模式
    LOCK_X_MODE = 3,     // 锁定X轴模式
    HOVER_MODE = 4       // 悬停模式
};

/**
 * @brief 位置PID控制节点类
 */
class PositionPIDController
{
public:
    /**
     * @brief 构造函数
     */
    PositionPIDController();
    
    /**
     * @brief 析构函数
     */
    ~PositionPIDController();
    
    /**
     * @brief 主控制循环
     */
    void controlLoop();

private:
    // ROS节点句柄
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅器
    ros::Subscriber target_position_sub_;
    ros::Subscriber height_sub_;
    ros::Subscriber pojiang_sub_;
    ros::Subscriber pid_mod_sub_;
    ros::Subscriber qr_offset_sub_;
    // 发布器
    ros::Publisher target_velocity_pub_;
    // TF监听器
    tf::TransformListener tf_listener_;

    // PID控制器
    PIDController pid_x_;
    PIDController pid_y_;
    PIDController pid_yaw_;
    PIDController pid_z_;
    PIDController pid_xy_speed_;

    // 控制参数
    double Xpos_tolerance_;         // x轴位置容差
    double Ypos_tolerance_;         // y轴位置容差
    double yaw_tolerance_;          // 角度容差
    double height_tolerance_;       // 高度容差
    
    // 速度限制
    double max_linear_vel_;         // 最大线速度
    double max_angular_vel_;        // 最大角速度
    double max_vertical_vel_;       // 最大垂直速度
    
    // 目标位置 (单位: cm, 度)
    double target_x_cm_, target_y_cm_, target_z_cm_, target_yaw_deg_;
    // 当前位置 (内部计算使用cm和度)
    double current_x_cm_, current_y_cm_, current_yaw_deg_, current_z_cm_;
    // 误差计算相关 (单位: cm, 度)
    double error_x_cm_, error_y_cm_, error_yaw_deg_, error_z_cm_;
    double distance_xy_cm_;

    // 数据可靠标志
    bool has_target_position_;
    bool has_target_height_;
    bool has_qr_offset_;
    // 迫降标志位
    bool pojiang_flag;    
    // 当前位置获取是否
    bool has_current_pose_;
    
    // 其他参数
    double control_frequency_; // 控制频率
    std::string map_frame_;    // 地图坐标系
    std::string laser_link_frame_; //激光雷达坐标系

    //当前pid模式(0导航，1精调)
    uint8_t current_pid_mode_;

    struct PIDParams {
        double kp_xy,ki_xy,kd_xy;
        double kp_yaw,ki_yaw,kd_yaw;
        double kp_z,ki_z,kd_z;
        double max_linear_vel,max_angular_vel,max_vertical_vel;
        double min_linear_vel,min_angular_vel,min_vertical_vel;
        double Xpos_tolerance,Ypos_tolerance, yaw_tolerance, height_tolerance;
    };

    ControlMode control_mode_;
    double max_slow_vel_;      // 慢速模式最大速度

    PIDParams navigate_params_;//导航参数集
    PIDParams fine_tune_params_;//精调参数集
    
    //应用PID参数
    void applyPIDParams(const PIDParams params);

    double qr_offset_x_;
    double qr_offset_y_;
    double qr_target_x_;
    double qr_target_y_;

    // 回调函数
    void targetPositionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void heightCallback(const std_msgs::Int16::ConstPtr& msg);
    void pidModeCallback(const std_msgs::UInt8::ConstPtr& msg);

    // 获取当前位置
    bool getCurrentPose();
    
    // 加载参数
    void loadParameters();
    
    // 误差计算
    void calculateErrors();
    
    // PID处理函数
    std_msgs::Float32MultiArray processPID(double dt);
    
    // 角度归一化 (度)
    double normalizeAngleDeg(double angle_deg);
    
    // 检查是否到达目标
    bool isTargetReached();
    
    // 单位转换函数
    double meterToCm(double meter) { return meter * 100.0; }
    double cmToMeter(double cm) { return cm / 100.0; }
    double radToDeg(double rad) { return rad * 180.0 / M_PI; }
    double degToRad(double deg) { return deg * M_PI / 180.0; }

};

#endif // PID_CONTROLLER_H