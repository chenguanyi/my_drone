#ifndef BLUETOOTH_PUBLISHER_H
#define BLUETOOTH_PUBLISHER_H
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <serial_comm/serial_comm.h>
#include <Eigen/Dense>
#include <memory>
#include <std_msgs/UInt8.h>
#include <yolo/yolo.h>
#include <std_msgs/Float32.h>

class BlueTooth_Publisher{

        

public:
        
        struct cma_inparam{
            double fx;//焦距x
            double fy;//焦距y
            double cx;//光学中心x
            double cy;//光学中心y
        };
        
        struct RobotPosition {
            float x;  // x坐标（厘米）
            float y;  // y坐标（厘米）
            bool valid; // 位置是否有效
        };

        struct WorldPosition {
            double x;  // x坐标（厘米）
            double y;  // y坐标（厘米）
        };
        
        explicit BlueTooth_Publisher(ros::NodeHandle& nh,cma_inparam came);

        ~BlueTooth_Publisher();

        // 初始化
        bool BlueTooth_init(double update_rate,const std::string source_frame,const std::string target_frame);
        //运行主循环
        void run();
        //接受处理
        void protocol_handler(uint8_t id, const std::vector<uint8_t>& data);
        //回调函数
        void yolo_callback(const yolo::yolo::ConstPtr& msg);
        void battle_callback(const std_msgs::Float32::ConstPtr& msg);
        void protocol_handler(ros::Publisher& num_pub,uint8_t id, const std::vector<uint8_t>& data);
        //tf变换
        void look_for_transform(void);
        //像素转世界坐标
        void pixelToWorldWithDepth(int u, int v, double Z, cma_inparam params,RobotPosition* pos);

private:
        //成员变量
        ros::NodeHandle nh_;  
        //订阅者
        ros::Subscriber yolo_sub_;
        ros::Subscriber battle_sub_;
        ros::Subscriber height_sub_;
        //串口对象
        std::unique_ptr<serial_comm::SerialComm> serial_comm_;
        // 发布者
        ros::Publisher sq_num_pub_;
        ros::Publisher pojiang_pub_;
        //相机参数
        cma_inparam came1;
        //tf监听得到的坐标
        RobotPosition current_pos;
        //相机计算的偏差坐标
        RobotPosition current_camepos;
        //最终坐标
        WorldPosition final_pos;
        //标志地图
        uint8_t whole_map[9][7] = {{0}}; 
        //高度
        uint16_t height;
        //标志位
        bool battle_flag = true;
        //tf
        std::string source_frame_;
        std::string target_frame_;
        double update_rate_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
        ros::Timer timer_;


};

#endif