#include "bluetooth/bluetooth_publisher.h"
#include "tf2/exceptions.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"

BlueTooth_Publisher::BlueTooth_Publisher(ros::NodeHandle& nh,cma_inparam came)
    :nh_(nh),came1(came){
        ROS_INFO("BlueTooth_Publisher created"); 
}

BlueTooth_Publisher::~BlueTooth_Publisher(){
    // 停止协议接收并关闭串口
    if (serial_comm_) {
        serial_comm_->stop_protocol_receive();
        serial_comm_->close();
    }
}

//串口初始化
bool BlueTooth_Publisher::BlueTooth_init(double update_rate,const std::string source_frame,const std::string target_frame)
{
    update_rate_ = update_rate;
    source_frame_ = source_frame;
    target_frame_ = target_frame;
    current_pos.valid = false;
    current_camepos.valid = false;
    serial_comm_ = std::make_unique<serial_comm::SerialComm>();
    if (!serial_comm_->initialize("/dev/ttyS2", 115200)) {
        ROS_ERROR("Failed to initialize serial port /dev/ttyS2 at 115200 baudrate");
        ROS_ERROR("Serial error: %s", serial_comm_->get_last_error().c_str());
        return false;
    } 
    ROS_INFO("Serial port /dev/ttyS2 initialized at 115200 baudrate");
    
    //订阅者
    // yolo_sub_ = nh_.subscribe<yolo::yolo>("/yolo",1,&BlueTooth_Publisher::yolo_callback,this);
    battle_sub_ = nh_.subscribe<std_msgs::Float32>("/battle_voltage",1,&BlueTooth_Publisher::battle_callback,this);
    height_sub_ = nh_.subscribe<std_msgs::Int16>("/height",1,
    [&](const std_msgs::Int16::ConstPtr& msg){

        height = msg->data;
    }
    );
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = nh_.createTimer(ros::Duration(1.0/update_rate),
                [this](const ros::TimerEvent& event) {this->look_for_transform();});
    //创建话题发布者
    sq_num_pub_ = nh_.advertise<std_msgs::UInt8>("/square_num", 10);
    pojiang_pub_ = nh_.advertise<std_msgs::UInt8>("/pojiang",10);
    //启动协议接收
    serial_comm_->start_protocol_receive(
        [this](uint8_t id, const std::vector<uint8_t>& data){
            protocol_handler(id, data);}
    );
    return true;
}

void BlueTooth_Publisher::run()
{
    ROS_INFO("BlueTooth_Publisher running");
    ros::spin();
}


void BlueTooth_Publisher::protocol_handler(uint8_t id, const std::vector<uint8_t>& data)
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
            sq_num_pub_.publish(msg);
            break;
        }
        case 0xF2:
        {
            std_msgs::UInt8 msg;
            msg.data = data[0];
            ROS_INFO("pojiang_sign Recevied");
            pojiang_pub_.publish(msg);
            break;
        }
    }
}

void BlueTooth_Publisher::yolo_callback(const yolo::yolo::ConstPtr& msg)
{
    uint16_t cen_x = msg->x;
    uint16_t cen_y = msg->y;
    // 修改: 添加对高度值的检查
    if (height <= 0) {
        ROS_WARN_THROTTLE(1.0, "Invalid height value: %d", height);
        return;
    }
    
    pixelToWorldWithDepth(cen_x, cen_y, height, came1, &current_camepos);
    
    // 修改: 确保所有位置数据都有效才进行计算
    if(current_pos.valid && current_camepos.valid && height > 0)
    {
        final_pos.x = current_pos.x + current_camepos.x;
        final_pos.y = current_pos.y + current_camepos.y;

        uint8_t index_x = final_pos.x / 50;
        uint8_t index_y = final_pos.y / 50;

        if(whole_map[index_y][index_x] == 0)
        {
            whole_map[index_y][index_x] = 1;//这个一定要放在前面
            serial_comm_->send_protocol_data(0xF2, 2, {index_x,index_y});//串口发给地面站目标位置
            ROS_INFO("send_zuobiao_x:%.f,y:%.f",final_pos.x,final_pos.y);
        }
    } else {
        // ROS_WARN_THROTTLE(1.0, "Position data not valid. current_pos.valid: %d, current_pos2.valid: %d, high1: %d", 
        //                     current_pos.valid, current_pos2.valid, high1);
    }
    
    // ROS_INFO("lidar_x:%.f,lidar_y:%.f,camera_x:%.f,camera_y:%.f",
    //                   current_pos.x, current_pos.y, 
    //                   current_pos2.valid ? current_pos2.x : std::numeric_limits<double>::quiet_NaN(),
    //                   current_pos2.valid ? current_pos2.y : std::numeric_limits<double>::quiet_NaN());
}
void BlueTooth_Publisher::battle_callback(const std_msgs::Float32::ConstPtr& msg)
{
    if (msg->data < 15.5 || msg->data > 17.5) { // 假设合理电压范围是15.5-17.5V
        ROS_WARN("Voltage out of expected range: %.2fV", msg->data);
        return;
    }
    // if(battle_flag == true)
    // {
        uint8_t num = static_cast<uint8_t>((msg->data - 15.5) / 0.02);
            serial_comm_->send_protocol_data(0xF1, 1, {num});//串口发给地面站可巡航的格数
        // battle_flag = false;
    // }
}

void BlueTooth_Publisher::look_for_transform()
{
    try{
        geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
            source_frame_,target_frame_, ros::Time(0),ros::Duration(0.1)
        );
        current_pos.x = transform.transform.translation.x;
        current_pos.y = transform.transform.translation.y;
        current_pos.valid = true;
        // ROS_INFO_THROTTLE(1.0, "Current position: x=%.2f, y=%.2f", current_pos.x, current_pos.y);
    } catch(tf2::TransformException &ex) {
        ROS_WARN("Transform failed to get: %s",ex.what());
    }
}

void BlueTooth_Publisher::pixelToWorldWithDepth(int u, int v, double Z, cma_inparam params,RobotPosition* pos) {
    // 计算归一化坐标（相对于主点的偏移）
    double dx = (u - params.cx) / params.fx;  // (u - cx) / fx
    double dy = (v - params.cy) / params.fy;  // (v - cy) / fy
    
    // 通过深度Z计算真实坐标并进行简单的坐标轴转化
    pos->x = dx * Z;
    pos->y = -dy * Z;
    pos->valid = true;
}