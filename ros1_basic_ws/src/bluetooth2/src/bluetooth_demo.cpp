#include <ros/ros.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <cstring>

class BluetoothNode {
private:
    ros::NodeHandle nh_;
    int bluetooth_socket_;
    struct sockaddr_rc remote_address_;
    bool connected_;
    std::string remote_mac_;

public:
    BluetoothNode() : connected_(false) {
        bluetooth_socket_ = -1;
    }

    ~BluetoothNode() {
        disconnect();
    }

    bool connect(const std::string& mac_address, uint8_t port = 1) {
        // 创建蓝牙RFCOMM套接字
        bluetooth_socket_ = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
        if (bluetooth_socket_ < 0) {
            ROS_ERROR("Failed to create bluetooth socket");
            return false;
        }

        // 设置远程地址
        remote_address_.rc_family = AF_BLUETOOTH;
        remote_address_.rc_channel = port;
        str2ba(mac_address.c_str(), &remote_address_.rc_bdaddr);

        ROS_INFO("Connecting to bluetooth device: %s", mac_address.c_str());

        // 连接蓝牙设备
        if (::connect(bluetooth_socket_, (struct sockaddr*)&remote_address_, sizeof(remote_address_)) < 0) {
            ROS_ERROR("Failed to connect to bluetooth device: %s", mac_address.c_str());
            close(bluetooth_socket_);
            bluetooth_socket_ = -1;
            return false;
        }

        connected_ = true;
        remote_mac_ = mac_address;
        ROS_INFO("Bluetooth connected to device: %s", mac_address.c_str());
        return true;
    }

    void disconnect() {
        if (connected_ && bluetooth_socket_ >= 0) {
            close(bluetooth_socket_);
            bluetooth_socket_ = -1;
            connected_ = false;
            ROS_INFO("Bluetooth disconnected from device: %s", remote_mac_.c_str());
        }
    }

    bool isConnected() const {
        return connected_;
    }

    std::string getRemoteMac() const {
        return remote_mac_;
    }

    bool receiveData(std::string& data) {
        if (!connected_ || bluetooth_socket_ < 0) {
            return false;
        }

        char buffer[1024];
        ssize_t bytes_received = recv(bluetooth_socket_, buffer, sizeof(buffer) - 1, 0);
        
        if (bytes_received > 0) {
            buffer[bytes_received] = '\0';
            data = std::string(buffer);
            return true;
        } else if (bytes_received == 0) {
            // 连接已关闭
            ROS_WARN("Bluetooth connection closed by remote device");
            disconnect();
            return false;
        } else {
            // 错误处理
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                ROS_ERROR("Error receiving data from bluetooth device");
                disconnect();
                return false;
            }
            return false;
        }
    }

    bool sendData(const std::string& data) {
        if (!connected_ || bluetooth_socket_ < 0) {
            return false;
        }

        ssize_t bytes_sent = send(bluetooth_socket_, data.c_str(), data.length(), 0);
        if (bytes_sent < 0) {
            ROS_ERROR("Error sending data to bluetooth device");
            disconnect();
            return false;
        }
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bluetooth_demo");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    BluetoothNode bt_node;

    // 从参数服务器获取蓝牙设备MAC地址
    std::string mac_address;
    private_nh.param<std::string>("mac_address", mac_address, "00:00:00:00:00:00");
    
    int port;
    private_nh.param<int>("port", port, 1);

    // 连接蓝牙设备
    if (!bt_node.connect(mac_address, port)) {
        ROS_ERROR("Failed to connect to bluetooth device");
        return -1;
    }

    ros::Rate loop_rate(10); // 10Hz

    while (ros::ok()) {
        // 检查连接状态
        if (!bt_node.isConnected()) {
            ROS_ERROR("Bluetooth connection lost");
            break;
        }

        // 接收数据
        std::string received_data;
        if (bt_node.receiveData(received_data)) {
            ROS_INFO("Received data: %s", received_data.c_str());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    // 断开蓝牙连接
    bt_node.disconnect();

    return 0;
}