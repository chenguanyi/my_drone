#include "serial_comm/serial_comm.h"
#include <iostream>
#include <vector>
#include <string>

void data_handler(uint8_t id, const std::vector<uint8_t>& data) {
    std::cout << "收到协议数据 - ID: 0x" << std::hex << (int)id << std::dec 
              << ", 数据长度: " << data.size() << ", 数据内容: ";
    
    for (const auto& byte : data) {
        std::cout << std::hex << (int)byte << " ";
    }
    std::cout << std::dec << std::endl;
    
    // 根据不同的ID处理不同类型的数据
    switch (id) {
        case 0x01:
            std::cout << "处理传感器数据..." << std::endl;
            break;
        case 0x02:
            std::cout << "处理控制指令..." << std::endl;
            break;
        case 0x03:
            std::cout << "处理状态信息..." << std::endl;
            break;
        default:
            std::cout << "未知数据类型ID: 0x" << std::hex << (int)id << std::dec << std::endl;
            break;
    }
}

void error_handler(const std::string& error_msg) {
    std::cout << "串口错误: " << error_msg << std::endl;
}

int main() {
    std::cout << "串口协议通信示例程序" << std::endl;
    
    // 创建串口通信对象
    serial_comm::SerialComm serial;
    
    // 初始化串口（请根据实际情况修改串口名）
    std::string port_name = "/dev/ttyUSB0";  // 可能需要根据实际情况修改
    unsigned int baud_rate = 115200;
    
    std::cout << "尝试打开串口: " << port_name << " @ " << baud_rate << " bps" << std::endl;
    
    if (!serial.initialize(port_name, baud_rate)) {
        std::cout << "串口初始化失败: " << serial.get_last_error() << std::endl;
        std::cout << "可用串口列表:" << std::endl;
        auto ports = serial_comm::SerialComm::get_available_ports();
        for (const auto& port : ports) {
            std::cout << "  " << port << std::endl;
        }
        return -1;
    }
    
    std::cout << "串口初始化成功!" << std::endl;
    
    // 开始异步接收协议数据
    serial.start_protocol_receive(data_handler, error_handler);
    std::cout << "开始异步接收协议数据..." << std::endl;
    
    // 发送测试数据
    std::cout << "\n发送测试数据:" << std::endl;
    
    // 测试1: 发送传感器数据
    std::vector<uint8_t> sensor_data = {0x12, 0x34, 0x56, 0x78};
    if (serial.send_protocol_data(0x01, 4, sensor_data)) {
        std::cout << "成功发送传感器数据 (ID: 0x01)" << std::endl;
    } else {
        std::cout << "发送传感器数据失败: " << serial.get_last_error() << std::endl;
    }
    
    // 测试2: 发送控制指令
    std::vector<uint8_t> control_cmd = {0x00, 0x01, 0xFF};
    if (serial.send_protocol_data(0x02, 3, control_cmd)) {
        std::cout << "成功发送控制指令 (ID: 0x02)" << std::endl;
    } else {
        std::cout << "发送控制指令失败: " << serial.get_last_error() << std::endl;
    }
    
    // 测试3: 发送状态信息
    std::vector<uint8_t> status_info = {0xAB, 0xCD};
    if (serial.send_protocol_data(0x03, 2, status_info)) {
        std::cout << "成功发送状态信息 (ID: 0x03)" << std::endl;
    } else {
        std::cout << "发送状态信息失败: " << serial.get_last_error() << std::endl;
    }
    
    // 等待接收数据
    std::cout << "\n等待接收数据... (按Enter键退出)" << std::endl;
    std::cin.get();
    
    // 清理
    serial.stop_protocol_receive();
    serial.close();
    
    std::cout << "程序结束" << std::endl;
    return 0;
}