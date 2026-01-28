#include "serial_comm/serial_comm.h"
#include <iostream>
#include <vector>
#include <string>

/**
 * @brief 串口通讯库使用示例
 * 
 * 本示例展示了如何使用serial_comm库进行串口通讯
 * 包括初始化、同步读写、异步读写等功能
 */

void example_basic_usage() {
    std::cout << "=== 基本使用示例 ===" << std::endl;
    
    // 创建串口通讯对象
    serial_comm::SerialComm serial;
    
    // 使用万能初始化函数（仅需串口名和波特率）
    std::string port_name = "/dev/ttyUSB0";  // 根据实际情况修改
    unsigned int baud_rate = 115200;
    
    if (serial.initialize(port_name, baud_rate)) {
        std::cout << "串口初始化成功: " << port_name << " @ " << baud_rate << std::endl;
        
        // 发送字符串数据
        std::string message = "Hello, Serial!\n";
        int bytes_written = serial.write(message);
        if (bytes_written > 0) {
            std::cout << "发送了 " << bytes_written << " 字节数据" << std::endl;
        }
        
        // 读取一行数据（带超时）
        std::string received_line;
        if (serial.read_line(received_line, 1000)) {  // 1秒超时
            std::cout << "接收到: " << received_line << std::endl;
        } else {
            std::cout << "读取超时或失败: " << serial.get_last_error() << std::endl;
        }
        
        serial.close();
    } else {
        std::cout << "串口初始化失败: " << serial.get_last_error() << std::endl;
    }
}

void example_advanced_config() {
    std::cout << "\n=== 高级配置示例 ===" << std::endl;
    
    serial_comm::SerialComm serial;
    
    // 使用高级配置
    serial_comm::SerialConfig config;
    config.port_name = "/dev/ttyUSB0";
    config.baud_rate = 9600;
    config.character_size = 8;
    config.parity = boost::asio::serial_port::parity::none;
    config.stop_bits = boost::asio::serial_port::stop_bits::one;
    config.flow_control = boost::asio::serial_port::flow_control::none;
    
    if (serial.initialize(config)) {
        std::cout << "使用高级配置初始化成功" << std::endl;
        
        // 发送二进制数据
        std::vector<uint8_t> binary_data = {0x01, 0x02, 0x03, 0x04, 0x05};
        int bytes_written = serial.write(binary_data);
        std::cout << "发送了 " << bytes_written << " 字节二进制数据" << std::endl;
        
        // 读取二进制数据
        std::vector<uint8_t> received_data;
        int bytes_read = serial.read(received_data, 10, 500);  // 最多读取10字节，500ms超时
        if (bytes_read > 0) {
            std::cout << "接收到 " << bytes_read << " 字节数据: ";
            for (uint8_t byte : received_data) {
                std::cout << "0x" << std::hex << (int)byte << " ";
            }
            std::cout << std::endl;
        }
        
        serial.close();
    } else {
        std::cout << "高级配置初始化失败: " << serial.get_last_error() << std::endl;
    }
}

void example_async_communication() {
    std::cout << "\n=== 异步通讯示例 ===" << std::endl;
    
    serial_comm::SerialComm serial;
    
    if (serial.initialize("/dev/ttyUSB0", 115200)) {
        std::cout << "开始异步通讯..." << std::endl;
        
        // 设置异步读取回调
        serial.start_async_read(
            // 数据接收回调
            [](const std::vector<uint8_t>& data) {
                std::cout << "异步接收到数据: ";
                for (uint8_t byte : data) {
                    if (std::isprint(byte)) {
                        std::cout << (char)byte;
                    } else {
                        std::cout << "[0x" << std::hex << (int)byte << "]";
                    }
                }
                std::cout << std::endl;
            },
            // 错误回调
            [](const std::string& error) {
                std::cout << "异步读取错误: " << error << std::endl;
            }
        );
        
        // 异步发送数据
        std::vector<uint8_t> async_data = {'A', 's', 'y', 'n', 'c', '\n'};
        serial.async_write(async_data, 
            [](const boost::system::error_code& ec, size_t bytes_transferred) {
                if (!ec) {
                    std::cout << "异步发送完成，发送了 " << bytes_transferred << " 字节" << std::endl;
                } else {
                    std::cout << "异步发送失败: " << ec.message() << std::endl;
                }
            }
        );
        
        // 让异步操作运行一段时间
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        serial.stop_async_read();
        serial.close();
    } else {
        std::cout << "异步通讯初始化失败: " << serial.get_last_error() << std::endl;
    }
}

void list_available_ports() {
    std::cout << "\n=== 可用串口列表 ===" << std::endl;
    
    auto ports = serial_comm::SerialComm::get_available_ports();
    if (ports.empty()) {
        std::cout << "未找到可用的串口设备" << std::endl;
    } else {
        std::cout << "找到 " << ports.size() << " 个串口设备:" << std::endl;
        for (const auto& port : ports) {
            std::cout << "  - " << port << std::endl;
        }
    }
}

int main() {
    std::cout << "Serial Communication Library Demo" << std::endl;
    std::cout << "=================================" << std::endl;
    
    try {
        // 显示可用串口
        list_available_ports();
        
        // 基本使用示例
        example_basic_usage();
        
        // 高级配置示例
        example_advanced_config();
        
        // 异步通讯示例
        example_async_communication();
        
    } catch (const std::exception& e) {
        std::cout << "发生异常: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "\n示例程序执行完成" << std::endl;
    return 0;
}