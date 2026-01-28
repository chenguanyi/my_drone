# 协议通信使用说明

## 协议格式

本库实现的串口通信协议格式如下：

```
| 帧头 | 地址 | ID | 数据长度 | 数据内容 | 和校验 | 附加校验 |
| 0xAA | 0xFF | ID |   Len    |   Data   |  SUM   |   ADD    |
|  1   |  1   | 1  |    1     |   Len    |   1    |    1     |
```

### 字段说明

- **帧头 (0xAA)**: 固定的帧头标识
- **地址 (0xFF)**: 固定的地址字段
- **ID**: 数据类型标识符，用于区分不同类型的数据
- **数据长度**: 表示数据内容字段的字节数（0-255）
- **数据内容**: 实际传输的有效数据
- **和校验 (SUM)**: 从帧头到数据内容结束的累加和（低8位）
- **附加校验 (ADD)**: 和校验过程中每次累加后的累加和（低8位）

### 校验算法

**和校验计算方法：**
```cpp
uint8_t sum_check = 0;
for (每个字节 from 帧头 to 数据结束) {
    sum_check = (sum_check + 字节值) & 0xFF;
}
```

**附加校验计算方法：**
```cpp
uint8_t sum_check = 0;
uint8_t add_check = 0;
for (每个字节 from 帧头 to 数据结束) {
    sum_check = (sum_check + 字节值) & 0xFF;
    add_check = (add_check + sum_check) & 0xFF;
}
```

## API 使用方法

### 1. 发送协议数据

```cpp
#include "serial_comm/serial_comm.h"

serial_comm::SerialComm serial;

// 初始化串口
if (serial.initialize("/dev/ttyUSB0", 115200)) {
    // 准备要发送的数据
    std::vector<uint8_t> data = {0x12, 0x34, 0x56, 0x78};
    
    // 发送协议数据
    uint8_t id = 0x01;          // 数据类型ID
    uint8_t length = 4;         // 数据长度
    
    if (serial.send_protocol_data(id, length, data)) {
        std::cout << "发送成功" << std::endl;
    } else {
        std::cout << "发送失败: " << serial.get_last_error() << std::endl;
    }
}
```

### 2. 接收协议数据

```cpp
#include "serial_comm/serial_comm.h"

// 定义数据处理回调函数
void data_handler(uint8_t id, const std::vector<uint8_t>& data) {
    switch (id) {
        case 0x01:
            // 处理传感器数据
            process_sensor_data(data);
            break;
        case 0x02:
            // 处理控制指令
            process_control_command(data);
            break;
        case 0x03:  
            // 处理状态信息
            process_status_info(data);
            break;
        default:
            std::cout << "未知数据类型: " << std::hex << (int)id << std::endl;
            break;
    }
}

// 错误处理回调函数
void error_handler(const std::string& error_msg) {
    std::cout << "通信错误: " << error_msg << std::endl;
}

int main() {
    serial_comm::SerialComm serial;
    
    // 初始化串口
    if (serial.initialize("/dev/ttyUSB0", 115200)) {
        // 开始异步接收协议数据
        serial.start_protocol_receive(data_handler, error_handler);
        
        // 主程序继续运行...
        // 接收到的数据会自动调用data_handler处理
        
        // 停止接收
        serial.stop_protocol_receive();
    }
    
    return 0;
}
```

## 示例程序

编译完成后可以运行示例程序：

```bash
# 运行协议通信示例
./devel/lib/serial_comm/serial_comm_protocol_example
```

## 注意事项

1. **数据长度限制**: 单次发送的数据内容最大为255字节
2. **串口权限**: 确保用户有串口设备的读写权限
3. **异步接收**: 数据接收是异步的，需要保持程序运行
4. **错误处理**: 务必检查发送函数的返回值和错误信息
5. **缓冲区管理**: 接收缓冲区会自动管理，但避免长时间不处理数据

## 调试技巧

1. **查看可用串口**:
```cpp
auto ports = serial_comm::SerialComm::get_available_ports();
for (const auto& port : ports) {
    std::cout << port << std::endl;
}
```

2. **获取错误信息**:
```cpp
if (!serial.send_protocol_data(id, length, data)) {
    std::cout << "错误: " << serial.get_last_error() << std::endl;
}
```

3. **检查串口状态**:
```cpp
if (serial.is_open()) {
    std::cout << "串口已打开" << std::endl;
}
```