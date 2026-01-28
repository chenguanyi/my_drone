# Serial Communication Library

一个基于boost::asio实现的高性能串口通讯库，专为ROS环境设计，但不包含任何ROS节点或话题，仅作为其他软件包的依赖库使用。

## 特性

- 🚀 **高性能**: 基于boost::asio的异步IO框架
- 🎯 **简单易用**: 万能初始化函数，仅需串口名和波特率
- 🔧 **高度可配置**: 支持自定义数据位、校验位、停止位、流控制等
- 📡 **同步/异步**: 支持同步和异步读写操作
- 🛡️ **异常安全**: 完善的错误处理和异常安全设计
- 🔍 **设备发现**: 自动扫描可用串口设备
- ⏱️ **超时控制**: 支持读取操作超时设置

## 快速开始

### 1. 基本使用（万能初始化）

```cpp
#include "serial_comm/serial_comm.h"

// 创建串口对象
serial_comm::SerialComm serial;

// 使用万能初始化函数（仅需串口名和波特率）
if (serial.initialize("/dev/ttyUSB0", 115200)) {
    // 发送数据
    serial.write("Hello, World!\n");
    
    // 读取一行数据
    std::string line;
    if (serial.read_line(line, 1000)) {  // 1秒超时
        std::cout << "接收到: " << line << std::endl;
    }
    
    serial.close();
}
```

### 2. 高级配置

```cpp
serial_comm::SerialConfig config;
config.port_name = "/dev/ttyUSB0";
config.baud_rate = 9600;
config.character_size = 8;
config.parity = boost::asio::serial_port::parity::none;
config.stop_bits = boost::asio::serial_port::stop_bits::one;
config.flow_control = boost::asio::serial_port::flow_control::none;

serial_comm::SerialComm serial;
if (serial.initialize(config)) {
    // 发送二进制数据
    std::vector<uint8_t> data = {0x01, 0x02, 0x03};
    serial.write(data);
    
    // 读取二进制数据
    std::vector<uint8_t> buffer;
    int bytes_read = serial.read(buffer, 10, 500);  // 最多10字节，500ms超时
}
```

### 3. 异步通讯

```cpp
serial_comm::SerialComm serial;
if (serial.initialize("/dev/ttyUSB0", 115200)) {
    // 开始异步读取
    serial.start_async_read(
        // 数据接收回调
        [](const std::vector<uint8_t>& data) {
            std::cout << "接收到数据" << std::endl;
        },
        // 错误回调
        [](const std::string& error) {
            std::cout << "错误: " << error << std::endl;
        }
    );
    
    // 异步发送
    std::vector<uint8_t> data = {'H', 'i', '\n'};
    serial.async_write(data, 
        [](const boost::system::error_code& ec, size_t bytes) {
            if (!ec) {
                std::cout << "发送完成: " << bytes << " 字节" << std::endl;
            }
        }
    );
}
```

## API参考

### 核心类: SerialComm

#### 初始化方法

- `bool initialize(const std::string& port_name, unsigned int baud_rate)`
  - 万能初始化函数，仅需串口名和波特率
  - 使用默认配置：8数据位，无校验，1停止位，无流控

- `bool initialize(const SerialConfig& config)`
  - 高级初始化函数，支持完整配置

#### 同步读写

- `int write(const std::string& data)` - 发送字符串
- `int write(const std::vector<uint8_t>& data)` - 发送二进制数据
- `int read(std::vector<uint8_t>& buffer, size_t max_size, unsigned int timeout_ms = 0)` - 读取数据
- `bool read_line(std::string& line, unsigned int timeout_ms = 0)` - 读取一行

#### 异步读写

- `void async_write(const std::vector<uint8_t>& data, callback)` - 异步发送
- `void start_async_read(DataReceivedCallback, ErrorCallback)` - 开始异步读取
- `void stop_async_read()` - 停止异步读取

#### 工具方法

- `bool is_open()` - 检查串口是否打开
- `void close()` - 关闭串口
- `std::string get_last_error()` - 获取最后的错误信息
- `static std::vector<std::string> get_available_ports()` - 获取可用串口列表

## 在其他ROS包中使用

### 1. 在CMakeLists.txt中添加依赖

```cmake
find_package(catkin REQUIRED COMPONENTS
  serial_comm
  # 其他依赖...
)

catkin_package(
  CATKIN_DEPENDS serial_comm
  # 其他配置...
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  # 其他包含目录...
)

target_link_libraries(your_target
  ${catkin_LIBRARIES}
  # 其他库...
)
```

### 2. 在package.xml中添加依赖

```xml
<build_depend>serial_comm</build_depend>
<exec_depend>serial_comm</exec_depend>
```

### 3. 在代码中使用

```cpp
#include "serial_comm/serial_comm.h"

// 在你的ROS节点或类中使用
class YourROSNode {
private:
    serial_comm::SerialComm serial_;
    
public:
    void init_serial() {
        if (serial_.initialize("/dev/ttyUSB0", 115200)) {
            ROS_INFO("串口初始化成功");
        } else {
            ROS_ERROR("串口初始化失败: %s", serial_.get_last_error().c_str());
        }
    }
};
```

## 编译和安装

```bash
# 进入工作空间
cd ~/ros1_basic_ws

# 编译
catkin_make

# 或者只编译serial_comm包
catkin_make --only-pkg-with-deps serial_comm

# 运行示例程序（可选）
./devel/lib/serial_comm/serial_comm_example
```

## 系统要求

- **操作系统**: Linux (Ubuntu 16.04+)
- **ROS版本**: ROS Kinetic 或更新版本
- **依赖库**: 
  - boost-system (≥1.58)
  - boost-thread (≥1.58)
  - pthread
- **编译器**: GCC 5.4+ (支持C++14)

## 常见问题

### Q: 串口权限问题
A: 将用户添加到dialout组：
```bash
sudo usermod -a -G dialout $USER
# 注销重新登录生效
```

### Q: 找不到串口设备
A: 使用库提供的函数检查可用串口：
```cpp
auto ports = serial_comm::SerialComm::get_available_ports();
```

### Q: 编译错误
A: 确保安装了所有依赖：
```bash
sudo apt-get install libboost-system-dev libboost-thread-dev
```

## 许可证

MIT License - 详情请见LICENSE文件

## 贡献

欢迎提交Issue和Pull Request来改进这个库。

## 更新日志

### v1.0.0
- 初始版本发布
- 支持基本的串口通讯功能
- 万能初始化函数
- 同步和异步读写支持
- 完整的错误处理机制