# PID控制器软件包

这个ROS软件包实现了一个位置PID控制器，可以订阅目标位置和当前高度信息，并输出控制速度命令。

## 功能描述

该软件包包含一个PID控制节点，可以：

1. **订阅话题**：
   - `/tf`: 监听`map` -> `laser_link`的变换，获取当前x, y, yaw角数据
   - `/target_position` (`geometry_msgs/PoseStamped`): 目标位置信息，包括x, y, z, yaw
   - `/height` (`std_msgs/Float64`): 当前z轴高度信息

2. **发布话题**：
   - `/target_velocity` (`geometry_msgs/Twist`): 输出的目标速度命令

## 文件结构

```
pid_control_pkg/
├── include/
│   └── pid_controller.h          # PID控制器头文件
├── src/
│   └── pid_controller.cpp        # PID控制器实现
├── launch/
│   └── position_pid_controller.launch  # 启动文件
├── CMakeLists.txt
└── package.xml
```

## 编译

在你的catkin工作空间中编译：

```bash
cd ~/ros1_basic_ws
catkin_make
source devel/setup.bash
```

## 使用方法

### 1. 直接运行节点

```bash
rosrun pid_control_pkg position_pid_controller
```

### 2. 使用launch文件

```bash
roslaunch pid_control_pkg position_pid_controller.launch
```

## 参数配置

可以通过修改launch文件或使用rosparam设置以下参数：

### 控制参数
- `control_frequency`: 控制频率 (默认: 50.0 Hz)
- `map_frame`: 地图坐标系名称 (默认: "map")
- `laser_link_frame`: 激光雷达坐标系名称 (默认: "laser_link")

### PID参数
- `kp_xy`, `ki_xy`, `kd_xy`: X-Y位置PID参数 (默认: 1.0, 0.0, 0.1)
- `kp_yaw`, `ki_yaw`, `kd_yaw`: Yaw角度PID参数 (默认: 2.0, 0.0, 0.2)  
- `kp_z`, `ki_z`, `kd_z`: Z高度PID参数 (默认: 1.5, 0.0, 0.15)

### 速度限制
- `max_linear_velocity`: 最大线速度 (默认: 2.0 m/s)
- `max_angular_velocity`: 最大角速度 (默认: 1.0 rad/s)
- `max_vertical_velocity`: 最大垂直速度 (默认: 1.0 m/s)

## 话题接口

### 订阅话题

1. **`/target_position`** (`geometry_msgs/PoseStamped`)
   - 目标位置和姿态
   - 包含x, y, z位置和四元数表示的姿态

2. **`/height`** (`std_msgs/Float64`)
   - 当前高度信息
   - 用于Z轴控制

### 发布话题

1. **`/target_velocity`** (`geometry_msgs/Twist`)
   - 目标速度命令
   - 包含线速度(x, y, z)和角速度(roll, pitch, yaw)

## TF变换

节点需要以下TF变换可用：
- `map` -> `laser_link`: 用于获取当前位置和姿态

## 测试示例

### 发送目标位置

```bash
# 发送目标位置 (x=2.0, y=1.0, z=0.5, yaw=0.5)
rostopic pub /target_position geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: 2.0
    y: 1.0
    z: 0.5
  orientation:
    x: 0.0
    y: 0.0
    z: 0.247
    w: 0.969"
```

### 发送高度信息

```bash
# 发送当前高度
rostopic pub /height std_msgs/Float64 "data: 0.3"
```

### 查看输出速度

```bash
# 监听输出速度
rostopic echo /target_velocity
```

## 依赖项

- `roscpp`
- `rospy`
- `std_msgs`
- `geometry_msgs`
- `tf`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `angles`

## 注意事项

1. 确保TF变换`map` -> `laser_link`正确发布
2. PID参数需要根据实际机器人特性进行调整
3. 速度限制参数应该符合机器人的安全操作范围
4. 控制频率建议设置在20-100Hz之间以获得良好的控制性能