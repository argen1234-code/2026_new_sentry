# SBUS 协议修改指南

## 概述

本指南说明如何将 `standard_robot_pp_ros2` 节点从自定义协议修改为 SBUS 协议，只发送 vx、vy、wz 三个速度数据。

## SBUS 协议简介

SBUS 是一种数字串行通信协议，常用于遥控器和接收机之间的通信。

### 协议特点

- **波特率**: 100000 bps（注意：不是 115200）
- **数据位**: 8 位
- **停止位**: 2 位
- **校验位**: 偶校验（Even Parity）
- **帧长度**: 25 字节
- **通道数**: 16 个通道，每个通道 11 位（0-2047）
- **信号反转**: SBUS 信号是反相的（需要硬件或软件反转）

### 帧结构

```
字节 0:      起始字节 (0x0F)
字节 1-22:   16 个通道数据（每个 11 位，打包存储）
字节 23:     标志位
字节 24:     结束字节 (0x00)
```

## 修改步骤

### 1. 修改头文件

在 `include/standard_robot_pp_ros2/standard_robot_pp_ros2.hpp` 中添加：

```cpp
#include "standard_robot_pp_ros2/sbus_protocol.hpp"

// 在类的 private 部分添加
bool use_sbus_protocol_;  // 是否使用 SBUS 协议
float max_linear_velocity_;  // 最大线速度 (m/s)
float max_angular_velocity_; // 最大角速度 (rad/s)
```

### 2. 修改源文件

#### 2.1 在 `getParams()` 函数中添加参数读取

在 `src/standard_robot_pp_ros2.cpp` 的 `getParams()` 函数末尾添加：

```cpp
// 读取协议类型
use_sbus_protocol_ = declare_parameter("use_sbus_protocol", false);

// 读取速度限制
max_linear_velocity_ = declare_parameter("max_linear_velocity", 3.0);
max_angular_velocity_ = declare_parameter("max_angular_velocity", 3.14);

// 如果使用 SBUS 协议，修改串口参数
if (use_sbus_protocol_) {
  RCLCPP_INFO(get_logger(), "Using SBUS protocol");
  // SBUS 协议参数：100000 bps, 8E2
  baud_rate = 100000;
  pt = Parity::EVEN;
  sb = StopBits::TWO;
  
  device_config_ = 
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}
```

#### 2.2 修改 `sendData()` 函数

将 `sendData()` 函数修改为：

```cpp
void StandardRobotPpRos2Node::sendData()
{
  RCLCPP_INFO(get_logger(), "Start sendData!");

  if (use_sbus_protocol_) {
    // 使用 SBUS 协议
    sendDataSbus();
  } else {
    // 使用原有协议
    sendDataOriginal();
  }
}
```

#### 2.3 添加 SBUS 发送函数

在 `sendData()` 函数后添加新函数：

```cpp
void StandardRobotPpRos2Node::sendDataSbus()
{
  RCLCPP_INFO(get_logger(), "Using SBUS protocol for data transmission");
  
  SbusFrame sbus_frame;
  sbus_frame.start_byte = SBUS_START_BYTE;
  sbus_frame.end_byte = SBUS_END_BYTE;
  sbus_frame.flags = 0;  // 无特殊标志
  
  // 初始化所有通道为中心值
  for (size_t i = 0; i < SBUS_NUM_CHANNELS; ++i) {
    sbus_frame.channels[i] = SBUS_CENTER_VALUE;
  }
  
  int retry_count = 0;
  
  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      RCLCPP_WARN(get_logger(), "send: usb is not ok! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }
    
    try {
      // 将速度值映射到 SBUS 通道
      // 通道 0: vx (前进速度)
      // 通道 1: vy (横向速度)
      // 通道 2: wz (旋转角速度)
      sbus_frame.channels[0] = SbusEncoder::mapValueToChannel(
        send_robot_cmd_data_.data.speed_vector.vx, max_linear_velocity_);
      sbus_frame.channels[1] = SbusEncoder::mapValueToChannel(
        send_robot_cmd_data_.data.speed_vector.vy, max_linear_velocity_);
      sbus_frame.channels[2] = SbusEncoder::mapValueToChannel(
        send_robot_cmd_data_.data.speed_vector.wz, max_angular_velocity_);
      
      // 编码 SBUS 帧
      std::vector<uint8_t> sbus_data = SbusEncoder::encode(sbus_frame);
      
      // SBUS 信号需要反转（如果硬件不支持反转）
      // 如果你的硬件支持信号反转，可以注释掉下面这行
      // sbus_data = SbusEncoder::invertBytes(sbus_data);
      
      // 发送数据
      serial_driver_->port()->send(sbus_data);
      
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error sending SBUS data: %s", ex.what());
      is_usb_ok_ = false;
    }
    
    // SBUS 标准发送频率约为 100Hz (10ms)
    // 可以根据需要调整
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void StandardRobotPpRos2Node::sendDataOriginal()
{
  // 原有的 sendData() 函数内容
  send_robot_cmd_data_.frame_header.sof = SOF_SEND;
  send_robot_cmd_data_.frame_header.id = ID_ROBOT_CMD;
  send_robot_cmd_data_.frame_header.len = sizeof(SendRobotCmdData) - 6;
  send_robot_cmd_data_.data.speed_vector.vx = 0;
  send_robot_cmd_data_.data.speed_vector.vy = 0;
  send_robot_cmd_data_.data.speed_vector.wz = 0;
  
  crc8::append_CRC8_check_sum(
    reinterpret_cast<uint8_t *>(&send_robot_cmd_data_), sizeof(HeaderFrame));

  int retry_count = 0;

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      RCLCPP_WARN(get_logger(), "send: usb is not ok! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    try {
      crc16::append_CRC16_check_sum(
        reinterpret_cast<uint8_t *>(&send_robot_cmd_data_), sizeof(SendRobotCmdData));

      std::vector<uint8_t> send_data = toVector(send_robot_cmd_data_);
      serial_driver_->port()->send(send_data);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error sending data: %s", ex.what());
      is_usb_ok_ = false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}
```

### 3. 修改配置文件

修改 `config/standard_robot_pp_ros2.yaml`：

```yaml
standard_robot_pp_ros2:
  ros__parameters:
    device_name: /dev/ttyUSB0
    
    # 协议选择
    use_sbus_protocol: true  # true=SBUS协议, false=原有协议
    
    # SBUS 协议会自动设置以下参数：
    # baud_rate: 100000
    # parity: even
    # stop_bits: "2"
    
    # 原有协议参数（use_sbus_protocol=false 时使用）
    baud_rate: 115200
    flow_control: none
    parity: none
    stop_bits: "1"
    
    # 速度限制（用于 SBUS 映射）
    max_linear_velocity: 3.0   # 最大线速度 (m/s)
    max_angular_velocity: 3.14 # 最大角速度 (rad/s)
    
    set_detector_color: false
    record_rosbag: false
    debug: false
```

### 4. 修改 CMakeLists.txt

在 `CMakeLists.txt` 中添加 SBUS 源文件：

找到类似这样的部分：
```cmake
add_library(${PROJECT_NAME} SHARED
  src/standard_robot_pp_ros2.cpp
  src/crc8_crc16.cpp
  src/gimbal_manager.cpp
)
```

修改为：
```cmake
add_library(${PROJECT_NAME} SHARED
  src/standard_robot_pp_ros2.cpp
  src/crc8_crc16.cpp
  src/gimbal_manager.cpp
  src/sbus_protocol.cpp  # 添加这一行
)
```

### 5. 在头文件中添加函数声明

在 `include/standard_robot_pp_ros2/standard_robot_pp_ros2.hpp` 的类定义中添加：

```cpp
private:
  void sendDataSbus();      // SBUS 协议发送
  void sendDataOriginal();  // 原有协议发送
```

## 编译和测试

### 编译

```bash
cd ~/printk_ws
colcon build --packages-select standard_robot_pp_ros2
source install/setup.bash
```

### 测试

1. **使用 SBUS 协议**：
```bash
# 修改配置文件中 use_sbus_protocol: true
ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py
```

2. **发送测试速度**：
```bash
# 发送前进速度
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 发送旋转速度
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

3. **监控串口数据**（可选）：
```bash
# 安装 minicom
sudo apt install minicom

# 监控串口（注意：SBUS 波特率是 100000）
sudo minicom -D /dev/ttyUSB0 -b 100000
```

## SBUS 数据映射

### 速度到通道值的映射

- **通道 0 (vx)**: 前进速度
  - -max_linear_velocity → 0
  - 0 → 1024 (中心值)
  - +max_linear_velocity → 2047

- **通道 1 (vy)**: 横向速度
  - -max_linear_velocity → 0
  - 0 → 1024 (中心值)
  - +max_linear_velocity → 2047

- **通道 2 (wz)**: 旋转角速度
  - -max_angular_velocity → 0
  - 0 → 1024 (中心值)
  - +max_angular_velocity → 2047

### 下位机解码示例

下位机接收 SBUS 数据后，需要将通道值转换回速度值：

```c
// 将 SBUS 通道值转换为速度
float channel_to_velocity(uint16_t channel_value, float max_velocity) {
    // channel_value 范围: 0-2047
    // 中心值: 1024
    float normalized = (channel_value - 1024.0f) / 1023.0f;  // 归一化到 [-1, 1]
    return normalized * max_velocity;
}

// 使用示例
float vx = channel_to_velocity(sbus_channels[0], 3.0);  // 最大 3.0 m/s
float vy = channel_to_velocity(sbus_channels[1], 3.0);
float wz = channel_to_velocity(sbus_channels[2], 3.14); // 最大 3.14 rad/s
```

## 注意事项

### 1. 波特率差异
- **原协议**: 115200 bps
- **SBUS**: 100000 bps
- 确保下位机也使用 100000 bps

### 2. 信号反转
SBUS 信号是反相的。有两种处理方式：
- **硬件反转**: 使用硬件电路反转信号（推荐）
- **软件反转**: 在代码中调用 `SbusEncoder::invertBytes()`

### 3. 发送频率
- **原协议**: 200 Hz (5ms)
- **SBUS**: 通常 100 Hz (10ms)
- 可根据需要调整

### 4. 数据精度
- SBUS 每个通道 11 位（0-2047），精度约为 0.05%
- 对于速度控制来说精度足够

### 5. 兼容性
修改后的代码保持向后兼容，可以通过配置文件切换协议：
- `use_sbus_protocol: true` → 使用 SBUS
- `use_sbus_protocol: false` → 使用原有协议

## 故障排查

### 问题 1: 下位机收不到数据
- 检查波特率是否为 100000
- 检查是否需要信号反转
- 检查串口设备是否正确

### 问题 2: 数据不正确
- 检查速度限制参数设置
- 检查通道映射是否正确
- 使用示波器或逻辑分析仪检查信号

### 问题 3: 编译错误
- 确保 `sbus_protocol.cpp` 已添加到 CMakeLists.txt
- 确保头文件路径正确
- 重新编译：`colcon build --packages-select standard_robot_pp_ros2 --cmake-clean-cache`

## 参考资料

- [SBUS 协议规范](https://github.com/bolderflight/sbus)
- [Futaba SBUS 文档](https://www.futabarc.com/sbus/)
- ROS 2 Serial Driver 文档

## 总结

通过以上修改，你的系统将：
1. 支持 SBUS 协议发送 vx、vy、wz 数据
2. 保持与原有协议的兼容性
3. 可通过配置文件轻松切换协议
4. 数据帧大小从 46 字节减少到 25 字节
5. 使用标准的遥控器协议，便于与其他系统集成