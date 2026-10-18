# SBUS 协议快速参考

## 核心信息

### 串口参数
```
波特率: 100000 bps
数据位: 8
停止位: 2
校验位: 偶校验 (Even)
```

### 数据帧结构 (25 字节)
```
[0x0F] [22字节通道数据] [标志位] [0x00]
```

### 通道映射
- **通道 0**: vx (前进速度)
- **通道 1**: vy (横向速度)  
- **通道 2**: wz (旋转角速度)
- **通道 3-15**: 未使用（保持中心值 1024）

### 数值范围
- **通道值**: 0 - 2047 (11位)
- **中心值**: 1024
- **速度映射**: 
  - 0 → -max_velocity
  - 1024 → 0
  - 2047 → +max_velocity

## 快速开始

### 1. 配置文件设置
```yaml
# config/standard_robot_pp_ros2.yaml
standard_robot_pp_ros2:
  ros__parameters:
    device_name: /dev/ttyUSB0
    use_sbus_protocol: true
    max_linear_velocity: 3.0
    max_angular_velocity: 3.14
```

### 2. 编译
```bash
cd ~/printk_ws
colcon build --packages-select standard_robot_pp_ros2
source install/setup.bash
```

### 3. 运行
```bash
ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py
```

### 4. 测试
```bash
# 前进
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 旋转
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

## 下位机解码示例

### C/C++ 代码
```c
// SBUS 帧解析
typedef struct {
    uint16_t channels[16];
    uint8_t flags;
} SbusData;

// 解析 SBUS 帧
bool parseSbusFrame(uint8_t* data, SbusData* sbus) {
    if (data[0] != 0x0F || data[24] != 0x00) {
        return false;  // 帧头/帧尾错误
    }
    
    // 解包通道数据（11位打包）
    sbus->channels[0]  = ((data[1]    | data[2]<<8)                 & 0x07FF);
    sbus->channels[1]  = ((data[2]>>3 | data[3]<<5)                 & 0x07FF);
    sbus->channels[2]