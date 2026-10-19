# Jetson ↔ STM32 通信协议（草案）

> 状态：草案，尚未冻结。协议事实源最终以 STM32 侧 `communication_host_schema.hpp` 为准，ROS 侧 `sentry_msgs` 与之保持字段一致。

## 1. 物理链路

| 项目 | 建议值 | 备注 |
|---|---|---|
| 主链路 | UART（USART2 / RS-485） | 沿用 old_sentry 的 Host 通信方式 |
| 备用链路 | USB CDC | 调试与高带宽场景 |
| 波特率 | 921600 8N1 | 以实测误码率为准 |
| 传输方式 | DMA + idle 中断收包 | STM32 侧 `communication_host` |
| 时间基准 | STM32 提供单调计数 | ROS 侧换算为 `ros::Time` |

## 2. 帧格式

沿用 `Tools/Printk-Protocol` 生成的 Proto2 RT 固定帧：

```text
+--------+--------+--------+--------+----------+--------+
| header | msg_id |  seq   |  len   | payload  | crc16  |
+--------+--------+--------+--------+----------+--------+
```

约定：

- `msg_id` 为 `uint8_t`，`0x00` 与 `0xFF` 保留。
- 单帧 payload ≤ 255 字节。
- `seq` 用于丢包与乱序检测。
- CRC 错误帧直接丢弃并计数，不回传错误帧。
- 所有多字节字段统一小端。

## 3. 消息清单（初稿）

### host_to_device

| msg_id | 名称 | 频率 | 内容 |
|---|---|---|---|
| 0x10 | `HostHeartbeat` | 10 Hz | 主机存活、模式请求 |
| 0x11 | `GimbalCmd` | 100–200 Hz | yaw/pitch 目标、角速度前馈、控制模式 |
| 0x12 | `ChassisCmd` | 50–100 Hz | vx/vy/wz、底盘模式 |
| 0x13 | `ShootCmd` | 事件/50 Hz | 发射触发、射频、摩擦轮使能 |
| 0x14 | `UiFrame` | ≤ 10 Hz | 裁判系统 UI 图元 |
| 0x15 | `HostModeCmd` | 事件 | 自主/遥控/测试模式切换 |

### device_to_host

| msg_id | 名称 | 频率 | 内容 |
|---|---|---|---|
| 0x20 | `DeviceHeartbeat` | 10 Hz | 固件状态、错误码、上电时间 |
| 0x21 | `RobotState` | 100 Hz | IMU 姿态、云台角度、底盘速度 |
| 0x22 | `MotorState` | 50 Hz | 关键电机角度/速度/电流/温度 |
| 0x23 | `RefereeState` | 10 Hz | 血量、热量、弹量、比赛阶段 |
| 0x24 | `ErrorReport` | 事件 | 模块、错误码、严重级别 |

> 具体 `msg_id`、字段和频率在实现前统一评审，未评审前不得写入生成代码。

## 4. 安全与超时

- `HostHeartbeat` 超过 200 ms 未更新：Jetson 指令降级为无效，STM32 进入遥控/安全模式。
- 云台、底盘、发射分别做幅值与变化率钳位，钳位在 STM32 侧完成。
- 任何模式切换必须先通过权限与状态检查。
- 通信统计（CRC 错误、丢帧、超时）进入 `ErrorReport` 与 Debug probe。

## 5. 协议生成流程

1. 修改 `stm32_ws/Component/Communication/Src/communication_host_schema.hpp`。
2. 执行：

   ```bash
   python tools/Printk-Protocol/generate.py \
       stm32_ws/Component/Communication/Src/communication_host_schema.hpp \
       --out-dir stm32_ws/Component/Communication/Src/Generate
   ```

3. 生成产物提交到仓库，禁止手改 `Generate/` 下的文件。
4. 同步更新 `ros_ws/src/sentry_msgs/msg/` 中的对应消息定义，并补充一致性测试。

## 6. ROS 侧映射

| 协议消息 | ROS 消息 | 方向 |
|---|---|---|
| `GimbalCmd` | `sentry_msgs/msg/GimbalCmd` | ROS → STM32 |
| `ChassisCmd` | `sentry_msgs/msg/ChassisCmd` | ROS → STM32 |
| `ShootCmd` | `sentry_msgs/msg/ShootCmd` | ROS → STM32 |
| `RobotState` | `sentry_msgs/msg/RobotState` | STM32 → ROS |
| `RefereeState` | `sentry_msgs/msg/RefereeState` | STM32 → ROS |
| `ErrorReport` | `sentry_msgs/msg/ErrorReport` | STM32 → ROS |

串口桥节点 `sentry_devices/serial_bridge` 负责帧收发、校验、时间戳与 topic 转换，其它包不直接操作串口。

## 7. USB CDC 桥接（已实现）

Jetson 与 STM32 之间的第一条链路使用 USB CDC，协议事实源：

- STM32：`stm32_ws/Component/Communication/Src/communication_usb.h`
- Jetson：`ros_ws/src/sentry_hardware/usb_bridge/usb_bridge/protocol.py`

完整字段布局见 [usb_bridge/README.md](../ros_ws/src/sentry_hardware/usb_bridge/README.md)。

| 项 | 值 |
|---|---|
| USB VID:PID | `0483:5740` |
| 设备名 | `Printk DM 02` |
| 帧格式 | `AA 55 \| id \| seq \| len \| payload \| xor` |
| 主机命令 | `0x01` 心跳、`0x10` `UnifiedCmd`（45 B） |
| MCU 上报 | `0x81` 状态（40 B）、`0x82` 心跳（8 B） |
| 链路超时 | 500 ms 无主机帧则清除命令有效位 |
| 周期 | 命令 50 Hz，MCU 状态 50 Hz，心跳 10 Hz |

> 修改字段时必须同时更新 `communication_usb.c`、`protocol.py` 和本文档；
> 帧长固定，任一方向长度不一致都会导致解析失败。
