# usb_bridge

Jetson ↔ STM32H723 USB CDC 桥接节点。

## 链路

```text
/usb_bridge/state  <- MCU_STATE (0x81, 50 Hz)  <- STM32 USB CDC (0483:5740)
/unified_cmd       -> UNIFIED_CMD (0x10, 50 Hz) -> STM32 USB CDC
```

节点启动后自动按 `VID:PID = 0483:5740` 查找 STM32，不会误开 DAPLink 的 CDC
或雷达串口。没有新命令或命令超过 `command_timeout` 时，发送零命令；STM32
侧 500 ms 收不到主机帧会自动清除命令有效位。

## 帧格式

```text
AA 55 | id | seq | len | payload[len] | xor
xor = id ^ seq ^ len ^ payload[0] ^ ... ^ payload[len-1]
```

| ID | 方向 | 内容 |
|---|---|---|
| `0x01` | Jetson → MCU | 主机心跳：`mode u8, flags u8` |
| `0x10` | Jetson → MCU | `UnifiedCmd`，45 字节小端 |
| `0x81` | MCU → Jetson | 机器人状态，40 字节小端 |
| `0x82` | MCU → Jetson | MCU 心跳，8 字节小端 |

`UnifiedCmd` 布局：

```text
i32 delta_yaw_udeg
i32 delta_pitch_udeg
u16 gimbal_status
i32 vx_mm_s
i32 vy_mm_s
i32 wz_mdeg_s
i32 chassis_mode
u16 chassis_status
u8  fire_on
i32 fire_mode
i32 burst_count
u64 host_ts_ns
```

MCU 状态布局：

```text
u32 mcu_ts_ms
f32 imu_roll, imu_pitch, imu_yaw
f32 gimbal_yaw, gimbal_pitch
f32 vx, vy, wz
u8  mode, status
u16 error_flags
```

协议实现：

- Jetson：[`usb_bridge/protocol.py`](usb_bridge/protocol.py)
- STM32：[`stm32_ws/Component/Communication/Src/communication_usb.c`](../../../../stm32_ws/Component/Communication/Src/communication_usb.c)

修改字段时必须两边同时改，并保持 45 / 40 字节长度不变。

## 运行

```bash
cd ~/2026_new_sentry/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to usb_bridge
source install/setup.bash

ros2 launch usb_bridge usb_bridge.launch.py
```

检查：

```bash
ros2 topic echo /usb_bridge/state
ros2 topic info /unified_cmd
```

## 稳定设备名（可选）

```bash
sudo cp udev/99-sentry-stm32.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

之后可使用 `/dev/sentry_stm32`，并把 `port` 参数设为该路径。

## 安全约定

- 本节点只做桥接，不直接产生电机输出。
- STM32 侧只缓存命令，当前 target 的控制 App 尚未消费该命令。
- 后续接入控制链路时，必须在 App 层做超时、模式合法性检查和输出钳位。
