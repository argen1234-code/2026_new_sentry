# usb_bridge

Jetson ↔ STM32H723 USB CDC 桥接节点。

## 链路

```text
/usb_bridge/state       <- MCU_STATE    (0x81, 50 Hz)
/chassis/state          <- WHEEL_STATE  (0x83, 50 Hz)
/imu/data               <- IMU_STATE    (0x84, 50 Hz)
/serial/joint_states_raw<- JointState   (由 0x83 生成，joint_state_publisher 合并)
/diagnostics            <- 1 Hz
/unified_cmd            -> UNIFIED_CMD  (0x10, 50 Hz)
```

节点启动后自动按 `VID:PID = 0483:5740` 查找 STM32，不会误开 DAPLink 的 CDC
或雷达串口。没有新命令或命令超过 `command_timeout` 时发送零命令；STM32 侧
500 ms 收不到主机帧会清除命令有效位，底盘 App 会 force_stop。

`cmd_vel_to_unified` 是可选节点：订阅 `/cmd_vel`，转换成 `UnifiedCmd`。
默认 `enable_motion=false`，只有显式开启并带 `chassis_status` 的
`CMD_VALID | CMD_MOTION_ENABLE` 位时，MCU 才接受底盘速度。

## 帧格式

```text
AA 55 | id | seq | len | payload[len] | xor
xor = id ^ seq ^ len ^ payload[0] ^ ... ^ payload[len-1]
```

| ID | 方向 | 内容 | 长度 |
|---|---|---|---:|
| `0x01` | Jetson → MCU | 主机心跳：`mode u8, flags u8` | 2 |
| `0x10` | Jetson → MCU | `UnifiedCmd` | 45 |
| `0x81` | MCU → Jetson | 兼容机器人状态 | 40 |
| `0x82` | MCU → Jetson | MCU 心跳 | 8 |
| `0x83` | MCU → Jetson | 四轮动态状态 | 58 |
| `0x84` | MCU → Jetson | IMU 动态状态 | 58 |

### `0x83 WheelState`（58 B）

```text
u32 mcu_ts_ms
u8  online_mask           # bit0..3 = RR, FL, RL, FR
u8  mode                  # ChassisMode
u16 chassis_status        # ChassisStatus
u16 fault_flags           # bit0..7 见下，bit8..11 = dm_fault_mask
f32 speed_mps[4]          # RR, FL, RL, FR
f32 position_rad[4]       # 电机侧 rad，直驱 1:1
u8  temp_mos_c[4]
u8  temp_motor_c[4]
i16 torque_mnm[4]         # 0.001 N*m
```

`fault_flags`：

- bit0 `dm_fault_latched`
- bit1 `dm_mode_fail_count > 0`
- bit2 `dm_tx_drop_count > 0`
- bit3 `imu_status != IMU_OK`
- bit4 `send_enable == 0`
- bit5 `online_mask != 0x0F`
- bit6 最近出现过周期 overrun
- bit7 `chassis_status != ChassisOk`

### `0x84 ImuState`（58 B）

```text
u32 mcu_ts_ms
f32 gyro[3]               # rad/s
f32 accel[3]              # m/s^2
f32 temperature_c
f32 quat_wxyz[4]          # valid_mask bit1 为 0 时无效
i32 imu_status
u8  valid_mask            # bit0 gyro+accel, bit1 attitude, bit2 temperature
u8  reserved
u32 last_ok_ms
```

### `UnifiedCmd`（45 B）

```text
i32 delta_yaw_udeg
i32 delta_pitch_udeg
u16 gimbal_status
i32 vx_mm_s
i32 vy_mm_s
i32 wz_mdeg_s
i32 chassis_mode
u16 chassis_status        # bit0 CMD_VALID, bit1 CMD_MOTION_ENABLE
u8  fire_on
i32 fire_mode
i32 burst_count
u64 host_ts_ns
```

## 运行

```bash
cd ~/2026_new_sentry/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to usb_bridge
source install/setup.bash

# 只读状态
ros2 launch usb_bridge usb_bridge.launch.py enable_motion:=false

# 允许 /cmd_vel 驱动底盘（先架空车轮）
ros2 launch usb_bridge usb_bridge.launch.py enable_motion:=true
```

检查：

```bash
ros2 topic echo /usb_bridge/state
ros2 topic echo /chassis/state
ros2 topic echo /imu/data
ros2 topic echo /joint_states
ros2 topic echo /diagnostics
```

## 安全约定

- `enable_motion` 默认 false；MCU 还要求 `chassis_status` 同时带 VALID 和
  MOTION_ENABLE，并检查 100 ms 命令新鲜度。
- 命令超时、链路断开、模式非法时，MCU 锁定 `usb_armed` 并 force_stop。
- `/cmd_vel` 已钳位到 0.45 m/s / 0.3 m/s / 1.0 rad/s，参数可调。
- 本节点仍可在 `enable_motion:=false` 下作为纯状态桥使用。
