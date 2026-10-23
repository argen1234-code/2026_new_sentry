# Jetson ↔ STM32 通信协议（草案）

> 状态：草案，尚未冻结。协议事实源最终以 STM32 侧 `communication_host_schema.hpp` 为准，ROS 侧 `sentry_interfaces` 与之保持字段一致。

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

1. 修改 `stm32_ws/legacy/Component/Communication/Src/communication_host_schema.hpp`。
2. 执行：

   ```bash
   python stm32_ws/legacy/Tools/Printk-Protocol/generate.py \
       stm32_ws/legacy/Component/Communication/Src/communication_host_schema.hpp \
       --out-dir stm32_ws/legacy/Component/Communication/Src/Generate
   ```

3. 生成产物提交到仓库，禁止手改 `Generate/` 下的文件。
4. 同步更新 `ros_ws/src/sentry_interfaces/msg/` 中的对应消息定义，并补充一致性测试。

## 6. ROS 侧映射

| 协议消息 | ROS 消息 | 方向 |
|---|---|---|
| `GimbalCmd` | `sentry_interfaces/msg/GimbalCmd` | ROS → STM32 |
| `ChassisCmd` | `sentry_interfaces/msg/ChassisCmd` | ROS → STM32 |
| `ShootCmd` | `sentry_interfaces/msg/ShootCmd` | ROS → STM32 |
| `RobotState` | `sentry_interfaces/msg/RobotState` | STM32 → ROS |
| `RefereeState` | `sentry_interfaces/msg/RefereeState` | STM32 → ROS |
| `ErrorReport` | `sentry_interfaces/msg/ErrorReport` | STM32 → ROS |

USB 桥节点 `sentry_hardware/usb_bridge` 负责帧收发、校验、时间戳与 topic 转换，其它包不直接操作串口/USB。

### 速度符号（车体系 vs 固件内部系）

协议里的 `vx/vy/wz` 按 REP-103（x 前、y 左、z 上、wz 逆时针为正）定义，但固件
内部系的符号由 `chassis_fw/App/common/robot_param.h` 里的
`CHASSIS_WHEEL_20x_DIRECTION` 决定，两者不保证一致。底盘是开环轮速环，符号错
了不会报故障，只会表现为“规划往前、车往后”。

**2026-10-23 结论：真正的偏差在固件侧**——`chas_inv_cal` 沿用上游
“x 向右、y 向前”的内部系，主机按 REP-103 下发时整车只会横移，表现就是
“车沿规划路径的垂直方向走 90°”。

实车标定出的修正量是**顺时针 90°**（相对上游内部系）：即
`rear/front = -vx ± r·wz`、`right/left = +vy ∓ r·wz`，
`chas_for_cal` 同步为 `vx = -½(rear+front)`、`vy = ½(right+left)`。
先按逆时针 90° 改过一次，实车偏差从 90° 变成 180°（沿路径倒着走），
方向由此确定。`Omni_chassis.c` 里两处加速度前馈/制动补偿的
`wheel_force[]` 必须用同一套轮端映射，否则前馈会把车往侧面推。

**同日第二处修正（自转）**：用 MID360 陀螺仪做独立信源，下发 `+1.0 rad/s`
（REP-103 俯视逆时针）时 `gyro.z` 实测为负、车体实际顺时针转。轮端分布
`(+,-,-,+)` 本身正确，是整体符号反了，故 `wz` 项在 `chas_inv_cal` /
`chas_for_cal` 里整体取负，`Omni_chassis.c` 两处偏航力矩前馈同步取负。

**同日第三处修正（平移 45°）**：固件修正后重新实测——下发 `/cmd_vel`
`-x`（0.25 m/s）时车身实际走向为车体系 226°，下发 `+y` 时为 134°，两次
独立测量都多 **+45°**。即轮系坐标系相对 LIO 发布的 `base_footprint` 转了
45°。这一层在 ROS 侧用 `frame_yaw` 补偿。

桥接节点的 xy 旋转：

```text
[vx_fw]   [ cos(a)  -sin(a) ] [vx_ros]
[vy_fw] = [ sin(a)   cos(a) ] [vy_ros]
```

`a` 即参数 `frame_yaw`，实车取 **-π/4**。`wz` 与平面旋转无关，单独用
`sign_wz` 校正（默认 1.0，与固件 `HOST_CHASSIS_WZ_SIGN` 配合后正确）。

| 参数 | 默认 | 含义 |
|---|---|---|
| `frame_yaw` | **-0.785398** | 轮系相对 `base_footprint` 的偏航，xy 速度旋转量 |
| `sign_wz` | 1.0 | 自转方向 |

调法（不用重启，下一个 20 ms 周期生效）：

```bash
ros2 param set /cmd_vel_to_unified frame_yaw -0.785398   # 顺时针 45°（当前标定值）
ros2 param set /cmd_vel_to_unified frame_yaw 0.0         # 不旋转
```

标定顺序：先 `frame_yaw`（下发纯 +x，看车实际走向），再 `sign_wz`（原地转）。

若 `frame_yaw` 只是掩盖了两边坐标系不一致，根治办法是让固件的车体系
（`CHASSIS_WHEEL_20x_DIRECTION` 与安装方位）与 ROS 侧一致，桥接节点再退回单位旋转。

## 7. USB CDC 桥接（已实现）

Jetson 与 STM32 之间的第一条链路使用 USB CDC，协议事实源：

- STM32：`stm32_ws/legacy/Component/Communication/Src/communication_usb.h`
- Jetson：`ros_ws/src/sentry_hardware/usb_bridge/usb_bridge/protocol.py`

完整字段布局见 [usb_bridge/README.md](../ros_ws/src/sentry_hardware/usb_bridge/README.md)。

| 项 | 值 |
|---|---|
| USB VID:PID | `0483:5740` |
| 设备名 | `Printk DM 02` |
| 帧格式 | `AA 55 \| id \| seq \| len \| payload \| xor` |
| 主机命令 | `0x01` 心跳（2 B）、`0x10` `UnifiedCmd`（45 B） |
| MCU 上报 | `0x81` 兼容状态（40 B）、`0x82` 心跳（8 B）、`0x83` 四轮状态（58 B）、`0x84` IMU 状态（58 B） |
| 链路超时 | 500 ms 无主机帧则清除命令有效位 |
| 周期 | 命令 50 Hz，`0x81/0x83/0x84` 各 50 Hz，心跳 10 Hz |
| ROS 话题 | `/usb_bridge/state`、`/chassis/state`、`/imu/data`、`/joint_states`、`/diagnostics` |

轮序固定为 `RR, FL, RL, FR`；`0x10` 的 `chassis_status` bit0 为 `CMD_VALID`，
bit1 为 `CMD_MOTION_ENABLE`，MCU 还要求命令在 100 ms 内到达。

### 上游 sentinel 固件的兼容实现

`stm32_ws/sentinel/` 通过
`User/Communication/legacy/legacy_bridge.c` 实现同一套帧格式，使现有
`usb_bridge` 无需修改即可驱动新固件：

- USB 产品名是 `STM32 Virtual ComPort`，VID:PID 仍是 `0483:5740`；
- 下行 `0x01/0x10` 由兼容通道解析，其余字节交给 uproto；
- 上行 `0x81/0x82/0x83/0x84` 由 `chassis_move`、3508 反馈和 HWT906 组装；
- 旧协议主机在线时暂停 uproto 发送，避免两种帧交错。

模式映射、四轮数组顺序和验证清单见
[`stm32_ws/sentinel/UPSTREAM.md`](../stm32_ws/sentinel/UPSTREAM.md)。

> 修改字段时必须同时更新 `communication_usb.c`、`protocol.py` 和本文档；
> 帧长固定，任一方向长度不一致都会导致解析失败。改动 `sentinel/` 的兼容
> 通道时也必须同步 `legacy_proto.h` 与 `tools/host_tests/legacy_proto_test.c`。
