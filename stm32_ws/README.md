# STM32 固件工作空间

面向 STM32H723 控制板的电控固件，分层与命名沿用 `2026_old_sentry`。

## 目录职责

| 目录 | 职责 |
|---|---|
| `Algorithm` | 纯算法：PID、Mahony、Kalman、数学工具、弹道纯函数。 |
| `Bsp` | 板级外设句柄表和 HAL 薄封装，只描述板子提供什么外设能力。 |
| `Devices` | 设备驱动和协议解析：RC、IMU、电机、裁判系统、Flash、蜂鸣器、弹仓舵机。 |
| `Component` | 可复用业务组件：Controller、Input、Observer、Communication、Debug、Ui、Aim。 |
| `Apps` | 应用层 glue，平铺维护 `app_*.c/.h/_config.h`，负责读取设备与组件快照、组装输入、调度组件、执行输出。 |
| `Robot` | 具体车辆 target 配置入口，按“一台车一组完整事实”维护。 |
| `Tasks` | 工程主入口和 RTOS app 启动顺序；`main.c` + `firmware_main.h` 持有真正 `main()`，`Src/task_main.c/.h` 只编排 Apps。 |
| `Template` | 新增模块的模板文件与注释规范。 |
| `Tools` | 协议生成器、调试辅助工具。 |
| `Boards` | CubeMX/HAL/Keil 工程与板级生成文件。 |
| `ThirdParty` | 第三方库，CMSIS-DSP 使用这里的版本。 |
| `Doc` | 固件侧编码规范与工程差异说明。 |

## 依赖方向

```text
Tasks -> Robot -> Apps -> Component -> Devices -> Bsp -> Boards
                              \
                               Algorithm（纯算法，可被任意层调用）
```

- 下层不得 include 上层头文件。
- `Devices` 不出现具体机器人业务语义。
- `Apps` 不手写协议解析和电机控制算法。
- `Robot` 只放配置事实，不放实现。

## 固件选项（编译期选择）

固件通过 CMake 缓存变量 `ROBOT_TARGET` 选择具体车辆，同一套源码编译出不同固件：

| 选项 | 说明 | CAN 电机 |
|---|---|---|
| `sentry_2026` | 2026 哨兵，默认选项 | 使用 |
| `sentry_2026_bringup` | 2026 哨兵台架：仅 Communication + USB 桥接 | 不启动 |
| `can1_dmh6215_test` | CAN1 单电机 DMH6215 台架测试 | 仅 CAN1 |
| `infantry_steer_2025` | 2025 舵轮步兵 | 使用 |
| `board_only_dm02` | DM02 光板板载资源测试 | 不启动 |

构建（在 `Boards/` 下）：

```bash
cd Boards

# 默认：sentry_2026
cmake --preset Debug
cmake --build --preset Debug

# 切换到其它选项
cmake --preset Debug -DROBOT_TARGET=board_only_dm02
cmake --build --preset Debug

# 或使用独立构建目录，避免反复改缓存
cmake -S . -B build/board_test -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug \
  -DROBOT_TARGET=board_only_dm02
cmake --build build/board_test
```

产物：`build/Debug/CtrlBoard-H7_IMU.elf`。

## 烧录与调试

目标板：STM32H723VGTx（1 MB Flash），调试器：CMSIS-DAP v2 / DAPLink。

```bash
# 烧录并复位
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
  -c "program build/Debug/CtrlBoard-H7_IMU.elf verify reset exit"

# GDB 调试
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg
gdb-multiarch build/Debug/CtrlBoard-H7_IMU.elf
(gdb) target extended-remote :3333
(gdb) monitor reset halt
(gdb) load
```

首次导入前已把板内固件备份到 `backup/original_1MB.bin`（该目录不提交到仓库）。

## 车辆 target 约定

```text
Robot/robot.h                        选择当前 ROBOT_TARGET
Robot/<Role>/<target>.h              身份、App 启动选择、机构构型
Robot/<Role>/<target>_map.h          接线、外设绑定、电机 bus/id、GPIO/TIM/SPI/UART 映射
Robot/<Role>/<target>_param.h        输入映射、控制参数、观测参数、调试和通信策略
```

新增车辆时复制一组 target 文件，完整写清楚“这台车是谁、怎么接、用什么参数”。允许重复宏，优先保证配置事实明确。

当前预留给 2026 哨兵的目录：

```text
Robot/Sentry/       # sentry_2026.h / _map.h / _param.h
Robot/Test/         # 光板与单项验证 target
```

## 启动链路（目标形态）

```text
Communication
  -> Calibration
  -> Observe
  -> Input
  -> Magazine
  -> Gimbal
  -> Shoot
  -> Chassis
  -> Referee
  -> Daemon
  -> Alarm
```

具体启用哪些 App 由 target 决定；未接负载的机构默认关闭。

## 编码约定

- C 文件和头文件是事实来源，`.hpp` 只做零成本便捷包装。
- 不引入虚函数、RTTI、运行时动态分派或不必要的运行时分配。
- 源码 include 只写目标头文件名，不写跨模块路径（由工程 include path 提供）。
- 不保留旧 API 兼容层，迁移时直接修改实现和上层调用。
- 底层模块只表达协议、设备、算法和安全钳位，不绑定具体机器人业务语义。
- 新协议先改 `Component/Communication/Src/communication_host_schema.hpp`，再生成代码，不手改 `Generate/`。

## 板级工程

`Boards/` 由 STM32CubeMX 生成，包含 HAL、CMSIS、FreeRTOS 和 Keil 工程。约定：

- 不直接修改 `.ioc` 以外的生成文件来承载业务逻辑。
- 不使用 CubeMX 生成的 `Core/Src/main.c` 作为最终入口，最终入口是 `Tasks/main.c`。
- CMSIS-DSP 使用 `ThirdParty/CMSIS-DSP` 的版本，不使用 CubeMX 自带旧库。

## USB CDC 桥接

`Component/Communication/Src/communication_usb.c` 实现 Jetson ↔ STM32 的 USB CDC 帧协议，
由 `app_communication` 在 50 Hz 周期内收发；协议与 Jetson 侧 `ros_ws/src/sentry_hardware/usb_bridge` 保持一致。

- 帧格式：`AA 55 | id | seq | len | payload | xor`
- 主机命令：`0x01` 心跳、`0x10` `UnifiedCmd`
- MCU 上报：`0x81` 状态、`0x82` 心跳
- 主机 500 ms 无帧时 MCU 自动清除命令有效位
- USB 枚举为 `0483:5740`（`Printk DM 02`），Jetson 侧按 VID/PID 自动识别

## CAN1 DMH6215 台架测试

`can1_dmh6215_test` 选项只启动 Communication + `app_can_test`：

- FDCAN1，1 Mbps，标准帧
- 扫描 ID 1..8，发送达妙使能、原生速度模式命令
- 收到反馈后，锁定电机 ID，向 `ID + 0x200` 持续发送 float 速度指令
- 速度目标宏：`APP_CAN_TEST_VEL_RAD_S`（默认 3.0）
- 测试会一直运行，直到断电或烧回 `sentry_2026_bringup`
- 诊断量可在 GDB 中读取：`app_can_test_rx_count`、`app_can_test_last_id`、
  `app_can_test_last_data`、`app_can_test_last_tx`、`app_can_test_tec/rec/psr`、
  `app_can_test_phase`、`app_can_test_cmd_vel`

实测结果（2026-10-19，右后 DMH6215 悬空）：

| 项 | 值 |
|---|---|
| 反馈 ID | `0x00`（四轮 ID 规划后改为 `0x51`） |
| 电机 ID | `1`（反馈 `data[0]=0x11`） |
| TEC / REC | `0 / 0` |
| 速度指令帧 | `00 00 40 40 00 00 00 00`（float 3.0） |
| 反馈 | 位置持续变化，电机连续旋转 |
| 结论 | CAN_H/CAN_L 未接反，原生速度模式可连续运行 |

> DMH6215 的 MIT 量程以说明书 7.1.6 为准：`P_MAX=±12.5 rad`、`V_MAX=±45 rad/s`、
> `T_MAX=±10 Nm`（旧代码里的 ±200 rad/s 不适用于该型号）。

### 保守低速测试（`APP_CAN_TEST_CONSERVATIVE=1`）

指定 ID 的首次试转使用 MIT 速度控制：`Kp=0`、`t_ff=0`，力矩上限约为
`Kd * v_des`。速度按斜坡上升/下降，限时后自动零速、零力矩、失能；
同时有反馈超时、位置变化上限和堵转兜底。

```bash
cd stm32_ws/Boards
cmake -S . -B build/can_safe -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DROBOT_TARGET=can1_dmh6215_test \
  -DAPP_CAN_TEST_CONSERVATIVE=1 \
  -DAPP_CAN_TEST_TARGET_ID=3 \
  -DAPP_CAN_TEST_SAFE_VEL_RAD_S=0.5F
cmake --build build/can_safe
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
  -c "program build/can_safe/CtrlBoard-H7_IMU.elf verify reset exit"
```

默认参数：`0.5 rad/s`、`Kd=0.5`（最大力矩约 `0.25 Nm`）、斜坡 `500 ms`、
保持 `1000 ms`、位置变化上限 `1.0 rad`、总超时 `3 s`。
诊断量：`app_can_test_safe_state/reason`、`app_can_test_safe_pos/vel`、
`app_can_test_safe_moved_rad`、`app_can_test_safe_fb_count`。

实测（2026-10-19，CAN1 只挂左后 ID 3，反馈 ID `0x53`）：

| 项 | 值 |
|---|---|
| 状态 / 原因 | `DONE` / `RUN_DONE` |
| 反馈帧数 | 244 |
| TEC / REC | `0 / 0` |
| 起始 / 结束位置 | `0.0006 / 0.3336 rad` |
| 位置变化 | 约 `0.305 rad` |
| 结论 | 正指令使解算位置增大，电机可低速连续转动；结束时已失能 |

> 测试结束后固件停在 `DONE`，但复位/重新上电会重新执行测试。
> 离开台架前烧回 stop-only 或 `sentry_2026_bringup`。

已记录的底盘方向事实：

- 右后轮（CAN1 / ID 1 / 反馈 `0x51`）：正速度指令 `+3.0` 使车身**后退**，所以车身前进时该轮取负。
- 四轮 DMH6215 的统一 ID：右后 `1 / 0x51`、前左 `2 / 0x52`、左后 `3 / 0x53`、前右 `4 / 0x54`。
- 底盘类型已切换为 `ROBOT_CHASSIS_KIND_MECANUM`。
- 完整电机映射与待测轮子见 [`docs/hardware.md`](../docs/hardware.md)。

停止测试（烧回无电机输出的 bring-up 固件）：

```bash
cd stm32_ws/Boards
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
  -c "program build/bringup/CtrlBoard-H7_IMU.elf verify reset exit"
```

## 当前状态

- 已导入 H723 板级基线（HAL / FreeRTOS / USB CDC / CMake / Keil）。
- 已加入 `sentry_2026` 与台架选项 `sentry_2026_bringup`。
- USB CDC 桥接已在台架上跑通（约 60 帧/s 双向，0 CRC 错误）。
- 左后 DMH6215（CAN1 / ID 3 / 反馈 `0x53`）已通过低速保守测试：
  `0.5 rad/s`、`Kd=0.5`、移动约 `0.305 rad`，`TEC/REC=0`，结束自动失能。
- 四轮 DMH6215 的发送/反馈 ID 已统一为 `1/0x51`、`2/0x52`、`3/0x53`、`4/0x54`；
  右后、前左、前右均已单独断电复验保存生效，四台同时上电扫描确认反馈 ID 唯一，
  `motor_mask=0x1E`、`TEC/REC=0`。
- 2026-10-20 四轮悬空 `0.3 rad/s`、20 s 前进测试通过；前进符号为
  RR `-1`、FL `+1`、RL `+1`、FR `-1`。
- DM 电机必须先读/写 `CTRL_MODE`（RID=10）并保存；速度模式的使能/失能
  要发到 `ID + 0x200`，不能发在基础 ID。
- `sentry_2026` 的 map/param 目前继承 `infantry_steer_2025`，实车接线、电机 ID 和 PID 参数需要按哨兵硬件标定。
- 全功能 `sentry_2026` 目前在台架上卡在 `app_observe_init`（IMU/observer 初始化失败）；台架调试请使用 `sentry_2026_bringup`。
- 已确认 HAL tick 优先级必须为 0，否则 FreeRTOS 启动前的 `BASEPRI=0x50` 会屏蔽 TIM6，导致 `HAL_Delay` 卡死。
