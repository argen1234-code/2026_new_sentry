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
| `Tasks` | 工程主入口和 RTOS app 启动顺序；`main.c/.h` 持有真正 `main()`，`Src/task_main.c/.h` 只编排 Apps。 |
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

## 当前状态

目前仅创建目录骨架，尚未导入 CubeMX 工程与外设初始化代码。
