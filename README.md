# 2026 New Sentry

RoboMaster 2026 哨兵机器人工程。工程分层沿用 `2026_old_sentry` 的设计思想：

> 底层只描述能力，组件只描述可复用的业务，应用层只做编排，车辆 target 描述“这一台车是谁、怎么接、用什么参数”。

## 工程组成

| 目录 | 职责 |
|---|---|
| `ros_ws/` | Jetson Orin Nano 上的 ROS 2 Jazzy 工作空间：感知、定位、导航、决策 |
| `stm32_ws/` | STM32H723 控制板固件：云台/底盘/发射控制、裁判系统、传感器、通信 |
| `docs/` | 跨端文档：架构、通信协议、构建与部署 |
| `tools/` | 主机侧辅助脚本（协议生成、调试、部署） |

## 分层映射

| old_sentry 层 | ROS 侧 | STM32 侧 |
|---|---|---|
| `Algorithm` | `sentry_algorithm` | `Algorithm` |
| `Bsp` + `Devices` | `sentry_devices` | `Bsp` + `Devices` |
| `Component/Controller` | `sentry_control` | `Component/Controller` |
| `Component/Input`、`Aim`、`Observer` | `sentry_perception` | `Component/Input`、`Aim`、`Observer` |
| `Component/Communication` | `sentry_msgs` + `sentry_devices`（串口桥） | `Component/Communication` |
| `Component/Debug`、`Ui` | `sentry_tools` | `Component/Debug`、`Ui` |
| `Apps` | `sentry_decision`（行为编排） | `Apps` |
| `Robot` | `sentry_robot`（YAML 配置） | `Robot` |
| `Tasks` | `sentry_bringup`（launch 编排） | `Tasks` |
| `Tools` | `sentry_tools` | `Tools` |

## 主数据流

```text
相机 / 雷达
    -> sentry_perception
    -> sentry_localization / sentry_navigation
    -> sentry_decision
    -> sentry_control
    -> sentry_devices (serial bridge)
    -> STM32: Component/Communication
    -> Apps (Input / Gimbal / Shoot / Chassis)
    -> Devices/Motor
```

STM32 状态回传：

```text
Devices/Imu + Devices/Referee + Devices/Motor
    -> Apps/app_observe, app_referee
    -> Component/Communication
    -> sentry_devices (serial bridge)
    -> sentry_localization / sentry_decision
```

## 快速开始

ROS 2 工作空间：

```bash
cd ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

STM32 固件：见 [`stm32_ws/README.md`](stm32_ws/README.md) 和 [`docs/build.md`](docs/build.md)。

## 文档索引

- [`docs/architecture.md`](docs/architecture.md)：分层、依赖方向、字段所有权
- [`docs/protocol.md`](docs/protocol.md)：Jetson ↔ STM32 通信协议草案
- [`docs/build.md`](docs/build.md)：构建、烧录、部署
- [`docs/hardware.md`](docs/hardware.md)：底盘、电机 CAN 映射与方向约定
- [`ros_ws/README.md`](ros_ws/README.md)：ROS 功能包职责
- [`stm32_ws/README.md`](stm32_ws/README.md)：固件分层与 target 约定

## 开发原则

- C 源文件和头文件是事实来源，`.hpp` 只做零成本包装。
- 不引入虚函数、RTTI、运行时动态分派或不必要的运行时分配。
- 依赖方向只能从上层指向下层，禁止反向依赖。
- 设备层不绑定机器人业务语义，App 层不重新持有协议或控制事实。
- 车辆差异收敛到 target 配置，不写进公共模块。
- 新协议先改 schema，再重新生成代码，不手改生成产物。
