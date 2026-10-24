# 2026 New Sentry

RoboMaster 2026 哨兵机器人工程。工程分层沿用 `2026_old_sentry` 的设计思想：

> 底层只描述能力，组件只描述可复用的业务，应用层只做编排，车辆 target 描述“这一台车是谁、怎么接、用什么参数”。

## 工程组成

| 目录 | 职责 |
|---|---|
| `ros_ws/` | Jetson Orin Nano 上的 ROS 2 Humble 工作空间：感知、定位、导航、决策 |
| `stm32_ws/` | STM32H723 控制板固件：`legacy/` 原固件（CMake）与 `sentinel/` 上游新固件（Keil），两套独立工程 |
| `docs/` | 跨端文档：架构、通信协议、构建与部署 |
| `tools/` | 主机侧辅助脚本（协议生成、调试、部署） |

## 分层映射

| 职责 | ROS 侧 | STM32 侧（`stm32_ws/legacy/`） |
|---|---|---|
| 纯算法 | `sentry_decision` / `sentry_navigation` 内部算法 | `Algorithm` |
| 设备与驱动 | `sentry_hardware`（雷达、串口、USB 桥） | `Bsp` + `Devices` |
| 控制 | `sentry_navigation`（Nav2 控制器） | `Component/Controller` |
| 感知与输入 | `sentry_perception` | `Component/Input`、`Aim`、`Observer` |
| 通信 | `sentry_hardware/usb_bridge` + `sentry_interfaces` | `Component/Communication` |
| 编排与启动 | `sentry_bringup`、`sentry_decision` | `Apps`、`Tasks` |
| 配置 | `ros_ws/config/` | `Robot/` |

## 主数据流

```text
相机 / MID360
    -> sentry_perception
    -> sentry_localization / sentry_navigation
    -> sentry_decision
    -> sentry_hardware/usb_bridge
    -> STM32: Component/Communication
    -> Apps (Input / Gimbal / Shoot / Chassis)
    -> Devices/Motor
```

STM32 状态回传：

```text
Devices/Imu + Devices/Referee + Devices/Motor
    -> Apps/app_observe, app_referee
    -> Component/Communication
    -> sentry_hardware/usb_bridge
    -> sentry_localization / sentry_decision
```

## 快速开始

ROS 2 工作空间（Jetson 8 GiB 必须用低内存脚本，避免 OOM 卡死）：

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
./scripts/build_lowmem.sh
source install/local_setup.bash
```

STM32 固件：见 [`stm32_ws/README.md`](stm32_ws/README.md) 和 [`docs/build.md`](docs/build.md)。

## 文档索引

- [`docs/architecture.md`](docs/architecture.md)：分层、依赖方向、字段所有权
- [`docs/protocol.md`](docs/protocol.md)：Jetson ↔ STM32 通信协议草案
- [`docs/build.md`](docs/build.md)：构建、烧录、部署
- [`docs/hardware.md`](docs/hardware.md)：底盘、电机 CAN 映射、控制模式与方向约定
- [`docs/chassis_speed_reference.md`](docs/chassis_speed_reference.md)：推荐速度、死区、顺时针/逆时针与 `wz` 符号
- [`docs/localization_roadmap.md`](docs/localization_roadmap.md)：定位与建图规划（位姿源、GICP、2D 栅格职责、未来里程计接入）
- [`docs/chassis_calibration.md`](docs/chassis_calibration.md)：地面与遮挡区标定（换底盘必做；URDF 的高度与姿态不可信，须实测）
- [`docs/pitfalls.md`](docs/pitfalls.md)：踩坑记录（转向符号、速度环死区、IMU QoS）
- [`docs/boot-fix.md`](docs/boot-fix.md)：MID360 插入时 Jetson 开机卡住的修复
- [`ros_ws/README.md`](ros_ws/README.md)：ROS 功能包职责、低内存构建、MID360 网络
- [`stm32_ws/README.md`](stm32_ws/README.md)：两套固件工程的分工与解耦约定
- [`stm32_ws/legacy/README.md`](stm32_ws/legacy/README.md)：原固件分层、target 约定与台架流程

## 开发原则

- C 源文件和头文件是事实来源，`.hpp` 只做零成本包装。
- 不引入虚函数、RTTI、运行时动态分派或不必要的运行时分配。
- 依赖方向只能从上层指向下层，禁止反向依赖。
- 设备层不绑定机器人业务语义，App 层不重新持有协议或控制事实。
- 车辆差异收敛到 target 配置，不写进公共模块。
- 新协议先改 schema，再重新生成代码，不手改生成产物。
