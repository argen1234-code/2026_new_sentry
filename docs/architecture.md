# 系统架构

## 1. 设计目标

1. **实时与非实时分离**：STM32 负责硬实时控制与安全钳位，Jetson 负责计算密集的感知、定位和决策。
2. **依赖单向**：上层可以依赖下层，下层不得依赖上层；公共模块不得知道具体车辆。
3. **单一事实来源**：算法只有一份，配置只有一份，协议只有一份。
4. **增量验证**：按数据流逐段打通，不一次性替换整条链路。
5. **车辆差异配置化**：同一兵种不同底盘、不同电机、不同年份使用独立 target。

## 2. 两端职责

| 侧 | 运行平台 | 职责 | 不负责 |
|---|---|---|---|
| ROS 侧 | Jetson Orin Nano / Ubuntu 22.04 / ROS 2 Humble | 目标检测、自瞄解算、SLAM/定位、路径规划、行为决策、可视化与日志 | 电机换向、电流环、安全钳位 |
| STM32 侧 | STM32H723 / FreeRTOS | 电机与传感器 I/O、姿态解算、机构控制、裁判系统、通信链路、失效保护 | 视觉识别、全局路径规划、复杂决策 |

## 3. STM32 侧分层

```text
Tasks           工程入口、RTOS 启动顺序
  |
Robot           target 选择与单台车配置（*.h / *_map.h / *_param.h）
  |
Apps            应用层 glue：读设备快照、组装输入、调度组件、执行输出
  |
Component       可复用业务能力：Controller / Input / Observer / Aim / Communication / Debug / Ui
  |
Devices         设备驱动与协议解析：电机、IMU、遥控器、裁判系统、Flash、舵机
  |
Bsp             板级外设句柄与 HAL 薄封装：FDCAN、UART、SPI、TIM、ADC、USB
  |
Boards          CubeMX / HAL / Keil 生成工程与板级启动文件
```

`Algorithm` 为纯算法库（PID、Kalman、Mahony、弹道、数学工具），可被 Bsp 以上的任意层调用，但不反向依赖业务层。

> 上述分层描述的是 `stm32_ws/legacy/`（原固件）。`stm32_ws/sentinel/` 是独立引入的
> 上游工程（`Core/User/APP/APP_Support/Communication` 分层 + Keil 构建），
> 不遵循本节的层次约定，详见 [`stm32_ws/README.md`](../stm32_ws/README.md)。

### 依赖规则

- `Devices` 只输出原始协议与状态缓存，不出现“云台”“发射”等业务命名。
- `Component/Controller` 只接收语义化指令和状态快照，不直接访问 HAL。
- `Apps` 负责跨组件编排，不重写协议解析和控制算法。
- `Robot` 只放配置事实，不放实现。
- `Tasks` 只持有 `main()` 和启动编排。

## 4. ROS 侧分层

```text
sentry_bringup          launch / 系统组合 / 参数装载
      |
sentry_decision         行为状态机、战术决策、模式切换
      |
sentry_perception       点云地形分析、点云转激光、扫描生成
sentry_localization     Point-LIO、重定位、坐标变换
sentry_navigation       Nav2 插件、控制器、行为树、代价地图
      |
sentry_hardware         雷达、串口桥、USB 桥、底盘 UDP 指令
      |
sentry_interfaces       msg / srv / action 接口定义
```

`sentry_description` 存放 URDF/xacro/mesh；`ros_ws/config/` 存放通用导航参数、单车参数和场地地图。

### 依赖规则

- `sentry_interfaces` 不依赖其它业务包。
- 业务包之间通过 topic/service/action 交互，不直接链接彼此的内部实现。
- 具体车辆参数从 `ros_ws/config/robots/*.yaml` 注入。
- Jetson ↔ STM32 的第一条实车链路是 `sentry_hardware/usb_bridge` 的 USB CDC 桥。

## 5. 跨端数据流

### 上行（STM32 → Jetson）

```text
Imu / Referee / Motor
  -> app_observe / app_referee
  -> communication_host
  -> USB CDC
  -> sentry_hardware/usb_bridge
  -> sentry_interfaces 状态消息
  -> localization / decision / navigation
```

### 下行（Jetson → STM32）

```text
sentry_decision / sentry_navigation
  -> sentry_interfaces 指令消息
  -> sentry_hardware/usb_bridge
  -> USB CDC
  -> communication_board
  -> Components/Input + Controller
  -> Devices/Motor
```

链路超时、CRC 错误、模式非法时，STM32 侧必须进入安全状态，不等待 Jetson 恢复。

## 6. 车辆 target 约定

STM32 侧：

```text
Robot/<Role>/<role_chassis_year>.h         身份、App 启动选择、机构构型
Robot/<Role>/<role_chassis_year>_map.h     接线、外设绑定、电机 bus/id
Robot/<Role>/<role_chassis_year>_param.h   输入映射、控制/观测参数、通信策略
Robot/robot.h                              只负责选择当前 ROBOT_TARGET
```

ROS 侧：

```text
ros_ws/config/robots/<role_chassis_year>.yaml
ros_ws/config/common/*.yaml
ros_ws/src/sentry_bringup/launch/sentry.launch.py
```

新增车辆时复制一组文件并完整写清楚事实，不做隐式继承或模糊分类。

## 7. 字段所有权

| 数据 | 唯一来源 | 说明 |
|---|---|---|
| 电机编码器/电流 | STM32 `Devices/Motor` | Jetson 只消费状态 |
| IMU 姿态 | STM32 `Component/Observer` | Jetson 可用于融合，但不回写 |
| 裁判系统状态 | STM32 `Devices/Referee` | Jetson 只订阅 |
| 目标位姿 | ROS `sentry_perception` | STM32 不维护视觉状态 |
| 路径/行为模式 | ROS `sentry_decision` | STM32 只执行指令并做安全检查 |
| 协议 schema | STM32 `communication_host_schema.hpp` | ROS `sentry_msgs` 与之保持同步 |
