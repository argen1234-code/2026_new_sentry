# 工程差异说明

本文说明当前新工程和 `_ref` 旧工程之间的核心差异。`_ref` 只作为迁移参考来源，不作为当前工程构建依赖。

## 总体变化

旧工程的主要问题是任务层过厚：任务循环里同时读取设备、解析输入、维护全局状态、运行控制器、发送电机命令和输出调试数据。新工程改成明确分层：

```text
Devices -> Component -> Apps -> Robot target
```

- `Devices` 只负责设备接收、发送、协议解析和状态缓存。
- `Component` 只负责可复用业务能力，例如控制器、观测器、输入仲裁、通信协议、调试 probe。
- `Apps` 只做应用层 glue，把设备和组件接起来，并维护任务周期。
- `Tasks` 持有工程自己的 `main()` 和 app 启动编排，不再使用 CubeMX 生成 main 作为最终入口。
- `Robot target` 描述具体车辆身份、硬件映射、App 启动选择和运行参数。

旧工程里的全局状态、弱函数覆盖、任务式入口和硬编码设备绑定不作为新工程接口保留。

## 模块对照

| 旧工程职责 | 新工程落点 |
|---|---|
| INS 姿态解算、速度观测、IMU 温控 | `Component/Observer`、`Apps/app_observe.*`、`Component/Controller/Thermal` |
| 遥控器解析和人工输入 | `Devices/Rc`、`Component/Input`、`Apps/app_input.*` |
| 自瞄误差整形 | `Component/Aim`，由 `Apps/app_input.*` 接入统一输入链路 |
| 云台机构控制 | `Component/Controller/Gimbal`、`Apps/app_gimbal.*` |
| 底盘机构控制 | `Component/Controller/Chassis`、`Apps/app_chassis.*` |
| 发射机构控制 | `Component/Controller/Shoot`、`Apps/app_shoot.*` |
| 弹仓舵机控制 | `Devices/Magazine`、`Apps/app_magazine.*` |
| 裁判系统协议和状态 | `Devices/Referee`、`Apps/app_referee.*` |
| 裁判客户端 UI 图元发送 | `Component/Ui`，由 `Apps/app_referee.*` 驱动 |
| 在线检测和错误告警 | `Apps/app_daemon.*`、`Apps/app_alarm.*` |
| 板间通信、Host 通信、VOFA | `Component/Communication`、`Apps/app_communication.*` |
| Debug probe 和日志 | `Component/Debug`，由 Communication 输出到 VOFA/CDC |
| 校准和持久化 | `Apps/app_calibration.*` |
| 电机协议和发送组 | `Devices/Motor` |

## 配置差异

旧工程使用分散的车型宏、机构宏和任务内默认参数。新工程将具体车辆配置收敛到 `Robot`：

- `Robot/robot.h` 选择当前 target。
- `Robot/<Role>/<target>.h` 描述这台车的身份、App 启动选择和机构构型。
- `Robot/<Role>/<target>_map.h` 描述硬件接线和设备绑定。
- `Robot/<Role>/<target>_param.h` 描述输入映射、控制参数、观测参数、通信和调试策略。

当前示例 target 是 `Robot/Infantry/infantry_steer_2025.h`。同一兵种下不同年份、不同底盘构型或不同电机配置，都应新增独立 target 文件组，而不是复用模糊的兵种宏。

## 行为迁移原则

迁移不是原样复制旧工程，而是保留有价值的业务语义并重新放到合适层级：

- 设备协议进入 `Devices`，不在 App 里手写协议。
- 控制算法进入 `Component/Controller` 或 `Algorithm`，不在任务循环里直接堆控制逻辑。
- 输入语义进入 `Component/Input` 和 `Component/Aim`，不让设备层绑定机器人控制含义。
- 应用任务只负责调度和 glue，不重新持有控制事实。
- 明显错误、重复和备份性质的实现不迁移。
- 意义不明但可能是现场问题妥协的行为，需要在新位置显式命名和注释，不用旧全局接口包起来。

## 已明确不迁内容

以下内容当前不进入新工程运行链路：

- `_ref/ui_back` 完整状态面板。
- OLED 任务和 OLED 设备栈。
- WS2812 灯效 App。
- 多轴机械臂构型。
- 额外 Numerical、FFT、FIR/IIR、EKF、自适应滤波等算法库。
- 旧 USB 任务、空闲任务、slip 相关残留和通用舵机调参任务。

这些内容不是当前 `infantry_steer_2025` 云台板固件的必要功能。后续如果确实需要，应按新分层新增明确模块，而不是恢复旧任务式写法。

## 当前运行链路状态

当前 target 默认启用：

```text
Communication
Calibration
Observe
Input
Magazine
Gimbal
Shoot
Referee
Daemon
Alarm
```

当前 target 默认关闭：

```text
Chassis
BoardLink
```

这表示当前固件是云台板固件。底盘板、板间通信或其它机器人 target 需要独立配置启用。

## 对后来开发者的要求

- 新增车辆时先新增 Robot target，不要把配置写进底层模块。
- 新增机械构型时先扩展对应 Controller 公共接口，再在 `Src` 下新增具体实现。
- 新增输入设备时先在 `Devices` 做原始协议解析，再在 `Component/Input` 增加映射。
- 新增调试输出时优先使用 `Component/Debug` probe 和 `Component/Communication`，不要写 App 专用调试三件套。
- 新增通信协议时优先使用 `Tools/Printk-Protocol` 生成事实代码，不手写互相猜 schema 的协议。
