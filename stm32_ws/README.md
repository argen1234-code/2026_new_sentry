# STM32 固件工作空间

同一块 STM32H723VGTx 控制板上并行维护**两套互相独立的固件工程**。两套
工程各有自己的入口、构建系统和通信协议，不共享源码，也不互相 include：

| 目录 | 固件 | 构建 | 通信 | 状态 |
|---|---|---|---|---|
| [`legacy/`](legacy/README.md) | 仓库原有 2026 哨兵固件 | CMake + Ninja + arm-none-eabi | USB CDC `UnifiedCmd`（`AA 55` 帧） | 与当前 ROS `usb_bridge` 配套 |
| [`sentinel/`](sentinel/README.md) | [threfire/Sentinel](https://github.com/threfire/Sentinel) 最新代码 | Keil MDK-ARM（μVision） | USB CDC + uproto / channel，另带旧协议兼容通道 | 可被现有 `usb_bridge` 驱动 |
| [`chassis_fw/`](chassis_fw/README.md) | 从 `sentinel/` 抽出的底盘驱动 + USB CDC | CMake + Ninja + arm-none-eabi | USB CDC 旧协议 `UnifiedCmd`（`AA 55` 帧） | 全向轮速度直驱最小固件 |
| `old_stm32_code_reference/` | 更早的 DM-balance 参考代码 | — | — | 只作对照，不参与构建 |

## 为什么解耦而不是合并

两套代码的差异在架构层，不在个别驱动：

| 维度 | `legacy/` | `sentinel/` |
|---|---|---|
| 分层 | `Tasks/Robot/Apps/Component/Devices/Bsp` | `Core/User/APP/APP_Support/Communication` |
| 构建 | CMake + Ninja，`ROBOT_TARGET` 编译期选车 | Keil `.uvprojx`，单一 Hero 工程 |
| 上位机协议 | 固定长度 `UnifiedCmd`（45 B）+ 状态帧 | uproto 可变长 + channel/mailbox |
| 板级入口 | `Tasks/main.c` + `Boards/Core` | CubeMX `Core/Src/main.c` + `CtrlBoard-H7_WS1812.ioc` |
| 车辆 | 哨兵（云台板 / 底盘 DMH6215） | 英雄（云台 + 发射 + 功率控制） |

直接合并会同时破坏两边的任务调度和协议边界。正确做法是保持两个工程
可独立编译、独立烧录，需要互通时在**协议层**做适配，而不是在源码层交叉引用。
`sentinel/` 里的
[`User/Communication/legacy`](sentinel/User/Communication/legacy/)
就是这样一个兼容通道：它把旧协议的 `UnifiedCmd` 翻译成上游的
`gimbal_chassis_cmd_t`，并按旧格式回发 `0x81/0x82/0x83/0x84` 状态帧，
ROS 侧 `usb_bridge` 无需改动。

`chassis_fw/` 是第三种用法：只把 `sentinel/` 的全向轮底盘驱动和 USB CDC
外设抽出来，配 GCC 构建层，做**速度直驱**的最小可烧录固件。它不复制
`sentinel/Core`，构建时直接引用，因此外设配置仍以那份 `.ioc` 输出为准；
云台、IMU、遥控器、功率计由 `App/stubs/` 提供最小桩。

## 当前约定

- ROS 侧默认仍然对接 `legacy/` 的 USB 协议；`sentinel/` 通过兼容通道
  可以直接使用同一套 `usb_bridge`。烧录 `sentinel/` 后先用
  `enable_motion:=false` 验证状态回传，再打开运动。
- 兼容通道只在检测到旧协议主机时接管发送；没有主机时 uproto 行为不变。
  本地改动清单见 [`sentinel/UPSTREAM.md`](sentinel/UPSTREAM.md)。
- 两边都面向 STM32H723VGTx + DAPLink，但外设初始化以各自 `.ioc` 为准，
  不要把一边的 `Core/` 拷到另一边。
- `legacy/` 的构建说明见 [`legacy/README.md`](legacy/README.md) 和
  [`docs/build.md`](../docs/build.md)；`sentinel/` 用 Keil 打开
  `sentinel/MDK-ARM/` 下的工程。

## 目录变更记录（2026-10-22）

- 原固件从 `stm32_ws/{Algorithm,Apps,Boards,...}` 整体移入 `stm32_ws/legacy/`，
  文件内容未改动，仅路径变化。
- 新增 `stm32_ws/sentinel/`，对应上游 `main` 分支某一提交，版本记录见
  [`sentinel/UPSTREAM.md`](sentinel/UPSTREAM.md)。
