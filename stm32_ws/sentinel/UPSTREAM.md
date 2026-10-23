# 上游来源与对接说明

本目录以上游固件为基线，唯一的功能性改动是新增旧协议兼容通道
（见下文“本地改动”）。其余文件保持上游原样，改动逐条可追溯。

## 版本信息

| 项 | 值 |
|---|---|
| 仓库 | https://github.com/threfire/Sentinel |
| 分支 | `main` |
| 提交 | `a16bfae5a913781ec734cc5ded2f7492d34cc0f1` |
| 提交信息 | `v1.0.1` |
| 提交时间 | 2026-10-22 14:11:16 UTC |
| 拉取时间 | 2026-10-22 |
| 校验 | 344 个文件逐一与 GitHub tree 的 blob SHA 比对，全部一致 |

上游工程名是 `Hero`，目标芯片同样是 **STM32H723VGTx**，CubeMX 入口为
`CtrlBoard-H7_WS1812.ioc`。它包含云台、底盘、发射、自瞄通信、裁判系统、
功率控制、W25Q64 等功能，板级资源（FDCAN1/2/3、USART1/5/7/10、USB CDC
HS、OCTOSPI2、TIM24）与 `legacy/` 一致。

## 构建

上游使用 Keil MDK-ARM，不是 CMake：

```text
MDK-ARM/CtrlBoard-H7_WS1812.uvprojx
```

用 μVision 打开即可（上游记录：μVision V5.40 / ARMCC V5.06）。本仓库的
`legacy/` 继续使用 CMake + Ninja，两者互不影响。

## 与 legacy/ 的关键差异

| 维度 | `legacy/` | `sentinel/`（上游） |
|---|---|---|
| 应用目标 | 哨兵 | 英雄（Hero） |
| 分层 | `Tasks/Robot/Apps/Component/Devices/Bsp` | `Core/User/APP/APP_Support/Communication` |
| 构建 | CMake + Ninja | Keil `.uvprojx` |
| 上位机协议 | 固定帧 `AA 55 \| id \| seq \| len \| payload \| xor` | uproto：`AA 55` + 16 B 头 + CRC + channel/SID |
| USB 标识 | `0483:5740`，产品名 `Printk DM 02` | `0483:5740`，产品名 `STM32 Virtual ComPort` |
| 底盘命令 | `UnifiedCmd`（45 B，`0x10`） | `GIMBAL_SID_CHASSIS`（`0x0206`），`gimbal_chassis_cmd_t` |
| 底盘反馈 | `0x83 WheelState`（58 B） | 无专用底盘状态通道 |
| 任务入口 | `Tasks/main.c` | CubeMX `Core/Src/main.c` + `freertos.c` |

## 能否与现有 ROS 链路融合

**源码层不合并**。两套工程的调度、外设初始化和协议边界完全不同，混合
include 会同时破坏两边的任务时序和帧格式。

需要在 ROS 上驱动这套固件时，走**协议适配**，有两条路：

1. **MCU 侧兼容通道（已实现）**
   在 `User/Communication/legacy/` 新增兼容模块，把现有 `UnifiedCmd`
   （45 B）解析后映射到上游 `gimbal_chassis_cmd_t`，并周期回发
   `0x81/0x82/0x83/0x84` 状态。ROS 侧 `usb_bridge` 不需要任何改动。
   细节见下一节。
2. **Jetson 侧新桥（未做）**
   照 `User/Communication/example/host/cpp/` 的参考实现，写一个 ROS 2
   节点直接说 uproto，替换 `usb_bridge`。固件保持上游原样，升级方便。

两条路都必须注意：上游 USB 产品名是 `STM32 Virtual ComPort`，与
`legacy/` 的 `Printk DM 02` 不同，但 VID:PID 都是 `0483:5740`。
`usb_bridge` 按 VID:PID 查找端口，两套固件不会同时在线；插错固件时
节点仍会打开端口，但收不到合法状态帧，诊断会显示链路超时。

## 本地改动：旧协议兼容通道

新增目录 `User/Communication/legacy/`：

| 文件 | 作用 |
|---|---|
| `legacy_proto.h/.c` | 纯 C 协议层：`AA 55` 帧解析/组帧、`UnifiedCmd` 解码、模式映射。不依赖 HAL，可主机测试 |
| `legacy_bridge.h/.c` | 胶水层：接收分流、命令缓存与 100 ms 新鲜度、状态帧组包 |

对上游文件的改动（仅 4 处）：

| 文件 | 改动 |
|---|---|
| `example/device/usb_cdc_port.c` | RX 先过 `legacy_bridge_feed_rx()`，剩余字节交给 uproto；旧协议主机在线时暂停 uproto 发送；发送统一走环形缓冲 |
| `example/device/comm_app.c` | 初始化兼容通道；循环里调用 `legacy_bridge_tick()`；底盘注入抽成 `comm_apply_chassis_cmd()`，旧协议命令不受自瞄开关限制 |
| `USB_DEVICE/App/usbd_cdc_if.c/.h` | 原本未被引用的 `proto_port_write()` 改为导出的 `usb_cdc_write_queued()` |
| `MDK-ARM/CtrlBoard-H7_WS1812.uvprojx` | 加入两个新源文件和 `../User/Communication/legacy` 头文件路径 |

### 下行（Jetson → MCU）

- 解析 `0x01` 心跳（len 2）与 `0x10 UnifiedCmd`（len 45）；
- 命令必须同时带 `CMD_VALID` 和 `CMD_MOTION_ENABLE`，否则按安全停车处理
  （速度清零、模式 no-move）；超过 **100 ms** 未更新同样失效；
- 模式映射：

  | 旧模式 | 上游模式（RC 开关） |
  |---|---|
  | 0 Stop | 3 no-move |
  | 2 NoFollow | 0 no-follow yaw |
  | 3 FollowGimbalYaw | 1 follow gimbal yaw |
  | 5 Spin | 2 spin |
  | 其他 | 3 no-move |

- 沿用上游的遥控注入链路（`CHASSIS_VX_RC_SEN` 等），方向盘符号与
  旧固件的一致性需要在实车上确认。

### 上行（MCU → Jetson）

| 帧 | 周期 | 数据来源 |
|---|---|---|
| `0x81` MCU 状态 | 20 ms | `chassis_move`（vx/vy/wz、模式）+ HWT906 姿态 + 云台电机反馈 |
| `0x82` 心跳 | 100 ms | `chassis_move` 模式 |
| `0x83` 四轮状态 | 20 ms | `CHASSIS_MOTOR_MEASURE`；速度按 `MPS_to_RPM` 换算，位置为电机侧弧度，力矩由电流命令估算 |
| `0x84` IMU 状态 | 20 ms | HWT906 陀螺/加速度/温度，四元数由欧拉角合成 |

注意：

- 四轮数组顺序沿用上游 `CHASSIS_MOTOR_MEASURE`（205 后 / 206 右 /
  207 前 / 208 左），与旧 ROS 注释里的 `RR, FL, RL, FR` 不同。
  2026 模型 URDF 不含车轮，`/chassis/state` 的逐轮数组只影响备用
  麦轮模型的显示，不影响导航。
- IMU 轴向与零位尚未实车标定；`/imu/data` 目前只作诊断用。

### 共存行为

- 接收方向两种协议可以同时解析：只有 `id=0x01/len=2` 和
  `id=0x10/len=45` 且 xor 正确的帧会被兼容通道吞掉，其余字节原样
  交给 uproto。
- 发送方向互斥：最近 500 ms 内收到过合法旧协议帧时，暂停 uproto 发送；
  主机离线后自动恢复。没有旧协议主机时，上游行为不变。

### 验证状态

- `tools/host_tests/legacy_proto_test.c` 覆盖参考帧（由 ROS
  `protocol.py` 生成）、任意分块解析、坏校验、uprotu 透传、模式映射，
  在 PC 上全部通过。
- 尚未用 Keil 编译、未上实车。烧录前至少确认：
  1. `usb_bridge` 能收到 `0x81/0x83/0x84` 且序列号连续；
  2. `enable_motion:=false` 时四轮不转；
  3. 低速前进/左移/自转方向与 `/cmd_vel` 一致。

## 升级上游代码

```bash
# 记录当前版本
git -C /path/to/Sentinel rev-parse HEAD

# 对比后用新提交的 tarball 覆盖本目录（保留本文件）
# 并更新上表的提交号、日期和文件数
```
