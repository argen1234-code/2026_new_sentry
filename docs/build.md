# 构建与部署

## 1. ROS 2 工作空间

环境：Ubuntu 24.04 + ROS 2 Jazzy（Jetson Orin Nano）。

```bash
cd ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

常用选项：

```bash
# 只编译某个包及其依赖
colcon build --symlink-install --packages-up-to sentry_bringup

# 运行测试
colcon test --packages-select sentry_algorithm
colcon test-result --verbose
```

新增功能包后，把包目录放到 `ros_ws/src/`，包内至少包含 `package.xml` 和 `CMakeLists.txt`（或 `setup.py`）。

语言约定：

| 类型 | 语言 | 示例 |
|---|---|---|
| 实时控制、设备桥、算法核心 | C++17 | `sentry_control`、`sentry_devices` |
| 决策、工具、脚本 | Python 3.12 | `sentry_decision`、`sentry_tools` |
| 消息/服务/动作 | `.msg/.srv/.action` | `sentry_msgs` |

## 2. STM32 固件

目标板：STM32H723VGTx（1 MB Flash）。构建：CMake + Ninja + arm-none-eabi-gcc。
调试/烧录：DAPLink v2（CMSIS-DAP）+ OpenOCD 0.12。

- 最终入口：`stm32_ws/Tasks/main.c`，**不要**把 CubeMX 生成的 `Core/Src/main.c` 当作最终入口。
- `.ioc` 只用于生成板级事实，禁止手改生成文件和 `main()` 初始化顺序。
- Keil 工程仍保留在 `Boards/MDK-ARM/`，但日常构建使用 CMake。

固件选项：

```bash
cd stm32_ws/Boards

# 默认 sentry_2026
cmake --preset Debug
cmake --build --preset Debug

# 其它选项
cmake --preset Debug -DROBOT_TARGET=board_only_dm02
cmake --build --preset Debug
```

可选值：

| target | 用途 |
|---|---|
| `sentry_2026` | 2026 哨兵整车（云台板，当前默认） |
| `sentry_2026_bringup` | 台架启动，仅 Communication + USB 桥 |
| `sentry_2026_chassis` | 底盘 DM6215 原生速度模式验证（同板启用底盘） |
| `can1_dmh6215_test` | CAN1 DMH6215 单轮/四轮台架 |
| `can1_dmh6215_angle_test` | CAN1 单电机角度 PID 台架 |
| `infantry_steer_2025` | 2025 舵轮步兵 |
| `board_only_dm02` | DM02 光板资源测试 |

台架调试（只启动 Communication + USB 桥接）：

```bash
cmake -S . -B build/bringup -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DROBOT_TARGET=sentry_2026_bringup
cmake --build build/bringup
```

烧录：

```bash
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
  -c "program build/Debug/CtrlBoard-H7_IMU.elf verify reset exit"
```

验证顺序建议：

1. `board_only_dm02` 光板测试（Flash / IMU / 蜂鸣器 / USB CDC）。
2. `sentry_2026_bringup` 启动，确认 USB CDC 枚举、`/usb_bridge/state` 有数据。
3. `sentry_2026` 启动，确认 IMU/observer 初始化通过后再接入控制链路。
4. 按数据流逐步验证：CAN 反馈 → 云台电机读写 → 遥控链路 → 发射链路。

USB 桥接验证：

```bash
cd ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch usb_bridge usb_bridge.launch.py

# 另开终端
ros2 topic echo /usb_bridge/state
```

STM32 枚举为 `0483:5740`（`Printk DM 02`），节点自动查找，不依赖 `/dev/ttyACM` 编号。

## 3. 部署到机器人

ROS 侧：

```bash
# 在开发机交叉/本机编译后同步
rsync -av --delete ros_ws/install/ argen@192.168.31.183:~/2026_new_sentry/ros_ws/install/
```

或直接在 Jetson 上编译（当前开发方式）：

```bash
ssh argen@192.168.31.183
cd ~/2026_new_sentry/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

后续建议：

- 使用 `systemd` 用户服务管理 `sentry_bringup`，实现开机自启与崩溃重启。
- 串口设备使用 udev 规则固定名称，避免 `/dev/ttyUSB*` 漂移。
- 日志统一输出到 `~/.ros/log`，关键状态同时上报裁判系统或调试工具。

## 4. 环境备注

- Jetson 与开发机的具体 IP、远程桌面和代理设置属于本机环境事实，不写入仓库；以现场网络为准。
- Jetson 只有 8 GiB 统一内存，ROS 编译必须使用 `ros_ws/scripts/build_lowmem.sh`，不要直接 `colcon build -j6`。
- 串口/USB 设备建议用 udev 规则或 VID:PID 固定，不依赖 `/dev/ttyUSB*` 编号。
