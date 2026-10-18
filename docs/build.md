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

沿用 old_sentry 的构建方式：CubeMX 生成板级工程，Keil MDK 编译。

- Keil 工程：`stm32_ws/Boards/MDK-ARM/`（待导入）
- 最终入口：`stm32_ws/Tasks/main.c`，**不要**把 CubeMX 生成的 `Core/Src/main.c` 当作最终入口。
- `.ioc` 只用于生成板级事实，禁止手改生成文件和 `main()` 初始化顺序。

编译流程（导入工程后）：

1. 打开 `Boards/MDK-ARM/*.uvprojx`。
2. 选择 target 和调试器（DAP-Link / ST-Link）。
3. Build，确认无 warning 后烧录。
4. 先验证板上资源（Flash、IMU、蜂鸣器、USB CDC），再验证电机与通信链路。

可选 CMake 构建（待定）：`stm32_ws/Boards/cmake/` 预留。

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

- Jetson 当前 Wi‑Fi 地址：`192.168.31.183`（建议在路由器做 DHCP 保留）。
- 远程桌面：RustDesk 1.5.0，IP 直连端口 `21118`，服务已设置开机自启。
- 本机 `/etc/hosts` 存在 Steam++ 的 GitHub 重定向规则，访问 GitHub 时需注意。
