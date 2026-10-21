# ROS 2 工作空间

运行平台：Jetson Orin Nano / Ubuntu 24.04 / ROS 2 Jazzy。

工作空间结构沿用 `2026_sentry_nav`：按职责分层的实车导航工程。

```text
src/
├── sentry_interfaces                      自定义 msg / srv
├── sentry_description                     URDF、mesh、RViz 与 TF
├── sentry_hardware/                       雷达、底盘、裁判系统接入
│   ├── livox_ros_driver2                  Livox MID360 驱动
│   ├── extra_cmd                          底盘 UDP 指令桥
│   ├── referee_serial_bridge              裁判系统串口桥
│   ├── standard_robot_pp_ros2             官方串口协议节点（备用）
│   └── sentry_hardware_bringup            传感器 launch 与 MID360 网络配置
├── sentry_localization/                   Point-LIO、SLAM Toolbox、重定位
├── sentry_perception/                     点云地形分析、点云转激光、扫描生成
├── sentry_navigation/                     Nav2 插件、控制器、行为树与参数
├── sentry_decision/                       比赛决策与底盘指令发布
├── sentry_bringup/                        实车一键启动入口
├── mecanum_car_model/                     当前麦轮小车 CAD 模型（URDF + mesh）
└── robot_base_description/                麦轮小车 xacro / launch / RViz
```

工作空间级资源：

```text
config/
├── common/      通用导航参数
├── robots/      单台车参数
└── arenas/      场地地图、先验点云
scripts/
└── start_sentry.sh
```

## 功能包

| 包 | 职责 |
|---|---|
| `sentry_interfaces` | 自定义消息与服务定义 |
| `sentry_description` | 静态描述：默认当前训练车麦轮模型，`chassis_model:=sentry` 切换旧舵轮+云台模型；含 mesh、RViz 配置 |
| `sentry_hardware` | 雷达与裁判系统 launch、MID360 网络配置 |
| `livox_ros_driver2` | Livox MID360 驱动（含 arm64 SDK） |
| `extra_cmd` | `cmd_vel` 转 UDP 底盘指令 |
| `referee_serial_bridge` | 裁判系统串口数据桥 |
| `usb_bridge` | Jetson ↔ STM32H723 USB CDC 桥接（`/unified_cmd` → MCU，MCU → `/usb_bridge/state`） |
| `point_lio` | LiDAR-IMU 里程计与建图 |
| `small_gicp_relocalization` | 先验点云重定位 |
| `sentry_localization` | 建图 / 定位统一 launch |
| `terrain_analysis`、`terrain_analysis_ext` | 地形与障碍物分析 |
| `pointcloud_to_laserscan` | 点云转激光 |
| `sensor_scan_generation`、`loam_interface` | 坐标变换与里程计接口 |
| `pb2025_nav_bringup` | Nav2 启动、参数、行为树、地图 |
| `pb_nav2_plugins`、`pb_omni_pid_pursuit_controller` | 自定义 Nav2 插件与控制器 |
| `pb_teleop_twist_joy`、`fake_vel_transform` | 遥控与速度变换 |
| `sentry_decision` | 比赛决策节点 |
| `sentry_bringup` | `sentry.launch.py` 统一入口 |
| `mecanum_car_model` | 当前麦轮小车 CAD 模型：URDF、CSV、STL mesh |
| `robot_base_description` | 麦轮小车 xacro、RViz 与 view_robot launch |

## 依赖方向

```text
sentry_bringup
    -> decision / localization / navigation / perception
        -> hardware
            -> interfaces
```

业务包之间只通过 ROS 话题、服务和 action 交互，禁止链接彼此内部实现。

## 启动

### 低内存编译（Jetson 8 GiB）

本机只有 8 GiB 统一内存，`-j6` 编译会在 PCL/Point-LIO 模板实例化时
触发全局 OOM 并卡死系统。统一使用串行、带 cgroup 上限的构建脚本：

```bash
# 首次：安装缺失依赖（需要 sudo 密码）
sudo ./scripts/install_deps.sh

# 强烈建议：先加 swap，避免重载编译单元把桌面拖死
sudo ./scripts/setup_swap.sh 6G

# 串行编译，每包独立 MemoryHigh/MemoryMax，看门狗保底 500 MB
./scripts/build_lowmem.sh

# 断点续编 / 单包编译 / 只查看顺序
RESUME_FROM=point_lio ./scripts/build_lowmem.sh
ONLY=livox_ros_driver2 ./scripts/build_lowmem.sh
DRY_RUN=1 ./scripts/build_lowmem.sh
```

可调参数：`JOBS`、`MEM_FLOOR_MB`、`MIN_START_MB`、`MEM_HIGH_PCT`、
`MEM_MAX_PCT`、`MAX_CAP_MB`、`SWAP_MAX_MB`、`CPU_QUOTA`、`SKIP_PKGS`、
`PKG_MAX_MB_<包名>`。内存采样写入 `log/lowmem/mem_*.csv`，构建日志写入
`log/lowmem/build_*.log`。

编译完成后：

```bash
source install/local_setup.bash
```

实车启动：

```bash
./scripts/start_sentry.sh                    # RViz 显示（默认）
./scripts/start_sentry.sh visualization:=headless
./scripts/start_sentry.sh visualization:=foxglove
```

## MID360 网络

默认配置见
`src/sentry_hardware/sentry_hardware_bringup/config/MID360_config.json`：

| 项 | 值 |
|---|---|
| 雷达 IP | `192.168.1.153` |
| 主机 IP | `192.168.1.50/24` |
| 雷达网卡 | `enP8p1s0` |
| 点云话题 | `/livox/lidar` |
| IMU 话题 | `/livox/imu` |
| 点云 frame | `front_mid360` |

主机网卡配置（NetworkManager，一次即可）：

```bash
nmcli connection modify "MID360" \
  connection.interface-name enP8p1s0 \
  ipv4.method manual ipv4.addresses 192.168.1.50/24 ipv4.never-default yes \
  ipv6.method disabled
nmcli connection up "MID360"
ping -c 3 192.168.1.153
```

当前使用名为 `MID360` 的 NetworkManager profile，绑定 `enP8p1s0`，
静态地址 `192.168.1.50/24`，`never-default` 防止抢占默认路由，IPv6 已关闭。

## 当前状态

已按 `2026_sentry_nav` 导入完整包结构，并将 ROS 发行版从 Humble 适配到 Jazzy。
工作空间 **24 / 24 个包** 已编译安装，`sentry.launch.py` 参数可正常解析。

Jazzy 适配修改：

| 包 | 修改 |
|---|---|
| `livox_ros_driver2` | 显式设置点云/IMU 目的 IP 与端口、启用点云发送、查询内部状态 |
| `pb_nav2_plugins` | `Status` → `ResultStatus`、`Twist` → `TwistStamped`、`CombinationMethod` 转换 |
| `pb_omni_pid_pursuit_controller` | `nav2_core/exceptions.hpp` → `controller_exceptions.hpp` |
| `standard_robot_pp_ros2` | `fromVector()` 中 `T packet{}` 零初始化，消除 `-Werror=maybe-uninitialized` |

MID360 实测状态：

- `enP8p1s0` 载波正常，100 Mb/s，主机 `192.168.1.50`，雷达 `192.168.1.153`
- `/livox/lidar`：`livox_ros_driver2/msg/CustomMsg`，`frame_id = front_mid360`
- `/livox/imu`：`sensor_msgs/msg/Imu`
- `/livox/lidar/pointcloud`：`sensor_msgs/msg/PointCloud2`

低内存构建脚本见 `scripts/build_lowmem.sh`、`scripts/setup_swap.sh`、
`scripts/install_deps.sh`，构建产物与内存采样在 `log/lowmem/`。
