# 实车参数速查

> 只列**需要人为调整**、且在本工程实车调试中实际改过的参数；完整默认值见各自配置文件。
> 底盘速度与方向结论不在本文重复，见 [`chassis_speed_reference.md`](chassis_speed_reference.md)；
> 每个参数为什么这么定，见 [`pitfalls.md`](pitfalls.md)。

## 1. 底盘链路

### 1.1 `/cmd_vel` → `/unified_cmd`（ROS 侧）

文件：`ros_ws/src/sentry_hardware/usb_bridge/usb_bridge/cmd_vel_to_unified.py`

| 参数 | 当前值 | 说明 |
|---|---|---|
| `max_vx` / `max_vy` / `max_wz` | `0.45` / `0.30` / `1.0` | 与底盘有效速度一致；超限直接钳位 |
| `frame_yaw` | `-0.7853981634`（−45°） | 轮系相对 `base_footprint` 的平移偏角；**换底盘或改雷达安装后必须重测** |
| `sign_wz` | `1.0` | 自转符号；固件已单独处理 wz 取负，这里不要再翻 |
| `publish_rate` | `50.0` Hz | 稳态重发，避免上游 `/cmd_vel` 抖动导致底盘断续 |
| `command_timeout` | `0.5` s | 超过即发零速 |
| `enable_motion` | `false` | 安全门控；`false` 时只发 `mode=STOP,status=0` |

### 1.2 USB 桥（`usb_bridge.yaml`）

| 参数 | 当前值 | 说明 |
|---|---|---|
| `port` | `auto` | 按 `0483:5740` 自动找 STM32 CDC |
| `baud` | `115200` | CDC 实际不限速，保持一致即可 |
| `command_timeout` | `0.25` s | 桥接层判断主机断流 |
| `link_timeout` | `1.0` s | 上行断流告警阈值 |
| `wheel_radius` | `0.075` m | 必须与固件 `MPS_to_RPM` 一致 |
| `wheel_position_wrap_rad` | `0.3272` | 输出轴单圈角度，`2π/19.2032` |
| `wheel_joint_signs` | `[-1, 1, 1, -1]` | 轮序 `[后 205, 右 206, 前 207, 左 208]` |

### 1.3 固件侧

`stm32_ws/legacy/.../host_cmd.h`：`HOST_CHASSIS_WZ_SIGN = 1.0`。
固件只有 wz 一个符号修正点；平移 45° 已在 ROS 侧 `frame_yaw` 补偿。

## 2. 感知

> 同一组节点有两份参数：`sentry_perception_bringup/config/perception.yaml`
> （`perception.launch.py` 使用）和 `nav2_params.yaml` 里的同名块（`navigation_launch.py` 使用）。
> 两处 `terrain_analysis*` 的值目前一致；**`pointcloud_to_laserscan` 只有 `nav2_params.yaml`
> 里的那份生效**（节点由 `sentry_localization/mapping.launch.py` 启动，加载 nav2 参数）。

| 节点 | 参数 | 当前值 | 说明 |
|---|---|---|---|
| `terrain_analysis_ext` | `scanVoxelSize` | `0.1` | 远处地形点云体素，别调太小否则点太密 |
| | `lowerBoundZ` / `upperBoundZ` | `-0.6` / `0.8` | 地形分析高度带 |
| | `terrainUnderVehicle` | `-0.12` | 地面相对 `vehicleZ` 的实测高度；**与 URDF `lidar_z` 无关**，改 `lidar_z` 不需要动它 |
| | `ceilingFilteringThre` | `2.0` | 过滤天花板 |
| | `localTerrainMapRadius` | `6.0` | 只输出车周 6 m；原来是 4.0，4 m 处有一圈幻影障碍，加到 6.0 后地面误报 0.43% → 0.09% |
| | `decayTime` | `0.2` s | 动态障碍清除时间，对齐母工程 |
| `pointcloud_to_laserscan` | `min_height` / `max_height` | `0.16` / `0.50` | 高度带已随 `lidar_z` 抬升同步平移（地面在 `base_footprint` 高 +0.078 m），上限再按 Humble 收紧，见 [`chassis_calibration.md`](chassis_calibration.md) §4.1 |
| | `min_intensity` | `0.20`（`nav2_params.yaml`） | terrain 的 intensity 是高度差，低于此值不进 2D 图 |
| | `range_min` / `range_max` | `0.5` / `10.0` | 近处盲区与最远距离 |

## 3. 定位

### 3.1 Point-LIO

**两套参数并存，按入口区分：**

| 入口 | 参数来源 | `filter_size_surf` | `point_filter_num` | `scan_line` | `tf_send_en` | `pcd_save_en` |
|---|---|---|---|---|---|---|
| 建图 `mapping.launch.py` | `nav2_params.yaml` 的 `point_lio` 块 | `0.1` | `8` | `16` | `False` | `True` |
| 定位 `sentry.launch.py` | `sentry_localization/config/point_lio_mid360.yaml` | `0.5` | `4` | `4` | `True` | 由 launch 覆盖为 `False` |

| 参数 | 当前值 | 说明 |
|---|---|---|
| `blind` | `0.5` m | 雷达近处盲区 |
| `pcd_save.interval` | `-1` | **退出时才写盘**，进程被 kill 会丢 PCD |
| `pcd_save.pcd_save_en` | 建图 `true` / 导航 `false` | 导航时打开会一直攒全分辨率点云，退出还会覆盖 `PCD/scans.pcd` |
| `prior_pcd.enable` | `false` | 先验 PCD 由 GICP 用，不让 LIO 自己加载 |

> 建图用 `scan_line: 16`（MID360 实际 4 线被拆成 16 段），定位用 `4`；
> 改其中一处时注意另一处不会同步。

### 3.2 GICP（`small_gicp.yaml`）

| 参数 | 当前值 | 说明 |
|---|---|---|
| `num_threads` | `4` | Jetson 上别占满 |
| `global_leaf_size` | `0.1` m | 先验点云降采样 |
| `registered_leaf_size` | `0.05` m | 实时点云降采样 |
| `max_dist_sq` | `3.0` | 对应距离门限约 1.73 m |
| `map_frame` / `odom_frame` | `map` / `odom` | `map→odom` 由本节点发布 |
| `base_frame` / `lidar_frame` | `base_footprint` / `front_mid360` | 先验 PCD 会乘该外参，比对残差时必须用同一变换 |
| `init_pose` | `[0,0,0, 0,0,0]` | 起始位姿假设**车就在地图原点**；停在别处启动会先给一个错误的 `map→odom`，随后靠点云收敛 |

定位模式下没有 `slam_toolbox`，因此 **`/obstacle_scan` 没有任何订阅者**，
RViz 里该显示项为空是正常现象，不影响 GICP 与 Nav2。

## 4. 导航（`pb2025_nav_bringup/config/reality/nav2_params.yaml`）

### 4.1 代价地图

| 参数 | 当前值 | 说明 |
|---|---|---|
| `robot_radius` | `0.26` m | 实测车体半径（车边缘到圆心） |
| `intensity_voxel_layer.min_obstacle_intensity` | `0.1` | **低于此值的点直接丢弃**；`0.0` 会把地面杂点收进来糊死代价地图 |
| `max_obstacle_intensity` | `2.0` | terrain 的 intensity 是高度差，超过视为无效 |
| `min_obstacle_height` / `max_obstacle_height` | `0.15` / `0.50` | 障碍高度带；与 `pointcloud_to_laserscan` 同一组值，两头都把车体自身挡住 |
| `use_footprint_filter` / `footprint_padding` | `true` / `0.09` m | 按足迹剔除近场自身点，`0.26 + 0.09 = 0.35` 等价原 `clearingDis` |
| local 来源 | `terrain_map`，5.0 m | 随母工程 |
| global 来源 | `terrain_map_ext`，10.0 m | 随母工程 |
| `obstacle_min_range` | `0.2` m | 车体附近的点不可靠 |
| `inflation_layer` | `cost_scaling_factor: 3.0`，`inflation_radius: 1.04` | `1.04 = 0.26` 内切圈 `+ 3×0.26` 膨胀带，让 Theta* 在 2.3–4.3 m 走廊里有居中倾向；母工程的 0.7/4.0 在这里相当于没有梯度，见 §4.3 |
| 全局 `rolling_window` | `30 × 30` m | 地图较薄时覆盖大范围；母工程用整张静态图 |

### 4.2 规划与跟踪

| 参数 | 当前值 | 说明 |
|---|---|---|
| `GridBased.plugin` | `nav2_theta_star_planner/ThetaStarPlanner` | 必须与插件 XML 的 `name` 完全一致 |
| `allow_unknown` | `true` | 允许在未知区规划 |
| `w_traversal_cost` | `10.0` | 越大越远离内切区 |
| `FollowPath` 速度上限 | `0.45` m/s / `1.0` rad/s | 与底盘一致 |
| `lookahead_dist` | `1.2` m（0.5–2.5） | 前瞻 |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | `0.2` m / `0.3` rad | 车体 0.45 m，再大就会提前判定到达 |
| `velocity_smoother.max_velocity` | `[0.45, 0.3, 1.0]` | 与 `/cmd_vel` 限制一致，`scale_velocities: true` |

### 4.3 膨胀层为什么是 1.04 / 3.0

原来的 `0.35 / 5.0` 与 `robot_radius: 0.26` 只差 0.09 m，**膨胀带只有 2 格**，
外沿代价衰减到 63 就被 `inflation_radius` 截断。实测（基线包的 `/local_costmap/costmap`）
整张图只有 12 个取值，离墙 0.10 m 以外代价统统是 0。

而 Theta* 的单格通行代价是 `26 + 0.9 × costmap_cost`（`theta_star.hpp` 的 `getCost`），
代价场一片 0 就等于**沿最短线走**，规划器没有任何居中倾向——基线跑里车在 2.3 m 走廊的
横向摆幅是 1.14 m，基本是"走到哪算哪"。

改成 `1.04 / 3.0` 后实测的代价剖面（离墙距离 → 代价）：

| 0.26 m | 0.35 | 0.45 | 0.55 | 0.65 | 0.75 | 0.85 | 0.95 | 1.04 | >1.04 |
|---|---|---|---|---|---|---|---|---|---|
| 99 | 75 | 56 | 41 | 31 | 23 | 17 | 12 | 10 | 0 |

**加大半径不会误判碰撞**：控制器只在 `getCost >= INSCRIBED_INFLATED_OBSTACLE (253)` 时判碰撞
（`omni_pid_pursuit_controller.cpp:505`），253 那一圈由 `robot_radius` 决定，恒为 0.26 m。
可通行宽度同样不变。

> ⚠️ 居中效果**尚未完成实车整程验证**，详见 §7 末尾。

## 5. 地图与先验资产

| 文件 | 说明 |
|---|---|
| `pb2025_nav_bringup/map/reality/lab.{yaml,pgm}` | 本场地 2D 栅格：272×292 @ 0.05 m，origin `[-4.06, -8.23, 0]` |
| `pb2025_nav_bringup/pcd/reality/lab.pcd` | 本场地先验点云：51,968 点（0.1 m 降采样） |
| `config/arenas/lab/` | 同一套资产的备份位，与 `rmul/` 对齐 |
| `~/2026_new_sentry/maps/lab.*` | `map_saver_cli` 的原始落盘位置 |

`install/` 是按文件建的 symlink：**新增**资产后必须重新
`colcon build --symlink-install --packages-select pb2025_nav_bringup`，否则 install 里看不到。
`rviz_navigation.launch.py` 默认仍指 RMUL；`sentry.launch.py` 默认已改为 lab，换场地要传
`map:=` 与 `prior_pcd_file:=`。

## 6. 模式与遥控开关

| 入口 | 用途 | 关键参数 |
|---|---|---|
| `sentry.launch.py`（`start_sentry.sh`） | 定位导航整栈（GICP + Nav2） | `map:=`、`prior_pcd_file:=`、`enable_motion:=` |
| `mapping.launch.py` | **遥控建图**（无 Nav2） | `keyboard_teleop:=true\|false`、`rviz:=` |
| `rviz_navigation.launch.py` | 单机导航/实时建图 | `slam:=true`（建图）、`slam:=false`（定位） |

`mapping.launch.py` 的 `keyboard_teleop`：

- `true`（默认）：打开 USB 运动门控，另开终端运行 `ros2 run usb_bridge keyboard_teleop`；
  该节点运行期间固件认为 USB 在线，**遥控器被屏蔽**。
- `false`：门控关闭，`/cmd_vel` 不驱动底盘，底盘只认下位机遥控器。

> 注意：当前文档记录的固件 target 是 `sentry_2026_chassis`，其
> `ROBOT_APP_ENABLE_INPUT` 为 `0U`（不启动 Input/DBUS）。若实车仍是该固件，
> `keyboard_teleop:=false` 时底盘没有任何输入——开遥控器前先确认固件 target。

## 7. 换底盘后的参数校准记录（2026-10-23 ~ 10-24）

换底盘后**重新标定过的全部量**。数值来源是实测，不是 URDF/CAD——URDF 在这台车上至少错过
一项（`lidar_z` 差 7.6 cm）。下次换车照这张表逐项过一遍。

| 量 | 换车前 | 现在 | 怎么定的 | 依据 |
|---|---|---|---|---|
| 车体半径 `robot_radius` | 沿用母工程 `0.20` | **`0.26` m** | 实测车边缘到圆心；它同时决定碰撞阈值（253 那圈）与可通行宽度 | §4.1 |
| 雷达离地 `lidar_z` | URDF `0.120` | **`0.200` m** | `calib_ground.py` 拟合 1118 个纯地面格，残差 6.9 mm；卷尺量外壳顶 0.200 复核 | [`chassis_calibration.md`](chassis_calibration.md) §0 |
| 安装姿态 | URDF 未写 | pitch **2.3°**、roll 0° | 同上点云拟合；< 3° 暂不写入 URDF | 同上 |
| 地面在 `base_footprint` 的高度 | 隐含 0 | **+0.078 m** | `lidar_z` 抬高后点云整体上移，`base_footprint` 与真实地面的差变了 | 同上 §4.1 |
| `pointcloud_to_laserscan` 高度带 | `0.08 / 1.0` | **`0.16 / 0.50`** | 上一行推出下沿；上沿按 Humble 收紧到 0.50，与代价地图同值 | §2 |
| 自遮挡 / 盲区 `blind` | `0.5`（未验证） | **`0.5` 有效** | 自返回环半径小于 0.5 m，`blind` 能挡住 | [`chassis_calibration.md`](chassis_calibration.md) §2 |
| 轮系平移偏角 `frame_yaw` | 无 | **−45°** | 麦轮轮系相对车体转 45°；`wz` 的符号由固件单独取负 | §1.1、[`chassis_speed_reference.md`](chassis_speed_reference.md) |
| 地面相对 `vehicleZ` | — | **`-0.12`** | 与 `lidar_z` 无关，两者同步平移 | §2 |
| `localTerrainMapRadius` | `4.0` | **`6.0`** | 4 m 处有一圈幻影障碍；地面误报 0.43% → 0.09% | §2 |
| 膨胀带 | `0.35 / 5.0` | **`1.04 / 3.0`** | 见 §4.3 | §4.3 |
| 底盘有效速度 | — | **`0.45 / 0.30 / 1.0`** | 实测各方向上限；`/cmd_vel` 钳位、`velocity_smoother` 与固件三处同值 | §1.1 |

### 还没验证完的

- **膨胀层的居中效果只有离线验证**。离线复算的中线偏差从 0.31 m 降到 0.10 m（居中起步）、
  0.49 → 0.13 m（偏侧起步），最短距离 0.85 → 1.10 m，路径只长 1.5%。
  实车唯一一次 (`1.5 / 2.0`) 只跑了 67.7 s / 7.79 m，而且起始位置和基线不同，
  轨迹整体带 −0.39 m 常值偏移，**不足以判定**。整程复跑（≥15 m 往返、同一起点）待做。
- **动态障碍清除**：`clearingCloud` 改回一次性语义、`decayTime 0.2` 之后，运行时行为尚未上机验证
  （`56d562c` 提交时已注明）。
