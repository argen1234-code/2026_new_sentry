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
| | `terrainUnderVehicle` | `-0.12` | 雷达到地面实测高度（2026-10-23） |
| | `ceilingFilteringThre` | `2.0` | 过滤天花板 |
| | `localTerrainMapRadius` | `4.0` | **只输出车周 4 m**；2D 建图稀疏的直接原因 |
| `pointcloud_to_laserscan` | `min_height` / `max_height` | `0.08` / `1.0` | 只保留离地 8 cm–1 m 的点 |
| | `min_intensity` | `0.5`（`nav2_params.yaml`） | terrain 的 intensity 是高度差，低于此值不进 2D 图 |
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
| `min_obstacle_height` / `max_obstacle_height` | `0.0` / `2.0` | 障碍高度带 |
| local 来源 | `terrain_map`，5.0 m | 随母工程 |
| global 来源 | `terrain_map_ext`，10.0 m | 随母工程 |
| `obstacle_min_range` | `0.2` m | 车体附近的点不可靠 |
| `inflation_layer` | `cost_scaling_factor: 5.0`，`inflation_radius: 0.35` | 比母工程（4.0 / 0.7）更贴车体；当前规划验证通过 |
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
