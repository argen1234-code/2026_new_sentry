# 底盘切换接手文档（麦轮训练车 → 正式底盘）

> 适用：当前 `2026_new_sentry` 工程里的 ROS 2 导航栈与 STM32 固件。
> 目标：把“麦轮训练车”上验证过的建图/导航链路，迁移到正式底盘时知道**改哪里、为什么改、怎么验收**。
>
> 分支说明：本文档最初按桌面 Ubuntu 24.04 / ROS 2 Jazzy 记录；微型主机侧
> （`feature/humble-official-chassis`）为 Ubuntu 22.04 / ROS 2 Humble，
> 命令中的 `jazzy` 请替换成 `humble`，Nav2 接口差异见 `ros_ws/README.md`
> 的 Humble 适配表。

---

## 1. 当前测试底盘（事实）

| 项 | 值 | 来源 |
|---|---|---|
| 底盘 | 四轮麦克纳姆 | — |
| 电机 | 达妙 DMH6215 ×4，FDCAN1，1 Mbps 经典 CAN | `docs/hardware.md` |
| 电机 ID | RR 1 / FL 2 / RL 3 / FR 4，反馈 0x51–0x54 | 同上 |
| 轮半径 | 0.045 m | `sentry_2026_param.h` |
| 轮心 x | ±0.1716 m | URDF |
| 轮心 y | ±0.1362 m | URDF |
| 中心距 L/2+W/2 | 0.3077 m | `ROBOT_CHASSIS_CENTER_DIST` |
| 底盘外形 | 0.450 × 0.250 × 0.086 m | URDF |
| 雷达安装 | x=0.15, y=0, 离地 0.21 m，yaw=0（X 轴朝车头，已实测） | URDF |
| 底盘控制周期 | 5 ms / 200 Hz | `sentry_2026_param.h` |
| DM 使能重发 | 20 ms | 同上 |
| 反馈丢失/故障失能 | 1000 ms / 1000 ms | 同上 |
| 有效基础速度 | **0.45 m/s 前进 / 0.3 m/s 横移 / 1.0 rad/s** | USB 桥钳位 + 已验证 10 rad/s 轮速 |
| 固件 target | `sentry_2026_chassis` | `stm32_ws/legacy/Robot/Sentry/` |
| USB | CDC `0483:5740`，`AA 55 | id | seq | len | payload | xor` | `docs/protocol.md` |

> 注意：`ROBOT_CHASSIS_MAX_SPEED_X/Y` 目前仍是 2.5 m/s，那是固件内部上限；
> 实际导航速度由 ROS 侧 `velocity_smoother` 与 `cmd_vel_to_unified` 钳到 0.45/0.3/1.0。

---

## 2. ROS 侧数据链路

```text
MID360
 ├─ /livox/lidar ─┐
 └─ /livox/imu  ──┴─► point_lio ─► /cloud_registered (camera_init)
                                   │
                                   ▼
                            loam_interface ─► /registered_scan (odom)
                                   │            └─► /lidar_odometry
                                   ▼
                    terrain_analysis ─► /terrain_map ─► local_costmap
                    terrain_analysis_ext ─► /terrain_map_ext
                                   │            ├─► global_costmap
                                   │            └─► pointcloud_to_laserscan
                                   │                     └─► /obstacle_scan
                                   │                          └─► slam_toolbox ─► /map
                                   │                                              └─► static layers
                                   ▼
                    sensor_scan_generation ─► /sensor_scan, /odometry, TF odom→base_footprint
```

控制链：

```text
RViz 2D Goal Pose
 → bt_navigator (NavigateToPose)
 → planner_server (ThetaStar)
 → controller_server (OmniPidPursuit, 20 Hz) → /cmd_vel_controller
 → velocity_smoother → /cmd_vel_nav2_result
 → fake_vel_transform → /cmd_vel
 → usb_bridge/cmd_vel_to_unified → /unified_cmd
 → STM32（CMD_VALID | CMD_MOTION_ENABLE，100 ms 新鲜度）
```

TF 链：

```text
map → odom → base_footprint → base_link → gimbal_yaw → gimbal_yaw_fake
                                     └────► front_mid360
                                     └────► imu_link
```

- `map→odom` 现在由 **slam_toolbox** 发布（`transform_publish_period: 0.02`）
- `gimbal_yaw→gimbal_yaw_fake` 由 `fake_vel_transform` 发布
- 当前 `use_gimbal: false` → 该变换恒等

---

## 3. 换底盘时必须改的地方

### 3.1 机械 / URDF

默认模型：`ros_ws/src/sentry_description/urdf/sentry_2026_static.urdf`
（2026 正式底盘 + 四轴云台导出，`chassis_model:=sentry_2026`）。
旧训练车模型 `mecanum_static.urdf`（`chassis_model:=mecanum`）和更早的
舵轮模型 `robot.urdf`（`chassis_model:=sentry`）保留为备用。

2026 模型里必须更新：

| 参数 | 位置 |
|---|---|
| `base_footprint` 离地高度 | `base_footprint_to_base_link` 的 z（当前假设 CAD 原点在地面，取 0） |
| 雷达位姿 | `joint_ref_lidar` 的 origin/rpy（当前 x=0.0603, y=-0.1143, z=0.1139 相对 `link2_laser`，pitch 22°） |
| 云台 yaw 轴位置 | `joint1_big_yaw` 的 origin |

**帧名不要改**：`base_footprint`、`base_link`、`gimbal_yaw`、`front_mid360`、`imu_link`。
Nav2、Point-LIO、terrain、USB 桥都按这些名字引用（导出里的 `base`、
`link1_big_yaw`、`ref_lidar` 已分别改名）。

若再换一版模型，直接新建 URDF 并用 `robot_description_file:=<path>` 指定。

> **URDF 里的雷达 z 与姿态不可信，必须实测**：新底盘装配误差会让实际离地高度与设计值
> 差好几厘米（2026-10-24 实测：URDF 0.120 m，真实 0.196 m，偏小 7.6 cm）。
> 测量方法与配套脚本见 [`chassis_calibration.md`](chassis_calibration.md)。

### 3.2 STM32 固件

文件：

- `stm32_ws/legacy/Robot/Sentry/sentry_2026_chassis.h`（target 选择）
- `stm32_ws/legacy/Robot/Sentry/sentry_2026_param.h`（参数）
- `stm32_ws/legacy/Component/Controller/Chassis/Src/chassis_mecanum.c`（运动学）

需要改：

| 项 | 当前值 | 说明 |
|---|---|---|
| `ROBOT_CHASSIS_KIND_*` | `MECANUM` | 舵轮/差速要换运动学 |
| `ROBOT_CHASSIS_WHEEL_RADIUS_M` | 0.045 | 新轮径 |
| `ROBOT_CHASSIS_CENTER_DIST` | 0.3077 | 新 L/2+W/2 |
| `ROBOT_CHASSIS_HALF_LENGTH_M/WIDTH_M` | 0.1716 / 0.1362 | URDF 几何 |
| 电机 bus/id/方向 | RR/FL/RL/FR | `*_map.h` 与 `chassis_*` 的符号 |
| `ROBOT_CHASSIS_MIN_ONLINE_MOTORS` | 4 | 正式车电机数 |
| `ROBOT_CHASSIS_USB_CMD_ENABLE` | 1 | 上位机控制开关 |
| 控制周期 / 使能周期 | 5 ms / 20 ms | 与 CAN 负载一起评估 |
| 基础速度 | 0.45 / 0.3 / 1.0 | 同步改 ROS 侧钳位 |
| `ROBOT_CHASSIS_DM_IDLE_DISABLE_MS` | 300 | 空闲失能时间；换电机/驱动后重新确认 |
| `ROBOT_CHASSIS_DM_START_RETRY_MS` | 500 | 启动反馈未就绪时的重试周期 |

改完后先用台架 target（`can1_dmh6215_test` 或 `sentry_2026_chassis` + `APP_CHASSIS_BENCH_TEST`）
验证四轮转向与符号，再上车。

#### 当前 DM 使能/失能策略（重要）

速度模式（`CTRL_MODE=3`）下，电机在目标 0 时仍会闭环保持，产生保持力矩和
轻微电流声。当前固件在 `app_chassis.c` 里做了以下管理：

| 阶段 | 行为 |
|---|---|
| 上电/复位 | 写 `CTRL_MODE=3` → 使能 → 等待反馈；**反馈未就绪不锁存**，每 500 ms 重试 |
| 正常运行 | 每 20 ms 重发使能，补偿单帧丢失 |
| 空闲（Stop） | 持续 300 ms 后：先发零速，再逐轮失能；失能后电机不再上报反馈（正常） |
| 收到运动指令 | 先给四轮补写 `CTRL_MODE=3`，再重新使能；随后恢复 5 ms 速度帧 |
| 单轮掉使能 | 运行中某轮反馈 `state=0` 且仍有输出：每 300 ms 补写模式并使能该轮 |
| 真实故障 | 就绪后四轮反馈持续丢失 1 s → 锁存、失能，必须复位 |

`/chassis/state.fault_flags` 阅读方式：

| 值 | 含义 | 是否正常 |
|---:|---|---|
| 0 | 正常 | ✅ |
| 16 | bit4：当前未使能 | 空闲时正常 |
| 32 | bit5：在线数 ≠ 15 | 空闲失能后正常 |
| 48 | 16 + 32 | 空闲失能常见组合 |
| **1 / 17** | **bit0：故障锁存** | ❌ 需要复位 |

> 已知残留问题（暂时搁置）：空闲失能后电流声已明显减小，但高速正向前进时
> 仍能听到轻微电机啸叫；四轮速度/力矩已一致，暂按正常电机噪声处理。

### 3.3 ROS 导航参数

文件：`ros_ws/src/sentry_navigation/pb2025_nav_bringup/config/reality/nav2_params.yaml`

| 参数 | 当前值 | 换车时 |
|---|---|---|
| `local_costmap.robot_radius` | 0.26 | 新外接圆半径 |
| `global_costmap.robot_radius` | 0.26 | 同上 |
| `inflation_radius` | 0.35 | 半径 + 安全余量 |
| `footprint_padding` | 0.03 | 同上 |
| `velocity_smoother.max_velocity` | `[0.45, 0.3, 1.0]` | 新底盘速度上限 |
| `velocity_smoother.max_accel` | `[2.5, 2.5, 4.0]` | 新底盘斜坡 |
| `controller_server.min_y_velocity_threshold` | 0.001 | 非麦轮可调大 |
| `controller_server.v_linear/v_angular` | ±0.45 / ±1.0 | 新上限 |
| `goal_checker.xy/yaw` | 0.2 m / 0.3 rad | 按车体尺寸 |
| costmap `min/max_obstacle_height` | 0.0–1.0 m | 新车体高度 |
| `terrainUnderVehicle` | -0.12 | 新雷达离地高度（点云实测，取负值） |
| `minRelZ/maxRelZ/lowerBoundZ/upperBoundZ` | -0.35/0.8, -0.6/0.8 | 重新标定 |
| `pointcloud_to_laserscan.min_height/max_height` | 0.15 / 1.07 | 重新标定，且必须与 `lidar_z` 同步改 |
| `fake_vel_transform.use_gimbal` | `False` | 正式云台车改 `True` |

文件：`ros_ws/src/sentry_hardware/usb_bridge/usb_bridge/cmd_vel_to_unified.py`

- `max_vx` / `max_vy` / `max_wz` 默认 0.45 / 0.30 / 1.0
- 换车后与固件、`velocity_smoother` 三处保持一致

文件：`ros_ws/src/sentry_hardware/usb_bridge/usb_bridge/usb_bridge_node.py`

- 轮序映射：MCU 顺序 `[RR, FL, RL, FR]` → URDF 关节名
- 车轮符号 `[-1, +1, +1, -1]`、半径 0.045
- 新车轮径/轮序/符号都要改

### 3.4 定位 / 感知

文件：`nav2_params.yaml` 的 `point_lio` 段、`sentry_localization/point_lio/config/mid360.yaml`

- `mapping.gravity` / `gravity_init`：当前 `[0, 0, -9.810]`，与 MID360 实测 +1 g 对应。
  换 IMU 后必须重新确认；符号错会让姿态翻转 180°。
- `mapping.extrinsic_T/R`：雷达相对 IMU 的外参
- `preprocess.lidar_type/scan_line/timestamp_unit`：换雷达时改
- `publish.scan_bodyframe_pub_en`：当前 `False`；`cloud_registered_body` 不参与导航
- `pcd_save.pcd_save_en`：当前 launch 强制 `True`，长时间建图会累积内存，正式跑可关

MID360 安装位姿变化后，`terrainUnderVehicle`、`minRelZ/maxRelZ`、
`pointcloud_to_laserscan.min_height/max_height` 都要重新标定。

> 标定流程（采集 → 拟合 → 换算参数 → 验收判据）见
> [`chassis_calibration.md`](chassis_calibration.md)，分析用
> `ros_ws/tools/calib_ground.py <bag>`。注意 `lidar_z` 与 `min_height` 必须一起改，
> 只改一个是错的（原因见该文档 §4.1）。

---

## 4. 正式底盘带云台时额外要做的

1. **`fake_vel_transform.use_gimbal: True`**
   - 云台 yaw 与底盘解耦时，`gimbal_yaw` 是动态帧；
   - 该节点会把 Nav2 在云台系下的速度旋转到底盘系，并发布 `gimbal_yaw_fake`。
2. **确认云台 yaw 的来源**
   - 当前 `current_robot_base_angle_` 来自 `/odometry` 的 yaw；
   - 正式车应改成本地 `云台相对底盘 yaw`（来自云台电机/编码器），
     否则会像麦轮车一样被车体 yaw 误旋转。上真车前必须复核这段逻辑。
3. **URDF 切换**
   - 默认已改为圆形全向轮模型 `chassis_model:=omni_round`
     （`urdf/omni_round.urdf.xacro`）；2026 模型仍可用
     `chassis_model:=sentry_2026` 切回；
   - 新模型的 `gimbal_yaw` 固定在 `base_link`（本车无云台），`imu_link`
     与 `base_link` 重合，两者都是为了满足 Nav2 / usb_bridge 的帧名；
   - 云台反馈接入后要把 `gimbal_yaw` 改成可动关节。
4. **`robot_base_frame`**
   - costmap / behavior 继续用 `gimbal_yaw_fake`；
   - Nav2 planning 使用 `gimbal_yaw`。
5. **轮速与舵向标定**
   - 舵轮底盘先做转向零位、方向、轮速符号台架测试，再开导航。

---

## 5. 换车验收清单

按顺序做，前一步不通过不要进下一步：

1. **TF**
   ```bash
   ros2 run tf2_ros tf2_echo base_footprint front_mid360
   ros2 run tf2_ros tf2_echo map base_footprint
   ```
   期望：雷达 x/y/z 与实测一致；`map→base_footprint` 连续。
2. **里程计**
   ```bash
   ros2 topic hz /aft_mapped_to_init    # ~20 Hz
   ros2 topic hz /odometry              # ~20 Hz
   ```
   静止时位姿不漂；手推 1 m，位移与卷尺误差 < 5%。
3. **点云/扫描**
   ```bash
   ros2 topic hz /cloud_registered
   ros2 topic hz /obstacle_scan         # 单发布者
   ```
   地面不出现在 `obstacle_scan` 里。
4. **代价地图**
   RViz 看 `local_costmap/costmap`、`global_costmap/costmap`；
   障碍位置与实物一致，膨胀不过大/过小。

   **静态障碍物验证（车静止、环境不变）**：
   - 连续采样 10 s，同一障碍的 occupied 格数波动应 < 5%
   - 障碍边缘不应随时间“长毛”或缩水；`unknown` 区域不应反复被填/清
   - 地面不应出现在 local costmap（`min_obstacle_height=0.05` 起）
   - 障碍物中心与激光测距的偏差 < 0.1 m
   - 检查 `/terrain_map`、`/terrain_map_ext` 的点数是否稳定
   - 如出现抖动，优先查 `observation_persistence`、`decayTime`、
     `clearingDis` 和 Point-LIO 静止漂移，而不是调膨胀半径
5. **速度链（架空车轮）**
   ```bash
   ros2 launch sentry_bringup rviz_navigation.launch.py slam:=true enable_motion:=true
   ```
   先发一个 0.3 m 的目标，确认 `/cmd_vel` 方向正确、四轮转向正确、速度不超限。
6. **落地低速**
   0.2 m/s、1 m 直线 → 原地转 90° → 再走 1 m；确认无碰撞、无跑偏。
7. **建图**
   推/开到 2–5 m，走一个小回环；保存：
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/2026_new_sentry/maps/<name>
   ```
8. **失联保护**
   拔 USB / 停桥接，确认底盘在 500 ms–1 s 内零速。

---

## 6. 已知坑（本次已踩过）

1. **launch 参数名冲突**
   `description.launch.py` 与 `sentry_localization/mapping.launch.py` 都用过 `params_file`，
   全局配置只保留第一次声明，导致 Point-LIO 拿到错误的 YAML 并 SIGSEGV。
   现已改名为 `description_params_file` / `nav2_params_file`。
   **新增 launch 参数时避免通用名。**
2. **插件分隔符必须与插件 XML 的注册名一致**
   本机 nav2 是 1.1.20（Humble），pluginlib 按插件 XML 里的 `name` 查找，**没有**跨版本兼容：
   - XML 带 `name=` 的包（`nav2_theta_star_planner`、`nav2_behaviors`、`nav2_smac_planner`、
     `nav2_navfn_planner`、`pb_nav2_plugins` 的 behavior）必须写成 `/`：
     `nav2_theta_star_planner/ThetaStarPlanner`、`nav2_behaviors/Spin`、
     `pb_nav2_behaviors/BackUpFreeSpace`。
   - XML 只有 `type=` 的包（`nav2_costmap_2d`、`nav2_controller`、`nav2_smoother`、
     `nav2_waypoint_follower`、`pb_omni_pid_pursuit_controller`、`pb_nav2_plugins` 的
     costmap）用 `::`。

   写成 `nav2_theta_star_planner::ThetaStarPlanner` 会 FATAL：
   `planner_server: Failed to create global planner ... does not exist`，
   `lifecycle_manager_navigation: Failed to bring up all requested nodes. Aborting bringup`。
   **症状是 RViz 里设目标点没任何反应**（Nav2 全部停留在 unconfigured/inactive），
   而且 1–2 分钟后容器会以 `exit code -6` 退出。查这个问题先看
   `~/.ros/log/latest/component_container_isolated_*.log`。
3. **`plugin_lib_names` 位置**
   Jazzy 1.3.13 下放在 `bt_navigator` 根节点会被注册两次（`ID [ComputePathToPose] already registered`）。
   必须放在 **`navigate_to_pose.plugin_lib_names`** 下。
4. **BT XML 节点 ID 唯一**
   `RecoveryNode name="ComputePathToPose"` 会和动作节点 `<ComputePathToPose>` 的 ID 冲突；
   同类节点（`ClearLocalCostmap-Context` 出现两次）也要改名。
5. **slam_toolbox 必须 configure + activate**
   普通 `Node` 启动不会发布 `/map`；已改为 `LifecycleNode` + 自动 configure/activate。
6. **`map→odom` 不能双发布**
   已删除静态 `map→odom`，由 slam_toolbox 以 `transform_publish_period: 0.02` 发布。
7. **Point-LIO 重力符号**
   用旧 IMU 的 `+Z` 单位向量会让姿态 roll 翻转 180°；当前固定 `[0,0,-9.810]`。
8. **`publish_odometry_without_downsample`**
   `True` 会在逐点循环里发布里程计（实测 >1 kHz），压垮 TF/同步；当前 `False`。
9. **`map_saver` 启动竞态**
   高负载时 lifecycle_manager 可能在 `map_saver` 就绪前配置失败；
   不影响导航，保存地图用 `map_saver_cli` 即可。
10. **MID360 yaw 已实测修正**
    原 `front_mid360` yaw=-90° 来自旧 2D 雷达模型，实测会把物理前方映射到右扇区；
    已改为 **0°**。换车或重新安装雷达后，用“左右后近、前方远”的扇区距离重新确认。
11. **重复启动会留下孤儿节点**
    多次 kill/重启后可能同时存在**多个** `usb_bridge` 和 `cmd_vel_to_unified`。
    不同实例的 `enable_motion` 不同，会在 `/unified_cmd` 上交替发 NoFollow/Stop，
    表现为轮子一顿一顿、`mode` 在 2 和 0 之间反复切换。
    每次启动前先清理：
    ```bash
    ~/2026_new_sentry/ros_ws/tools/stop_ros_stack.sh
    ```
12. **RViz launch 参数名冲突**
    `lidar.launch.py` 原来也声明 `rviz`，上层 `rviz:=true` 会让它再开一个窗口；
    已改名 `lidar_rviz`。**新增 launch 参数不要用 `rviz`、`params_file` 这类通用名。**
13. **小屏 RViz 窗口与配色**
    1024×600 屏幕上 Nav2 默认面板的最小高度 >1100 px。已新增
    `rviz/nav2_compact_view.rviz`（只保留 Displays，1000×520），
    并由 `rviz_navigation` / `mapping` / `sentry` 默认使用。
    - 配色参照 `2026_robot_base/robot_base.rviz`，只改颜色值：
      全局路径 `240;50;40`、局部路径 `40;120;255`、扫描 `0;255;0`；
      代价地图仍用标准 `costmap` 配色（与参考工程方案相同）。
      透明度、QoS、线宽/线型、显示项均保持默认。
    - 灰度代价地图备选：`rviz/nav2_compact_gray.rviz`
      （代价地图用 `map` 配色，无青色/品红）
    - 两个入口都支持 `rviz_config:=<path>` 覆盖
    需要完整面板时手动用 `nav2_default_view.rviz`。
14. **空闲失能会让电机停止上报反馈**
    DM 速度模式失能后反馈消失，原“反馈丢失 1 s 锁存”会误判。
    当前已在 `idle_disabled` 期间跳过在线数故障判定。
    **不要在没有这一条件的情况下单独启用空闲失能。**
15. **重新使能必须补写 CTRL_MODE=3**
    实测左后轮（RL / ID 3）在失能→重新使能后会丢失速度模式，
    表现为目标正确但 `state=0`、单轮不转、其他轮正常。
    当前在重新使能前统一补写 `CTRL_MODE=3`，并每 300 ms 对
    `state=0` 的单轮自动补模式+使能。
16. **重新使能条件不能依赖 `output.send_enable`**
    失能后无反馈 → `chassis_step` 拒绝输出 → `send_enable=0`；
    若把它作为重新使能条件会形成死锁。只看“是否有运动指令”。
17. **烧录后 USB CDC 可能不枚举**
    表现为 `/dev/ttyACM0` 消失、`lsusb` 无 `0483:5740`。
    拔插 STM32 的 USB 线（不是 DAPLink）或整车断电重启即可恢复。
    固件本身仍在运行，可用 DAPLink 读取 `loop_count` 确认。

---

## 7. 本会话改动清单

| 文件 | 改动 | 原因 |
|---|---|---|
| `sentry_localization/.../mapping.launch.py` | `params_file`→`nav2_params_file`；slam_toolbox 改 LifecycleNode；删除静态 `map→odom` | 修参数冲突、激活建图、TF 由 slam_toolbox 发布 |
| `sentry_description/launch/description.launch.py` | `params_file`→`description_params_file` | 避免 launch 参数名冲突 |
| `sentry_bringup/launch/sentry.launch.py` | 传 `nav2_params_file` | 同上 |
| `sentry_bringup/launch/rviz_navigation.launch.py` | 新增 `enable_motion` 参数；删除重复 `cloud_to_scan` | 允许驱动底盘；去重 |
| `sentry_perception/.../perception.launch.py` | 删除 `pointcloud_to_laserscan` | `/obstacle_scan` 双发布 |
| `fake_vel_transform`（源码） | 新增 `use_gimbal` 参数 | 麦轮车无云台，恒等变换 |
| `nav2_params.yaml` | 插件 `::`；`plugin_lib_names` 移入 navigator；BT ID 去重；速度/阈值/容差；`use_gimbal`；`transform_publish_period`；`scale_velocities`；gravity | 让 Nav2 真正可用并匹配底盘能力 |
| `point_lio` gravity | `[0,0,-9.810]` | 修复姿态翻转 |
| `robot_state_publisher.yaml` | `publish_frequency: 50` | 降低 `/tf` 负载 |
| `usb_bridge/cmd_vel_to_unified.py` | 50 Hz 定时重发；`command_timeout` 0.5 s | `/cmd_vel` 抖动时不再清零，消除轮子卡顿 |
| `usb_bridge/usb_bridge_node.py` | `command_timeout` 0.25 → 0.4 s | 同上 |
| `fake_vel_transform`（源码） | 总是缓存输入，100 Hz 定时发布 | `/cmd_vel` 从 6–12 Hz 提升到 ~90 Hz |
| `nav2_compact_view.rviz`（新增） | 仅 Displays、1000×520、10 fps、关闭重点云显示 | 适配 1024×600 屏幕、降低 CPU |
| `lidar.launch.py` | `rviz` → `lidar_rviz` | 修复启动两个 RViz |
| `tools/stop_ros_stack.sh`（新增） | 清理孤儿节点 | 避免多桥接同时控制底盘 |
| `app_chassis.c` / `sentry_2026_param.h` | 空闲 300 ms 失能；重新使能补 CTRL_MODE；启动重试；单轮失能恢复 | 消除空闲电流声，且修复 RL 丢失速度模式 |
| `docs/chassis_swap_handover.md`（新增） | 换底盘接手文档 | 记录参数、流程、已知坑 |

---

## 8. 启动指令

> 每次启动前先清理上一次可能残留的节点，尤其是反复 kill 过之后：
> ```bash
> ~/2026_new_sentry/ros_ws/tools/stop_ros_stack.sh
> ```

### 只建图

```bash
cd ~/2026_new_sentry/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch sentry_bringup mapping.launch.py rviz:=true
```

### 建图 + Nav2 + RViz 点目标（当前测试车）

第一次先架空车轮：

```bash
ros2 launch sentry_bringup rviz_navigation.launch.py slam:=true enable_motion:=false
```

确认 `/cmd_vel` 方向与四轮转向后，再落地：

```bash
ros2 launch sentry_bringup rviz_navigation.launch.py slam:=true enable_motion:=true
```

RViz 里用 **2D Goal Pose** 点目标；Fixed Frame 选 `map`。

### 保存地图

```bash
mkdir -p ~/2026_new_sentry/maps
ros2 run nav2_map_server map_saver_cli -f ~/2026_new_sentry/maps/<name>
```
