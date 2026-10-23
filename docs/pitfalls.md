# 踩坑记录

> 每条按「现象 / 根因 / 修正 / 涉及文件 / 验证方式 / 日期」记录。新增条目追加在末尾或按主题插入，
> 不要删除历史条目（结论被推翻时标注「已修订」并写清新结论）。
> 母工程对比基线：`reference/pb2025_sentry_nav`，克隆于 2026-10-23，commit `9bfcfee`。

---

## 1. 导航「原地打转不前进」/ 前进却横移 90°

- **现象**：导航启动后车体自转，或下发前进指令车体横移；LIO 测得实际走向比命令稳定多 +45°。
- **根因**：
  1. 固件 `wz` 符号整体取反——下发 `+1.0 rad/s`（REP-103 逆时针）时 MID360 陀螺仪
     `gyro.z` 实测为负，车体实际顺时针转。
  2. 轮系相对 `base_footprint` 转过 45°，ROS 侧缺少补偿。
- **修正**：
  - 固件 `chas_inv_cal`：`wz_speed = -wz_set * CHASSIS_OMNI_ROTATE_RADIUS`；
    `chas_for_cal` 的 `*wz` 同步取负；`Omni_chassis.c` 两处 `yaw_force` 取负。
    **两棵树（`chassis_fw`、`sentinel`）都要改。**
  - ROS 侧 `cmd_vel_to_unified` 的 `frame_yaw` 改为 `-0.7853981634`（每 tick 重读，
    `ros2 param set` 即时生效，无需重启）。
  - 「顺时针 90°」（205/207 吃 `-vx`、206/208 吃 `+vy`）是 45° 修正的一部分历史，**别回退**。
- **涉及文件**：`stm32_ws/`（`chassis_fw` / `sentinel` 的底盘标定与 `Omni_chassis.c`）、
  `ros_ws/src/sentry_hardware/usb_bridge/usb_bridge/cmd_vel_to_unified.py`
- **验证方式**：自转方向只看 `/livox/imu` 的 `angular_velocity.z`（与 LIO 无关的独立信源）；
  平移看 `/odometry` 在起始车体系里的位移方向角。
- **补充实测（2026-10-23，键盘直连）**：以雷达安装方向为 0°，`W/S/A/D` 的现实运动方向
  分别为逆时针 135°、顺时针 45°、顺时针 135°、逆时针 45°，详见
  [`chassis_speed_reference.md`](chassis_speed_reference.md) 第 3 节。
- **日期**：2026-10-23

## 2. 底盘速度环跟不上目标（约 0.25 m/s 死区，稳态仅 ~35%）

- **现象**：`/cmd_vel` → `/unified_cmd` → USB → `chassis_fw` 全链路通，但车速远低于目标。
  实测 vx 阶跃（右/左轮稳态）：

  | vx 目标 (m/s) | 0.05–0.20 | 0.25 | 0.30 | 0.50 | 0.60 |
  |---|---|---|---|---|---|
  | 轮速 (m/s) | 0 | 0.034 | 0.092 | 0.178 | 0.212 |

- **根因**：轮系静/动摩擦大（稳态转矩约 2.4–3 N·m/轮），而 `CHASSIS_SPEED_PI_KP=1600`
  只产生约 0.4 A 电流。**不是 ROS 侧问题**：MCU 持续收到 vx=200 mm/s、mode=2，
  `online_mask=15`，电流与反馈均正常。
- **修正**：待定。方向是提高速度环 P/积分限幅或加大静摩擦前馈
  （`CHASSIS_FF_STATIC_CURRENT`）；改完需重新烧录并复测死区表。
- **涉及文件**：`stm32_ws/` 底盘固件（速度环参数）
- **验证方式**：重跑上表的 vx 阶跃，看稳态轮速是否达标
- **日期**：2026-10-23
- **参考**：推荐速度与方向约定见 [`chassis_speed_reference.md`](chassis_speed_reference.md)

## 3. `/livox/imu` 订阅读不到数据（QoS / 发现延迟）

- **现象**：新写的订阅节点在 `ros2 topic echo` 正常的情况下读不到 IMU，或启动头几秒是 “NO DATA”。
- **根因**：`/livox/imu` 的 publisher 是 RELIABLE，订阅端用 `qos_profile_sensor_data`
  （BEST_EFFORT）会不匹配；另外节点发现需要 2–3 s。
- **修正**：订阅端使用 RELIABLE QoS；启动后留足 2–3 s 再判读。
- **涉及文件**：任何订阅 `/livox/imu` 的节点
- **验证方式**：`ros2 topic info /livox/imu --verbose` 看 QoS 是否匹配
- **日期**：2026-10-23

## 4. 建图得到的 2D 地图很稀疏（只有行驶轨迹 + 周围一圈）

- **现象**：`slam_toolbox` 出的栅格只有车走过的区域和车周约 4 m，房间远处大片 unknown；
  同期 PCD 却覆盖整个房间。
- **根因**：2D 栅格由 `terrain_map_ext` 转激光而来，而 `terrain_analysis_ext.localTerrainMapRadius: 4.0`
  只发布车周 4 m 的地形点；再经 `pointcloud_to_laserscan` 的 `min_height: 0.08` 过滤，
  只留下离地 8 cm 以上的点。
- **修正**：2D 地图只作代价地图静态层与 RViz 参考，**不追求覆盖全场地**；全场结构以 PCD 为准。
  真要更大的 2D 图，需调大 `localTerrainMapRadius` 并把车开遍全场，代价是局部代价地图变慢。
- **涉及文件**：`sentry_perception_bringup/config/perception.yaml`、`sentry_localization_bringup/config/slam_toolbox.yaml`
- **验证方式**：建图时 `ros2 topic hz /obstacle_scan` ≈10–20 Hz；跑完后看 `map_saver_cli` 的
  occupied/free/unknown 比例
- **日期**：2026-10-23

## 5. GICP 看起来「偏了 90°」，其实是先验坐标系的比对基准错了

- **现象**：用实时点云暴力搜索先验 PCD 的 yaw，最好解落在 +65°，而 GICP 给出 −27°，误以为 GICP 错了。
- **根因**：`point_lio` 落盘的 PCD 在 `camera_init` 系；`loam_interface` 与
  `small_gicp_relocalization::loadGlobalMap()` **都会再乘一次** `base_footprint→front_mid360` 外参
  （`t=(0,−0.2,0.12)`，yaw −90°）。用**原始先验**做残差比对，等于人为多转了 90°。
- **修正**：比对基准用 `E * prior`（`E` 为上述外参），而不是原始 PCD。改正后 GICP 的 p70 残差
  0.132 m、99.3% 在 0.3 m 内，暴力搜索最优解也落在 −25°，与 GICP 一致。
- **涉及文件**：`sentry_perception/loam_interface/src/loam_interface.cpp`、
  `sentry_localization/small_gicp_relocalization/src/small_gicp_relocalization.cpp`
- **验证方式**：对 `E*prior` 做最近邻距离统计（p70 < 0.2 m 视为可用）
- **日期**：2026-10-23

## 6. 代价地图被障碍层糊死，规划器报“起始位姿是障碍”

- **现象**：机器人静止、周围无接触，`ComputePathToPose` 全部失败：
  “Either of the start or goal pose are an obstacle!”。机器人所在格 cost=99（内切），
  1–2 m 环内 63% 内切 + 20% 致命。
- **根因**：`intensity_voxel_layer.min_obstacle_intensity: 0.0` 把 terrain 点云里大量低强度
  （地面/拖尾）点也当障碍收进代价地图，每格再按 0.26 m 内切半径膨胀。
- **修正**：按母工程把 `min_obstacle_intensity` 改为 `0.1`、`max_obstacle_intensity` 改为 `2.0`、
  `max_obstacle_height` 改为 `2.0`、`obstacle_min_range` 改为 `0.2`，并去掉
  `observation_persistence`/`clearing`/`raytrace_*`/`use_footprint_filter`。改后机器人格 cost=0，
  8 个方向的目标全部规划成功（路径最大 cost 68，无内切/致命格）。
- **涉及文件**：`pb2025_nav_bringup/config/reality/nav2_params.yaml`（local + global 两处）
- **验证方式**：看 `/global_costmap/costmap` 中机器人格的 cost；再发 8 个方向目标，
  检查返回路径的逐格 cost 是否 < 99
- **日期**：2026-10-23

## 7. 代价地图永远等不到 `/map`（StaticLayer QoS）

- **现象**：`planner_server` 卡在 `waitForCostmap()` 的死循环里，不报错、不出路径。
- **根因**：`nav2_costmap_2d::StaticLayer` 默认按 VOLATILE 订阅 `/map`，而 `map_server`/`slam_toolbox`
  以 TRANSIENT_LOCAL 发布；QoS 不兼容时静态层收不到地图，`isCurrent()` 永远为 false。
- **修正**：`static_layer.map_subscribe_transient_local: True`。
- **涉及文件**：`pb2025_nav_bringup/config/reality/nav2_params.yaml`（local + global 两处）
- **验证方式**：启动后 `ros2 topic info /map --verbose` 看发布/订阅 QoS；`/global_costmap/costmap` 应有内容
- **日期**：2026-10-23

## 8. 建图 PCD 丢失（`pcd_save.interval: -1`）

- **现象**：建图跑了很久，结束时没有 `scans.pcd`，或只有 0 字节文件。
- **根因**：`point_lio` 的 `pcd_save.interval: -1` 表示**退出时一次性写盘**；直接 `kill`/`pkill`
  节点不会触发写盘。另外建图入口此前还硬编码了 `pcd_save.pcd_save_en: False`。
- **修正**：建图入口 `sentry_localization/mapping.launch.py` 打开 `pcd_save_en`；结束建图用
  Ctrl-C 让节点正常退出。`pcd_save` 只在建图入口打开，导航入口不打开，避免重复攒 100 MB+。
- **涉及文件**：`sentry_localization_bringup/launch/mapping.launch.py`、`config/point_lio_mid360.yaml`
- **验证方式**：结束前 `ls -lh src/sentry_localization/point_lio/PCD/scans.pcd`，退出后文件大小不再变化
- **日期**：2026-10-23

## 9. 键鼠遥控直接 kill 后遥控器也失效（`app_chassis_usb_armed` 锁存）

- **现象**：用 `ros2 run usb_bridge keyboard_teleop` 让车动过之后直接 `kill`/关终端，
  再拨下位机遥控器，底盘不动；重新上电才恢复。
- **根因**：固件只在 USB 指令带非零速度时置 `app_chassis_usb_armed = 1`。之后只要 USB 指令
  不再有效（断流、门控关闭、桥退出）且该位仍为 1，`app_chassis_fill_cmd` 每周期在 RC 输入之上
  写 `force_stop = 1`，底盘控制器因此拒绝输出。只有再次收到**新鲜且带 MOTION_ENABLE 的 USB 指令**
  （包括零速）才会清 `armed`。
- **修正**：退出键盘节点用 Ctrl-C（节点退出前会补发 10 帧零速，经桥写成零速 USB 指令解除锁存）；
  已经 `kill -9` 的话，重新启动一次 `usb_bridge`/键盘节点让它发零速，或重启底盘电。
- **涉及文件**：`stm32_ws/legacy/Apps/app_chassis.c`（`app_chassis_fill_cmd`）、
  `usb_bridge/usb_bridge/keyboard_teleop.py`（退出时补发零速）
- **验证方式**：`ros2 topic echo /usb_bridge/state` 观察 `force_stop` 位；Ctrl-C 后应回 0
- **日期**：2026-10-23

## 10. 新建地图/先验资产后 `map_server` 加载失败、GICP 输出恒等 `map→odom`

- **现象**：`sentry.launch.py` 里 `map_server` 报找不到 `.../map/reality/lab.yaml`；
  GICP 打印 `Couldn't read PCD file: ...`，之后 `map→odom` 不再更新，Nav2 拿不到定位。
- **根因**：`install/` 是按**文件**建的 symlink，`pb2025_nav_bringup` 用
  `ament_auto_package(INSTALL_TO_SHARE launch map pcd ...)` 在**配置时**枚举文件；
  lab 资产是在上一次 build 之后才加进来的，install 里没有对应 symlink。
- **修正**：新增/改名任何地图、PCD、launch、rviz 文件后重新
  `colcon build --symlink-install --packages-select pb2025_nav_bringup`。
- **涉及文件**：`pb2025_nav_bringup/CMakeLists.txt`、`map/reality/`、`pcd/reality/`
- **验证方式**：`ls -l install/pb2025_nav_bringup/share/pb2025_nav_bringup/map/reality/`
- **日期**：2026-10-23

## 11. 建图与定位的 Point-LIO 参数不是同一份

- **现象**：改 `point_lio_mid360.yaml` 的 `filter_size_surf`，建图的点云密度毫无变化；
  反过来在导航时看到 PCD 越攒越大。
- **根因**：`sentry_localization/mapping.launch.py` 加载的是 `nav2_params.yaml` 里的
  `point_lio` 块（`filter_size_surf 0.1`、`point_filter_num 8`、`scan_line 16`、
  `tf_send_en False`、`pcd_save_en True`），只有 `localization.launch.py` 用
  `config/point_lio_mid360.yaml`（`0.5 / 4 / 4 / True`）。两处互不影响。
- **修正**：按入口改对应文件；定位入口在 launch 里显式覆盖
  `pcd_save.pcd_save_en: False`，避免退出时覆盖 `PCD/scans.pcd`。
- **涉及文件**：`pb2025_nav_bringup/config/reality/nav2_params.yaml`、
  `sentry_localization_bringup/config/point_lio_mid360.yaml`、
  `sentry_localization_bringup/launch/{mapping,localization}.launch.py`
- **验证方式**：启动时看 point_lio 订阅到的参数（`ros2 param get /point_lio filter_size_surf`）
- **日期**：2026-10-23
