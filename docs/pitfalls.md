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
    **已修订（2026-10-29）**：上面这条数值已过时。代码与运行中参数现为
    `-2.356194490192345`（−135°），启动日志打印 `frame_yaw=-2.3562`；以代码为准，
    依据是当日实车跑通（先验图 + GICP，map 系横向误差 p50 2.1 cm / p90 5.8 cm，全程 74 s
    无碰撞无恢复）。若换成 −45° 会整体偏 90°，车会横着走。
    注意该偏角是**轮系相对 `base_footprint` 的安装偏角**，与云台无关；当前阶段车上没有
    云台（`use_gimbal: false`），只有底盘，换底盘或改轮系安装后仍需重测。
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
- **已修订（2026-10-24）**：上述数值已过期。实际生效的是 `nav2_params.yaml` 的
  `terrain_analysis_ext.localTerrainMapRadius: 6.0`、`pointcloud_to_laserscan`
  高度带 **0.16–0.50 m**、量程 0.5–10.0 m；本条引用的
  `sentry_localization_bringup/config/slam_toolbox.yaml` 当前**没有被任何 launch 加载**。
  实测比例见 [`corridor_baseline.md`](corridor_baseline.md)：occupied 1.8% / free 26.7% / unknown 71.5%。

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

## 12. 静止起栈时 `/map` 是 0×0，`planner_server` 永久死锁

- **现象**：`slam:=true` 起栈后车不动，发导航目标后 `planner_server` 接受目标却永不返回
  （连 `(0,0)` 都挂住），容器 CPU 接近空闲——所有线程都在 sleeping。
  日志里两个代价地图反复报 `Received map message is malformed. Rejecting.`
- **根因**：`slam_toolbox` 的 `minimum_travel_distance / minimum_travel_heading = 0.1`，
  车完全静止时一帧都不入图，按 2 Hz 持续发布 **width=0 height=0 的空图**。第一个目标永久占住
  `plan_mutex`，后续全部排队。
- **修正**：发目标前先看 `/map` 的 width/height，非 0 再发。空图时用键盘遥控先走 0.5 m 让 slam
  入第一帧，或改 yaml 把两个门限设为 0（`minimum_travel_distance` 在 configure 时被缓存成成员
  变量，运行时 `ros2 param set` **无效**，必须改 yaml 重启）。
- **涉及文件**：`pb2025_nav_bringup/config/reality/nav2_params.yaml`（`slam_toolbox` 块）、
  `pb2025_nav_bringup/config/reality/nav2_params.yaml`（local/global 的 `static_layer`）
- **验证方式**：`ros2 topic echo --once /map --field info` 看 width/height；
  `grep -c malformed` 应为 0（死锁时实测 150 s 内 1530 次）
- **日期**：2026-10-24

## 13. 建图收尾时地形节点 SIGABRT（-6）、Nav2 容器被 SIGKILL

- **现象**：Ctrl-C 停栈后 `launch.log` 报 `terrainAnalysis` / `terrainAnalysisExt` 与
  `livox_ros_driver2_node` 以 **exit code -6（SIGABRT）** 退出；
  `component_container_isolated`（nav2）与 `referee_serial_bridge_node` 收到 SIGINT 5 s 无响应，
  升级 SIGTERM 后仍无响应，最后被 **SIGKILL**。
- **根因**：未定。只发生在关闭阶段，运行期间 0 条 ERROR，不影响本次建图结果
  （`scans.pcd` 仍按预期在 16:55:49 写盘）。
- **修正**：暂无。判读一份包之前先看 `launch.log` 里这些错误的时间戳是否全在收尾窗口内——
  是则属于已知关闭噪声，不是运行期故障。
- **涉及文件**：无（`terrain_analysis*` 的析构顺序 / `livox_ros_driver2` 的退出路径待查）
- **验证方式**：`awk '{t=substr($1,1,10); if (t>START && t<END && $0~/\[ERROR\]/) c++} END{print c+0}' launch.log`
- **日期**：2026-10-24

## 14. `scans.pcd` 每次建图重写，体积进 git 不合适

- **现象**：每次 `slam:=true` 跑完，`ros_ws/src/sentry_localization/point_lio/PCD/scans.pcd`
  都会被重写；一次 15 m 走廊来回就长到 **153 MB**（此前一次是 42.8 MB，仓库里原本只有
  298 KB）。`git status` 因此长期挂着一个巨大的二进制改动。
- **根因**：`point_lio` 的 `pcd_save.pcd_save_en` 在建图入口打开，产物直接落在源码树内。
- **修正**：该文件不应进版本库。建议加入 `.gitignore` 并 `git rm --cached`（**未执行，待决定**）。
- **涉及文件**：`ros_ws/src/sentry_localization/point_lio/PCD/scans.pcd`、`.gitignore`
- **验证方式**：`git status --short | grep scans.pcd` 应无输出
- **日期**：2026-10-24

## 15. 膨胀带只有 2 格时，规划器完全没有居中倾向

- **现象**：2.3–4.3 m 宽的走廊，全局路径从墙边走、车在走廊里横向摆幅 1.14 m，
  怎么看都像"走到哪算哪"，但规划没有报错、也不撞。
- **根因**：`inflation_radius: 0.35` 与 `robot_radius: 0.26` 只差 **0.09 m（2 格）**，
  `cost_scaling_factor: 5.0` 又让代价在出内切圈第 2 格就衰减到 63。
  实测基线包的 `/local_costmap/costmap` 整张图只有 **12 个取值**，离墙 0.10 m 以外全是 0。
  Theta* 的单格代价是 `26 + 0.9 × costmap_cost`，代价场一片 0 就等于按最短线走。
- **修正**：把半径铺到走廊半宽量级——现为 `inflation_radius: 1.04`、`cost_scaling_factor: 3.0`
  （`1.04 = 0.26 + 3×0.26`）。加大半径**不会**引起误判碰撞：控制器只在
  `getCost >= INSCRIBED_INFLATED_OBSTACLE (253)` 时判碰撞，那圈由 `robot_radius` 决定。
- **涉及文件**：`pb2025_nav_bringup/config/reality/nav2_params.yaml`（local/global 两处）、
  `pb_omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp:505`
- **验证方式**：`ros2 topic echo --once /local_costmap/costmap --field data` 后数不同取值个数
  （改前 12，改后 79）；`ros2 topic hz /global_costmap/costmap` 不应变慢（基线 0.80 Hz）
- **日期**：2026-10-24

## 16. RViz 里代价地图的青色/品红是固定特殊色，不在渐变里

- **现象**：按"蓝→紫→红"的直觉解读膨胀区颜色，得出"最靠近车的是红色、蓝色在中间"的结论，
  与数值完全对不上。
- **根因**：`librviz_default_plugins.so` 的 `makeCostmapPalette()` 把两个特殊值单独写死：
  `99 = INSCRIBED_INFLATED_OBSTACLE` 是**纯青** (0,255,255)，
  `100 = LETHAL_OBSTACLE` 是**纯品红** (255,0,255)；中间 1→98 才是蓝→紫→绿→红→黄的渐变，
  255（未知）是灰绿。所以"青"= 内切圈边缘，"品红"= 致命障碍，其余都是代价梯度。
- **修正**：判参数一律落到数值，不凭颜色。要确认调色板本身，写个小程序链接
  `librviz_default_plugins` 调用 `makeMapPalette() / makeCostmapPalette() / makeRawPalette()`
  做全量转储即可（本次用的临时程序未入库）。
- **涉及文件**：`/opt/ros/humble/lib/librviz_default_plugins.so`
- **验证方式**：`ros2 topic echo --once /local_costmap/costmap --field data` 的直方图，
  与 RViz 上看到的色带逐段对照
- **日期**：2026-10-24

## 17. ROS 格式 `.pgm` 的第 0 行是图像顶部（y 最大）

- **现象**：拿 `maps/*.pgm` 做离线分析，算出走廊在 y −7.6…−3.8、只有 50 列同时有上下墙；
  而实际走廊在 y −1.2…+1.5，290 列都有墙。整张图被判成上下镜像。
- **根因**：ROS 存图沿用图像行序，**第 0 行是 y 最大那一行**。按
  `y = origin.y + row * resolution` 算，整张图上下翻转。
- **修正**：正确换算是 `y = origin.y + (height - 1 - row) * resolution`。
  写出前用已知点自检：`(0, 0)` 必须落在自由格上。
- **涉及文件**：任何直接解析 `maps/*.pgm` 的离线脚本（本次是临时脚本，未入库）。
  注意**消息和文件行序相反**：`OccupancyGrid` 的 `data` 从 row 0 = `origin.y` 开始，
  和 `.pgm` 正好上下颠倒；`ros_ws/tools/gap_report.py` 读的是 bag 里的消息，
  按 `y = oy + row*res` 是对的，不要据此"修正"它
- **验证方式**：把 `/odometry` 的若干位姿投影到地图上，应全部落在 free 格；
  行序反了会全落在 unknown 或障碍区
- **日期**：2026-10-24

## 18. 静止时 LETHAL 格逐帧换一批（看着像噪点，其实是身份抖动）

- **现象**：车不动，代价地图致命格数稳定（869–901），但每帧落点都换；
  全局相邻帧 LETHAL 的 IoU 只有 0.32、局部 0.42，路径带上的格尤其跳。
- **根因**：`IntensityVoxelLayer::updateBounds` 每周期 `resetMaps()` 重画
  （`pb_nav2_plugins/src/layers/intensity_voxel_layer.cpp:130`），而
  `observation_persistence` 默认 0.0 ⇒ 每张图只投影最新一帧地形云；地形源
  `decayTime: 0.2 s` 导致每帧覆盖不同（84% 的体素只在 ≤20% 帧出现）。
  另有一个陷阱：仅调大 persistence 不够，**代价地图发布频率必须与持久化窗口对齐**，
  否则相邻两张图来自互不重叠的观测窗（全局曾发 1 Hz、pers 0.5 s，等于没加）。
- **修正**：`mark_threshold: 1`；局部 `terrain_map` persistence 1.0 s；
  全局 `terrain_map_ext` persistence 1.5 s；全局 `publish_frequency` 1.0→2.0。
  实测全局 IoU 0.32→0.72、局部 0.42→0.75，路径带 0.77/0.75。完整方法见
  [lethal_flicker_fix.md](lethal_flicker_fix.md)。
- **涉及文件**：`ros_ws/src/sentry_navigation/pb2025_nav_bringup/config/reality/nav2_params.yaml`；
  代码侧只读参考 `pb_nav2_plugins/src/layers/intensity_voxel_layer.cpp:130`
- **验证方式**：静止起栈（`enable_motion:=false`）录 25 s
  `terrain_map|terrain_map_ext|costmap`，算相邻帧 LETHAL 的 `|A∩B|/|A∪B|`，
  要求全局/局部都 ≥0.70；不要只看格数。动态障碍尾迹（≤persistence）需动车另验
- **日期**：2026-10-25

## 19. 巡航中段的「曲率降速带」是地图里的实体隔断，不是控制器抖动

- **现象**：0.9 m/s 配置下，去程约 5–16 s、回程约 58–70 s 各有一段 11–13 s 的降速，
  指令从 0.9 掉到 0.40–0.51 并稳定在那里，看着像曲率估计在抖。**去程回程落在同一段走廊**
  （x ≈ 2.6–5.8），不是随机出现。
- **根因**：`maps/corridor_1029_run1.pgm` 在 **x ≈ 3.7、y = +0.2…−0.4** 有一道
  0.10 m 厚（2 格）、0.6 m 长的实体隔断，把走廊分成北 0.9 m（y +0.3…+1.2）和
  南 0.4 m（y −0.9…−0.5）两条通道。全局路径必须绕墙做 S 弯，用 `curvature_forward_dist
  1.2 / backward_dist 0.6` 量出的曲率达到 **0.86–1.00**（0.4 m 窗口更到 1.4–1.9），
  超过 `curvature_max 0.7`，按 `reduction_ratio_at_high_curvature 0.5` 降到 0.45。
  **这是设计行为，不是缺陷**——以 0.9 m/s 直冲 0.9 m 宽通道才是错的。
- **修正 / 结论**：不要把降速段当抖动去动 `curvature_min` / `curvature_max`。
  调参只能改「过渡方式」，改不了「要不要降速」：
  - `curvature_forward_dist 0.7 → 1.2`、`curvature_backward_dist 0.3 → 0.6`：
    估计变连续，0.9↔0.11 的锯齿消失，改为平滑降到 ~0.42。满速↔半速翻转 26→19 次，
    `|Δv|`/0.2 s 的 p90 0.29→0.23、p99 0.50→0.40，去程 45→35 s。**定稿采用。**
  - `reduction_ratio_at_high_curvature 0.5 → 0.75`：那段提到 ~0.67，全程 74.4→72.8 s，
    但抖动指标回到 p90 0.28 / p99 0.50（接近未改前的水平），故**回退为 0.5**。
- **排查时排除过的两个假设**（都不成立，别再走）：
  速度尖峰与 `/cmd_vel` 角速度相关性仅 0.107；与重规划时刻无关联（落差前 0.15 s 内有
  重规划的比例 38%，随机基线 40%）。
- **涉及文件**：`pb2025_nav_bringup/config/reality/nav2_params.yaml`（`FollowPath` 曲率四项）、
  `maps/corridor_1029_run1.pgm`；只读参考
  `pb_omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp:566-593`
- **验证方式**：逐秒打印 `/cmd_vel` 的 `|v|` 与 **map 系**（不是 odom 系，见 #21）路径曲率。
  **降速段里曲率接近 0，才是真的抖动**；曲率高则是真实弯道。
- **日期**：2026-10-29

## 20. 用「最近邻采样」跨真实数据缺口做分析，会凭空造出不存在的行为

- **现象**：曾据同一个 bag 判定出两条「铁证」——「末端偏航锁死 5.5 s，指令恒为
  `vx=vy=0, w=0.20`」「起步 8 s 只有 0.05 m/s」。据此把 `yaw_goal_tolerance` 改成 6.28，
  结论全错，已回退。
- **根因**：
  1. `/cmd_vel` 在没有导航目标时**根本不发布**。整段 100 s 的 bag 里它只有一个
     **10.5 s 的缺口**，且比 `/odometry` 晚 7.7 s 才开始。
  2. 用 `bisect` 找最近邻时，缺口两侧的样本会被拉到缺口内部的任意时刻，于是缺口被
     「填满」成一条看似连续的低速/零速序列——凭空生成的行为。
  3. 每个 topic 的首样本时间不同，用全局 `t0` 做时间轴会整体错位。
- **修正**：每个 topic **单独**取 `t0`；下任何时序结论前先打印该 topic 的采样间隔分位数
  和最大缺口；间隔 > 1 s 的区间标记为「无数据」，既不插值也不参与判读。
  `ros_ws/tools/replay/an_run.py` 已按 topic 单独排序，缺口处直接判为无数据。
- **涉及文件**：`ros_ws/tools/replay/an_run.py`（`load()` / `nearest()`）
- **验证方式**：`dead_cmd_s` 这类指标算完之后，用 `ros2 topic hz` 的原始分布复核一遍；
  若某段「异常」正好横跨一个缺口，先怀疑分析而不是控制器。
- **日期**：2026-10-29

## 21. 在 odom 系里比横向误差，会被 LIO 漂移放大成米级

- **现象**：拿 `/odometry` 的位姿直接与 `/plan` 求最近距离，得到 **0.4–1.1 m** 的横向误差，
  看上去跟踪完全失效。
- **根因**：`/plan` 在 **map 系**，`/odometry` 在 **odom 系**，两者之间隔着 LIO 位姿，
  一趟 74 s 会漂 0.4–1.1 m。误差量里绝大部分是漂移，不是跟踪误差。
- **修正**：用 `/tf` 的 `map → odom` 把 odom 位姿先投回 map 系再求距离。同一段数据的
  真实横向误差是 **p50 0.021 m / p90 0.058 m**。
- **涉及文件**：`ros_ws/tools/replay/an_run.py`（`to_map()`）
- **验证方式**：同一段 bag 分别按两种坐标系算，差值应等于 LIO 漂移量级；若两者接近，
  说明这段根本没漂，可用 odom 系省事。
- **日期**：2026-10-29

## 22. `map_server` 起栈后停在 `unconfigured`，日志里只有「Configuring map_server」

- **现象**：`lifecycle_manager` 打印 `Configuring map_server` 之后**再无输出**，
  `map_server` 自己连一行 `Configuring` 都没打印，`/map` 没有发布者，Nav2 整体起不来。
  同一份配置在另一次启动里 1.6 ms 就配置完成——是**间歇性**的。
- **根因**：未查明（现象可复现但非必现；怀疑是启动期资源竞争让 configure 回调没被调起）。
  已知和地图文件本身无关——同一张图重启后正常。
- **修正**：不用重启整栈，手动推生命周期即可：
  ```bash
  ros2 lifecycle set /map_server configure
  ros2 lifecycle set /map_server activate
  ```
  仍失败就重起整栈（实测每次重起都能好）。
- **涉及文件**：无代码改动
- **验证方式**：**发目标前必须两项都满足**——
  `ros2 lifecycle get /map_server` 应为 `active [3]`，`ros2 topic info /map` 发布者数应为 1。
- **日期**：2026-10-29

## 23. A→B 掉头时平移停住约 4 s（`use_rotate_to_heading`）

- **现象**：每趟任务在 A→B 掉头处，平移停止约 4 s（实测 R0 4.3 s、R1' 3.9 s、R2 4.0 s），
  期间 `vx=vy=0`、`wz` 从 1.1 降到 0.6，之后才重新起步。这是全程**唯一**的平移停顿
  （`/cmd_vel` 全程 100 Hz 无断流，实际速度无 <0.05 m/s 的短暂停顿）。
- **根因**：`use_rotate_to_heading: true` + 阈值 0.7 rad。偏差超过 40° 时控制器把
  `lin_dist` 置 0，**先原地转完再走**（`omni_pid_pursuit_controller.cpp:264-268`）。
  掉头时偏差接近 180°，所以必然触发。
- **未改的原因（2026-10-29 决定保持现状）**：本车是**圆形车体**（`body_radius 0.26`，
  costmap 用 `robot_radius: 0.26`）且 MID360 为 360°，车头朝向既不影响通行也不影响感知，
  理论上可以照母工程用 `enable_rotation: false` 彻底不转。但那需要**连带**把
  `yaw_goal_tolerance` 放到 6.28（母工程即如此），否则 `SimpleGoalChecker` 要求终点朝向
  ±17°、车又转不了，目标会判失败。本次不改，留作后续选项。
- **另一个必须知道的区别**：母工程「纯平移」的前提是**雷达装在云台 yaw 轴上**，
  `fake_vel_transform` 用 `gimbal_yaw → gimbal_yaw_fake` 把导航坐标系与车体转动解耦。
  **本车没有云台**（`use_gimbal: false`，`gimbal_yaw` 是 `base_link` 的固定子帧，
  `front_mid360` 挂在它下面随车体转），所以那套机制在这里不存在，不能直接照搬。
- **涉及文件**：`pb2025_nav_bringup/config/reality/nav2_params.yaml`（`enable_rotation`、
  `use_rotate_to_heading`、`general_goal_checker.yaw_goal_tolerance`）
- **验证方式**：`/cmd_vel` 里筛 `|v| < 0.05 且 |wz| > 0.3` 的连续段，就是原地转向段；
  当前应只有 1 段、约 4 s。
- **日期**：2026-10-29

## 24. 回放台的总时长是伪指标，用它比快慢会得到相反的结论

- **现象**：2026-10-30 五个变体跑同一个 `scenario_c`，总时长 201.6–206.1 s（差 2.2%），
  但"移动中位速"差 89%（0.249 → 0.470 m/s）。按时长排序会得出"提速没用"的错误结论。
- **根因**：`goal_schedule.py` 按**录包里的时刻**发目标（`t_origin + goal["t"]`，
  [goal_schedule.py:70-75](../ros_ws/tools/replay/goal_schedule.py)），不等上一个目标结束。
  总时长由时刻表决定，不由车跑得多快决定。
- **修正**：改用**每目标到点滞后** `lag_k = (s_k − s_0) − (t_k − t_0)`，
  `s_k` 取 bt_navigator 的 `Goal succeeded` 时刻。时长只做一致性检查。
  详见 [`nav2_tuning.md`](nav2_tuning.md) 第六节。
- **涉及文件**：`ros_ws/tools/replay/goal_schedule.py`、`ros_ws/tools/replay/metrics.py`
- **验证方式**：同一变体跑两次，时长差 <2% 而滞后会随速度明显变化。
- **日期**：2026-10-30

## 25. `cmd_accel_p95` 被重复帧放大 1.6 倍，会造出「超过底盘能力」的假警报

- **现象**：某变体指令加速度 p95 报 4.42 m/s²，高于底盘斜坡 2.5，看着像"发得比车做得快"；
  但该变体的 `max_accel` 设的就是 2.5，理论上不可能超。
- **根因**：`metrics.py::cmd_accel()` 对相邻两帧直接求导。`/cmd_vel` 是 20 Hz 内容被
  96 Hz 的发布架发出来的，实测 **83% 相邻帧完全相同**，真正的变化落在 0.05 s 边界上，
  却拿 0.01 s 的帧间隔去除 ⇒ 系统性放大 **约 1.6 倍**。
- **修正**：判断"车跟不跟得动"要先去重再求导（只保留真正变化的帧），
  对照关系才对得上（max_accel 3.0 → 去重 p95 3.04）。去重代码见
  [`nav2_tuning.md`](nav2_tuning.md) 第六节。**`metrics.py` 本身未改**，改它会打断已跑完的
  变体之间的可比性。
- **涉及文件**：`ros_ws/tools/replay/metrics.py`
- **验证方式**：同一段包，原始 p95 / 去重 p95 的比值应稳定在 1.6 附近。
- **日期**：2026-10-30

## 26. 回放台的 plant 模型没有最高速饱和，测不出真车能跑多快

- **现象**：台架上把 `v_linear_max` 抬到 1.5，移动中位速就跟着涨 70%，看起来车能跑多快
  取决于参数。
- **根因**：`plant_measured.json` 是从实录里拟合的线性模型（`v_gain 0.55`、
  `deadband_v 0.1406`、一阶滞后），**没有饱和项**，指令给多快它就按比例放大到多快。
  真车还叠着速度环死区与增益压制。
- **修正**：台架只用来定"指令该开到多少"，不能用来断言"车能跑多快"。
  抬 `v_linear_max` 时必须同步 `usb_bridge.launch.py` 的 `max_vx`，否则被更紧的一处
  静默截断（见 [`parameters.md`](parameters.md) §4.2）；真车速只能上车量 `/odometry`。
- **涉及文件**：`ros_ws/tools/replay/plant_measured.json`、
  `ros_ws/src/sentry_hardware/usb_bridge/launch/usb_bridge.launch.py`
- **验证方式**：台架上 `max_velocity` 设 1.5 与 2.0，移动中位速应严格按比例变化（说明无饱和）；
  真车上同样改，`/odometry` 的速度会撞到一个平台。
- **日期**：2026-10-30
