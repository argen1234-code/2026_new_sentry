# 定位与建图规划

> 状态：**规划，尚未执行**。本文记录 2026-10-23 讨论后的架构结论与执行顺序。
> 文中「已核实」= 读过代码/实测数据；「推断」= 未验证的判断，执行前需复核。

## 1. 结论

1. **2D 栅格的职责只有两个**：代价地图静态层、全局规划参考。它回答「哪里有墙」，不回答「我在哪」，也不能用来评判定位精度。
2. **重定位不能完全交给轮式里程计**。里程计是相对量，开机时不知道自己在场地哪里，必须有一个锚把它固定到地图上。
3. **锚用 GICP 对 3D 先验**（现状），不换成 2D 匹配。原因是输入太弱（见 §6）。
4. **未来加入 2 轮 + IMU 融合里程计后，只替换 `odom→base_footprint` 这一段**，`map→odom` 仍由 GICP 提供。分工不变，只是 GICP 需要修正的残差变小。
5. **不启用 `use_scan_matching` / `do_loop_closing`**，现在不启用，传感器到位后更不该启用。

## 2. 角色分工

| 对象 | 回答的问题 | 唯一来源 | 说明 |
|---|---|---|---|
| `odom→base_footprint` | 相对运动 | Point-LIO（现状）/ 轮式+IMU 里程计（未来） | 高频，局部无漂移或小漂移 |
| `map→odom` | 我在地图的哪里 | `small_gicp_relocalization` | 2 Hz，对 3D 先验做配准 |
| 2D 栅格 `/map` | 哪里有墙 | `slam_toolbox`（建图）/ `map_server`（导航） | 只做代价地图静态层与全局规划 |
| 漂移量 | 我错了几厘米 | 回环回归测试 / 外部真值 | **栅格测不出来** |

> 常见误解：从栅格歪不歪反推位姿精度。栅格是位姿精度的**结果**，不是尺子。

## 3. 现状链路（已核实）

```text
point_lio(点云 + IMU) → aft_mapped_to_init + cloud_registered
        ↓
loam_interface        → lidar_odometry + registered_scan
        ↓
sensor_scan_generation → TF odom→base_footprint + /odometry
        ↓
terrain_analysis / terrain_analysis_ext ← "lidar_odometry"
        ↓
pointcloud_to_laserscan → /obstacle_scan → slam_toolbox → /map
                                     ↘ Nav2（GICP 发 map→odom）
```

关键约束：**terrain_analysis 的位姿与点云必须同源**。现在两者都出自 Point-LIO 的同一个状态。

证据位置：

- `loam_interface` 重发位姿与点云：`sentry_perception/loam_interface/src/loam_interface.cpp`
- TF `odom→base_footprint`：`sentry_perception/sensor_scan_generation/src/sensor_scan_generation.cpp`
- terrain_analysis 订阅 `lidar_odometry`：`sentry_perception/terrain_analysis/src/terrainAnalysis.cpp`
- GICP 发 `map→odom`：`sentry_localization/small_gicp_relocalization/src/small_gicp_relocalization.cpp`

## 4. 未来架构（传感器到位后）

```text
2 轮 + IMU 里程计 ──→ odom→base_footprint ──→ GICP(map→odom) ──→ Nav2
Point-LIO         ──→ cloud_registered_body ──→ 去畸变点云
```

要点：

- **点云必须用 `cloud_registered_body`**（frame_id = `body`，逐点运动补偿后的 body 系点云），不能用 `cloud_registered`。后者绑在 Point-LIO 自己的 `camera_init` 轨迹上，会与外部位姿越走越偏。
- Point-LIO 保留，但降级为「只提供去畸变点云」。
- GICP 保留：外部里程计是局部精度，不是全局定位。

## 5. 三条约定（现在就可以定）

1. **地图原点 = 车的标准起始位姿**。开机 `map→odom = 单位阵`，`init_pose: [0,0,0,0,0,0]` 永远有效。
   现状 `lab.yaml` 的 origin 是 `[-6.37, -4.06]`，不满足此约定，正式建图时应按约定重做。
2. **固定起始位姿只做初值，GICP 必须常开**。车被撞、被推、被抬后纯里程计定位会静默失准且无法自恢复。
3. **`slam_toolbox` 建图后不参与导航**（现状如此），`use_scan_matching` / `do_loop_closing` 恒为 `false`。

## 6. 为什么不采用 2D 定位（Cartographer / slam_toolbox localization 模式）

`slam_toolbox` 确实有 `mode: localization`，但本工程不采用：

| 问题 | 依据 |
|---|---|
| 匹配输入太弱 | `pointcloud_to_laserscan` 合成扫描实测中位 191 点 / 720 线（26%），2D 相关匹配精度为厘米级，低于未来的毫米级里程计 |
| 资产要重做 | localization 模式需要 slam_toolbox 序列化位姿图（`.posegraph`），现有流程只产出 `.pgm` + `.pcd` |
| 丢掉 3D 信息 | 整条标定链（点云地面标定、地形分析、GICP）围绕 3D 先验建立 |

推论：**当里程计精度高于扫描匹配精度时，扫描匹配与回环只会引入噪声。** 因此即使启用 2D 回环，也需要同时把修正量作用到 3D PCD —— 而回环修正是非刚性的，单一矩阵不成立，正确做法需要保存关键帧位姿并离线重投影。该工作量在本规划中被明确排除。

## 7. 执行顺序（推荐）

| 顺序 | 事项 | 为什么现在做 | 依赖 |
|---|---|---|---|
| 1 | 修底盘速度环死区 | 146 s 只走 7.93 m，一切建图验证都跑不完；与传感器无关 | 需动 `stm32_ws/` 固件，单独审批 + 烧录 |
| 2 | 跑一次完整基线建图并录包 | 得到漂移数据；同时留下「旧地图 + 原始数据」作为新传感器的对照基准 | 需要 1 完成 |
| 3 | 2D 扫描合成提质 | 单格散点占连通块 49%，与位姿源无关，换传感器也不会自动变好 | 可并行 |

基线录包应包含原始数据，便于将来离线重放对比：

```bash
ros2 bag record -o ~/bags/baseline_$(date +%H%M) \
    /livox/lidar /livox/imu /aft_mapped_to_init /lidar_odometry \
    /obstacle_scan /map /tf /tf_static
```

## 8. 明确不做

- 不实现 2D 回环、不做栅格修正 3D PCD（做法 A/B 均排除，见 §6）。
- 不围绕 Point-LIO 的漂移精度做调参 —— 位姿源将来会被替换。
- 不在传感器到场前改 `loam_interface` 等接口（坐标系、频率、消息类型均未知，改就是猜）。

## 9. 待确认（传感器到场前必须问清）

1. 发布的话题与消息类型（`nav_msgs/Odometry` 还是自定义）。
2. 位姿原点是车体中心、`base_link` 还是轮系；航向相对哪根轴。
   > 底盘轮系坐标系相对 `base_footprint` 转过 -45°（见 `cmd_vel_to_unified` 的 `frame_yaw`），
   > 位姿进栈前必须完成这个转换，否则整张图旋转 45°。
3. 输出频率（Nav2 与 terrain_analysis 需要 ≥20–50 Hz 的 TF）。
4. 精度是在直行测的，还是含横移、原地转都测过。2 轮在全向底盘上只能直接测滚动方向，横向依赖 IMU。

## 10. 与当前阶段的关系

当前阶段（`terrain_analysis_ext.useSorting` / `decayTime` 调整）属于 §7 第 3 项的感知侧改动，与位姿源无关，不受本规划影响。该阶段的实测数据见会话记录，尚未做成 A/B 对照。
