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

## 3. `/livox/imu` 订阅读不到数据（QoS / 发现延迟）

- **现象**：新写的订阅节点在 `ros2 topic echo` 正常的情况下读不到 IMU，或启动头几秒是 “NO DATA”。
- **根因**：`/livox/imu` 的 publisher 是 RELIABLE，订阅端用 `qos_profile_sensor_data`
  （BEST_EFFORT）会不匹配；另外节点发现需要 2–3 s。
- **修正**：订阅端使用 RELIABLE QoS；启动后留足 2–3 s 再判读。
- **涉及文件**：任何订阅 `/livox/imu` 的节点
- **验证方式**：`ros2 topic info /livox/imu --verbose` 看 QoS 是否匹配
- **日期**：2026-10-23
