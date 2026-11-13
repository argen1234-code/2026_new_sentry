# 紧耦合 LiDAR-IMU-轮速里程计：资料与取舍

> 状态：**调研记录**，2026-11-13。当前本工程用的是**松耦合**（`wheel_odom_fusion` 在位姿层
> 融合 50 Hz 轮速与 20 Hz LIO），本文回答"如果要上紧耦合，有哪些现成东西、代价是什么"。
> 结论放最后。

## 1. 中科大 RoboWalker 的 Batch-LIWO（我们最接近的参考）

来源：本仓库 [`ustc_robowalker_2025.md`](ustc_robowalker_2025.md) §5（RM2025 技术报告转述）。

- **定位**：基于 **Point-LIO** 改进，输出最高 **100 Hz**，吃轮速。
- **状态量**：SO(3) + ℝ²¹（位置、速度、陀螺/加计零偏、**重力**、**IMU 角速度**、**IMU 加速度**）；
  把角速度也写进状态，便于在 IMU 饱和（MID360 内置 IMU 约 4 g）时放弃 IMU 更新、仅靠点云维持。
- **Batch 更新**：把约 1 ms 时间窗内的点变换到同一时间戳，逐点算残差后**拼接残差与测量矩阵
  一次 Update**。相比 Point-LIO 的逐点更新：更新次数大幅减少、可并行、每点置信度一致；
  短窗内 MID360 的 4 条线扫到多个平面，能有效约束自由度。
- **轮速融合（重点）**：残差定义在 **IMU 坐标系**，协方差 = 期望残差 × 基准噪声。
  期望残差 =「用轮速+舵角最小二乘得到无打滑底盘速度，再投影回四个轮」与实际轮速的等权平方和。
  设计原则是**保守**：允许把某个观测源短暂误判为失效（还有 IMU 和点云兜底），
  但不能容忍它给出错误的速度更新（他们国赛遇到过"底盘掉电反馈错误高置信度轮速"）。
- **全局定位**：故意不做独立模块——预建点云 + 初始位姿直接喂 LIO，摆放容差约 20 cm / 30°。
- **开源状态（2026-11-13 核实）**：GitHub `ZhangHaopeng-Dino/Batch-LIWO` 仓库存在，
  但本仓库的本地副本（`reference/Batch-LIWO`，2026-11-12 clone）**只有 README + LICENSE**，
  单条 `Initial commit`；RM2025 报告也标注"尚未开源"。**目前拿不到实现**。

## 2. 学术论文

| 论文 | 会议/期刊 | 耦合方式 | 要点 |
|---|---|---|---|
| **LIWO: Lidar-Inertial-Wheel Odometry** (Yuan et al.) | IROS 2023, arXiv 2302.14298 | 紧耦合（BA / 因子图） | 把轮速编码器作为**速度观测**约束速度变量；6 轴 IMU 即可，不依赖 AHRS；NCLT/KAIST 数据集验证 |
| **LIO-Vehicle** (Xiao et al.) | IEEE RA-L 2022 | 紧耦合（因子图） | 轮速+转角建 2-DOF 车辆动力学模型，做**动力学预积分因子**，与 IMU 预积分、LiDAR 里程计一起滑动窗口优化；专治隧道/无特征区 |
| **Okawara et al.** | IEEE Access 2024, arXiv 2404.02515 | 紧耦合（因子图） | 面向**滑移转向**机器人，"full linear wheel odometry factor"既是运动约束又做**在线运动学标定**；在线估计轮速协方差；明确针对退化场景 |
| **Okawara et al.**（后续） | RAS 2025, arXiv 2407.08907 | 紧耦合 | 用**神经网络运动学模型**在线学习，处理打滑/非线性 |
| **Tightly Coupled Lidar-Wheel-IMU Odometry** (Zhang et al.) | ACM SIUSA 2024 | 紧耦合（IESKF） | 把轮式里程计作为约束加进 IMU 前向传播，抑制发散，对 IMU 噪声与打滑鲁棒 |
| **LIW-RO** | IEEE 2025 | 紧耦合（因子图） | 轮速作因子 + 轮式 NHC（非完整约束），动态关键帧选择 |

共同点：轮速要么作为**速度观测因子**，要么经**运动学/动力学预积分因子**进入滤波器/因子图；
主要收益都在 **LiDAR 退化场景**（隧道、长走廊）抑制速度漂移，以及在线标定打滑模型。

## 3. 可拿到的开源实现

| 仓库 | 底座 | 说明 |
|---|---|---|
| `TakuOkawara/full_linear_wheel_odometry_factor` | 自研因子图 | 对应上面 Okawara IEEE Access 2024，含在线运动学标定与协方差估计；**开源**、ROS 1/2 |
| `zhh2005757/FAST-LIO-Multi-Sensor-Fusion` | FAST-LIO / IKFOM | 在 FAST-LIO 上加 GNSS 与轮速测量；结构最接近"给 FAST-LIO 加 wheel factor" |
| LIWO（Yuan et al.） | LIO-SAM 风格 | 论文配套代码已放出，速度观测嵌入 BA |
| `kousheekc/FAST_LIO_GPS` | FAST-LIO | 加 GPS 因子与回环位姿图；可作"加新测量块"的结构参考 |
| `ZhangHaopeng-Dino/Batch-LIWO` | Point-LIO | **只有 README**（截至 2026-11-13） |
| `hku-mars/ROG-Map` | — | USTC 感知前端的上游 |

实现层面的通用做法（来自 FAST-LIO2 的源码分析）：在 IKFOM 的观测模型
（`h_share_model` / `ObsModel`）里追加轮速残差 `v_body ≈ r·ω_wheel` 与
零侧向速度约束（NHC），并把轮速噪声/协方差按退化度量动态放大。

## 4. 对我们这条链的适用性

我们的底座是 **Point-LIO**，与 USTC 同源，所以"紧耦合"有两条路：

1. **改 Point-LIO（对齐 USTC）**：在它的 ESKF 里加轮速残差 + Batch 更新。
   优点是同源、能直接复现他们的设计；代价是 fork 一个 20 万点/秒的滤波器，
   而且我们现有硬约束是"`terrain_analysis` 的位姿与点云必须同源"，改动会牵到这条链。
2. **换底座（FAST-LIO 系）**：用 `FAST-LIO-Multi-Sensor-Fusion` 或
   `full_linear_wheel_odometry_factor` 做参考，把轮速因子加进 IKFOM 观测模型。
   优点是社区实现多、文档全；代价是换掉现有的去畸变点云输出与调好的参数。

**当前的判断（未变）**：先不上紧耦合。触发条件应是**实测到退化场景定位崩溃**
（隧道、长走廊、强遮挡），而不是"参数还能再优化"。在那之前，松耦合的收益
（融合 vs LIO p95 17 mm、路径长度更接近轮式真值）已经够用；
且轮速可信度靠 `alpha` 门控 + 超时降级兜底（见 [`wheel_odometry.md`](wheel_odometry.md)）。

真要做时，第一步不是改代码，而是**补一个退化场景的实车录包**，把问题量化出来。

## 5. 参考链接

- Batch-LIWO（README）：<https://github.com/ZhangHaopeng-Dino/Batch-LIWO>
- LIWO 论文：<https://arxiv.org/abs/2302.14298>
- LIO-Vehicle：<https://ieeexplore.ieee.org/document/9609604>
- Okawara（在线标定）：<https://arxiv.org/abs/2404.02515>
- 开源实现：<https://github.com/TakuOkawara/full_linear_wheel_odometry_factor>、
  <https://github.com/zhh2005757/FAST-LIO-Multi-Sensor-Fusion>
- RM2025 RoboWalker 开源报告：<https://bbs.robomaster.com/article/803727>
