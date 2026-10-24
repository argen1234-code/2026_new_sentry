# 地面与遮挡区标定（换底盘必做）

> **一句话**：新底盘的 URDF 可能是对的，但装配误差一定存在。雷达离地高度、安装姿态、
> 自遮挡范围这三件事**必须实测**，不能从 URDF 或 CAD 里抄。
>
> 本文的方法配套脚本：[`ros_ws/tools/calib_ground.py`](../ros_ws/tools/calib_ground.py)。
> 换底盘的整体流程见 [`chassis_swap_handover.md`](chassis_swap_handover.md)。

---

## 0. 为什么必须实测

2026-10-24 在同一台车上的三个数字：

| 来源 | 雷达离地高度 | 说明 |
|---|---|---|
| URDF `lidar_z`（2026-10-23 点云拟合） | 0.120 m | **错**，偏小 7.6 cm |
| 本次静止标定（纯地面格拟合） | **0.196 m** | 1118 个纯地面格，残差 6.9 mm |
| 卷尺（量到外壳顶） | 0.200 m | 与点云一致 |

同一次标定还测出雷达**俯仰 2.3°**（roll 0°）。这两项在 URDF 里都体现不出来。
结论：**URDF 的 z 与姿态不可信，必须实测**；x/y 的安装位置（2D 平面几何）可以先用 URDF。

---

## 1. 要测的五个量

| # | 量 | 决定哪个参数 | 判据 |
|---|---|---|---|
| 1 | 遮挡区 / 自返回点的半径上限 | `point_lio` 的 `blind` | `blind` > 该半径，否则自返回进图 |
| 2 | 雷达参考系到地面的距离 `h` | URDF `lidar_z` | 拟合残差 < 1 cm，随距离不漂移 |
| 3 | 地面安装倾角 pitch/roll | URDF `joint_lidar` 的 rpy（通常忽略） | < 1° 可忽略 |
| 4 | 地面在 `base_footprint` 的高度 `g` | `pointcloud_to_laserscan` 的 `min_height` | \|g\| < 2 cm 为理想 |
| 5 | 地面相对 `vehicleZ` 的高度 | `terrainUnderVehicle`、`minRelZ`、`lowerBoundZ` | — |

**四号量最容易被忽略**：`min_height` 是相对 `base_footprint` 的，而 `base_footprint`
在这个链路里并不等于"地面"。见 §4.1。

---

## 2. 采集（车不动，零风险）

要求：车**着地、静止**，场景里最好同时有「中间平整地面」和「两侧障碍物」——两侧障碍物
能暴露拟合被污染的问题（见 §6），平地能保证纯地面格足够多。

```bash
cd ~/2026_new_sentry/ros_ws
./scripts/start_sentry.sh slam:=true enable_motion:=false visualization:=headless
```

起栈后先确认频率，任一项明显偏低就停：

```bash
ros2 topic hz /livox/lidar/pointcloud    # 期望 ~20 Hz
ros2 topic hz /terrain_map_ext           # 期望 10–20 Hz
```

录 90 s（约 900 MB）：

```bash
ros2 bag record -o ~/bags/calib_static_$(date +%m%d_%H%M) \
    /livox/lidar/pointcloud /registered_scan /lidar_odometry \
    /terrain_map /terrain_map_ext /obstacle_scan /map /tf /tf_static
```

Ctrl-C 停录 → `./tools/stop_ros_stack.sh` 停栈。

> 必须录 `/livox/lidar/pointcloud`（**未经 `blind` 过滤的原始点云**），不能用
> `/registered_scan` 代替——后者已下采样且被 `blind` 滤过，单帧内点太少，拟合会偏 3–8 cm。

---

## 3. 分析

```bash
python3 ~/2026_new_sentry/ros_ws/tools/calib_ground.py ~/bags/calib_static_XXXX_XXXX
```

脚本输出四段，逐段读法如下。

### 3.1 遮挡区（输出段 [1]）

```
r<0.5 m 的点 25369 个（1.5%），半径 p5/p50/p95 = 0.102/0.131/0.210 m
z    p5/p50/p95 = -0.005/+0.003/+0.044 m
```

这是**支架/外壳自返回**形成的环。要点：

- `blind` 必须大于这里的 `p95`（现为 0.210 m，配置 `blind: 0.5` ✅ 有余量）。
- `blind` 是 3D 距离，所以**车体自返回永远进不了图**——判断遮挡不需要 URDF 的几何，
  只需要这个环的半径。
- `terrain_analysis` 的 `clearingDis`（0.35）与 `blind`（0.5）**功能重复**，都拦不住
  0.2 m 的环之外的东西。保留即可，不要指望它做遮挡判断。

**新底盘如果雷达装得更低或支架更大，这个环会变**——重测，别沿用 0.5。

### 3.2 地面高度与倾角（输出段 [2]）

主方法是「纯地面格」：

```
[主方法 纯地面格]   原点处 z0=-0.1967 m  倾角=2.29° (pitch -2.29°, roll -0.00°)
                    内点 795/1370 格  rms=6.9 mm
    按距离残差: r1.2-2.0 -0.0 cm | r2.0-2.5 +0.2 | r2.5-3.0 +0.1 | r3.0-3.5 -0.5
```

原理：按 5 cm 网格切片，**只保留「点数 ≥5 且格内 z 极差 <3 cm」的格子**。障碍物的格子
同时含底边和侧面/顶面，极差必然大，会被自动剔除；地板格则几乎平的。剩下的点就是地面本体。

读法：

- `z0` = 雷达参考系原点所在水平位置的地面高度 → **就是 URDF 的 `lidar_z`**（取绝对值）。
- `pitch/roll` = 安装姿态误差。**本次 2.29°/0°，属装配误差，按项目约定不写进 URDF**
  （写了会让 `base_footprint` 变水平，与车体实际姿态不符）。
- 按距离残差应全部在 ±1 cm 内且**不单调漂移**；若单调漂移，说明平面没拟合好。

### 3.3 odom 系交叉验证（输出段 [3]）

```
registered_scan 地面: d=0.008 m  tilt=0.60°
odom->base_footprint z 中位: 0.0157 m
地面在 base_footprint 的高度: -0.0076 m   (判据 |x|<0.02)
lidar_odometry z 中位: 0.1324 m
地面相对 vehicleZ: -0.1243 m
```

- `tilt 0.60°` 说明**重力对齐成立**（Point-LIO 把世界 z 对到了重力方向，地面接近水平）。
  这个值 > 2° 就要怀疑 IMU 或 `gravity` 配置。
- **`地面在 base_footprint 的高度`** 是给 `min_height` 用的（见 §4.1）。
- **`地面相对 vehicleZ`** 是 `terrainUnderVehicle` / `minRelZ` / `lowerBoundZ` 的依据。

### 3.4 地面误报 / 幻影圈（输出段 [4]）

```
地面点占输出 3.2%；经 intensity>=0.10 闸门后误报 0.43%
误标点距离分布: 4.0-4.5 m: 2723 | 4.5-5.0: 884 | 5.0-6.0: 392 | 6.0-8.0: 639
误标 5cm 栅格数: 111
```

看误标点的**距离分布**：

- 全部集中在某个固定半径外的一圈 → 远场地形分支的边界伪影（`localTerrainMapRadius`）。
- 均匀散布 → 地面拟合参数（`quantileZ`）偏置。
- 靠近车体 → 自返回或遮挡区问题。

---

## 4. 从测量值换算参数

### 4.1 `lidar_z` 与 `min_height` 必须一起改

链路里 `lidar_z` 身兼两职：既描述几何，又决定**点云相对 `base_footprint` 的抬升量**。
原因是 `loam_interface` 把 Point-LIO 的位姿（**那是 IMU 位姿**）当成雷达位姿，再叠上
URDF 的 `base→lidar` 偏移；而 `sensor_scan_generation` 里同一个偏移被抵消了。净效果：

```
地面在 base_footprint 的高度 g ≈ (lidar_z − h) + extrinsic_T_z + 残差
```

（`extrinsic_T_z` 当前 0.044；残差来自 LIO 世界与原始点云的不一致，2026-10-24 实测约 −2.4 cm。）

**操作顺序（必须按这个顺序，不能一步到位）**：

1. 把 `lidar_z` 改成实测的 `h`；
2. 采一包，跑脚本读段 [3] 的 `g`；
3. 把 `min_height` 改成 `g + 0.08`，`max_height` 改成 `g + 1.00`；
4. 再采一包确认 `g` 不变（`lidar_z` 只影响点云，不影响 `base_footprint`）。

> **为什么不能直接把 `min_height` 当成"离地 8 cm"**：`min_height` 相对的是
> `base_footprint`，而 `base_footprint` 在这个链路里比真实地面低 `h − lidar_z`。

2026-10-24 的取值：`lidar_z 0.196`、`min_height 0.15`、`max_height 1.07`
（改动前是 0.12 / 0.08 / 1.0）。

> **后续修订（commit `f890f77` → `56d562c`）**：`lidar_z` 最终填 **0.200**（卷尺复核值），
> 高度带最终是 **`0.16 / 0.50`**——上沿按 Humble 的导航高度闸门收紧，与代价地图
> `min/max_obstacle_height` 对齐。以 `nav2_params.yaml` 为准，见
> [`parameters.md`](parameters.md) §2。

### 4.2 其余高度类参数

| 参数 | 依据 | 2026-10-24 实测 |
|---|---|---|
| `terrainUnderVehicle` | 地面相对 `vehicleZ` | −0.124 → 填 −0.12（**与 `lidar_z` 无关**，两者同步平移） |
| `terrain_analysis.minRelZ` | ≤ (地面相对 vehicleZ) − 0.2 余量 | −0.35 |
| `terrain_analysis_ext.lowerBoundZ` | 同上逻辑 | −0.6 |
| `terrain_analysis.maxRelZ` / `upperBoundZ` | 车体与需检测障碍的高度上限 | 0.8 / 0.8 |
| `point_lio.blind` | > 遮挡环 p95 | 0.5 |

### 4.3 地面剔除的两个折中参数

`quantileZ`（逐体素地面估计的分位点）与代价地图的 `min_obstacle_intensity`（按 `disZ`
闸门）共同决定「地面误报」与「低矮障碍漏检」的平衡。2026-10-24 在同一份数据上实测：

| `quantileZ` | 地面估计偏差 | 低矮障碍(10–30 cm)点保留 | 地面误报 |
|---|---|---|---|
| 0.05 | −3.0 cm | 65.9% | 20.5% |
| 0.10 | −2.1 cm | 61.2% | 17.1% |
| **0.20（现值）** | −1.1 cm | 52.7% | 9.9% |
| 0.30 | −0.4 cm | 44.6% | 6.8% |

| `min_obstacle_intensity` | 低矮障碍格被标记 | 地面格误标 |
|---|---|---|
| 0.05 | 89.9% | 15.3% |
| 0.07 | 85.1% | 13.1% |
| **0.10（现值）** | 78.4% | 9.6% |
| 0.15 | 61.2% | 6.0% |

没有免费午餐：降 `quantileZ` 或降阈值都会同时抬高地面误报。换底盘后重跑这张表再定。

---

## 5. 验收判据

| 项 | 判据 | 2026-10-24 |
|---|---|---|
| 高度测量可靠 | 纯地面格拟合残差 < 1 cm，按距离不漂移 | rms 6.9 mm ✅ |
| 地图水平（重力对齐） | odom 系地面倾角 < 1° | 0.60° ✅ |
| `base_footprint` 落点 | \|地面在 base_footprint 的高度\| < 2 cm | −0.008 m ✅ |
| 地面不被当成障碍 | `/terrain_map_ext` 经 intensity 闸门后误报 < 0.5% | 0.43% ✅ |
| 遮挡区 | `blind` > 自返回环 p95 | 0.5 > 0.21 ✅ |
| 重复性 | 分帧拟合 p5–p95 跨度 < 1 cm | 9.6 mm ✅（勉强） |

---

## 6. 方法学踩坑（2026-10-24 实际踩过）

同一个 bag、同一时刻，不同拟合方法给出的答案能差 **8 cm**：

| 方法 | `z0` | 倾角 | 问题 |
|---|---|---|---|
| 宽窗口自由 RANSAC（1.2–5.5 m） | 0.116 m | 0.65° | 被两侧障碍物拉偏，且会"桥接"地面与远处障碍顶 |
| 近环带自由 RANSAC（1.8–3.0 m） | 0.110 m | 0.21° | 同样被障碍物污染，内点率仅 37% |
| 逐 xy 格最低点包络 | 0.154 m | 1.50° | 比真值偏低：最低点受噪声下偏，且混合了无障碍区域的格子 |
| **纯地面格（主方法）** | **0.1967 m** | **2.29°** | 无（残差 6.9 mm，按距离不漂移） |

三条判伪经验：

1. **看随距离的单调性**。纯地面格的 z 随距离单调上升 4.1 cm/m → 这是**真实倾角**；
   平面对应的是「雷达俯仰 2.3°」，不是「地面倾斜」（odom 系地面只有 0.6°，说明地面本身是平的）。
2. **看远场是否升到 0 以上**。真地面在雷达系永远在 z<0；若拟合平面在 4 m 外跑到 z>0，
   说明它爬上了障碍物顶部，是桥接平面，必须弃用。
3. **采样量影响结论**。同一 bag 用 1/10 帧和全帧，RANSAC 会选出不同平面。所以
   **结论必须跨方法一致才算数**；只跑一种方法就下结论是本项目已经犯过的错。

另外：**不要用单帧拟合**。单帧只有约 400 个内点，拟合跨度达 8.5 cm。
分帧只用来评估**重复性**（相对聚合平面的偏差），不能用来定值。

---

## 7. 记录模板

换底盘后把这张表填进换车记录：

```
日期：
底盘 / 雷达安装变动：
bag 路径：

[1] 遮挡环      半径 p95 = ____ m   → blind = ____
[2] 纯地面格    z0 = ____ m   pitch = ____°   roll = ____°   rms = ____ mm
                按距离残差 ____ / ____ / ____ / ____ cm
[3] odom 地面倾角 ____°   ground_in_base = ____ m   ground_vs_vehicleZ = ____ m
[4] 地面误报 ____%（intensity 闸门后）   误标栅格 ____ 个，距离分布 ____

改动：lidar_z ____ → ____；min_height ____ → ____；max_height ____ → ____
      其他：____
复测日期：____  复测结论：____
```
