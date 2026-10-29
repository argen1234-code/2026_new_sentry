# rosbag 回放调参法（离线调 nav2 规划器与控制器）

> 目的：不碰实车、不使能底盘，用**录好的实车点云**闭环驱动**真实 nav2 代码**，
> 把「参数 → 指标」的对照实验从「一次 3 分钟、要有人走位」变成可重复的离线跑批。
> 台架代码在 [`ros_ws/tools/replay/`](../ros_ws/tools/replay/)，2026-10-29 首次跑通。
> 踩坑条目另见 [`pitfalls.md`](pitfalls.md)；参数取值见 [`parameters.md`](parameters.md)。

## 一、能做什么、不能做什么

**能**：给定同一段世界点云，比较不同参数下**规划路径形状、重规划频率、指令平滑度、
停车次数、里程/时长**的相对排序。这是调参真正需要的东西。

**不能**：给绝对值背书。世界点云是**单视角录制**——遮挡空洞、行人的出现/消失时刻都是
录制那一刻的；底盘是**建模**不是实物。所以结论按**相对排序**读，绝对值仍需实车 A/B。

## 二、架构

```
ros2 bag play --clock          只回放"世界"：/terrain_map、/terrain_map_ext
        │  --remap → /terrain_map_raw、/terrain_map_ext_raw
        ▼
   cloud_crop.py               按离仿真车距离裁剪（局部 6 m / 扩展 12 m），
        │                      还原"车只看得到视野内"；--no-crop 可关
        ▼
   nav2（真实代码）             navigation_launch.py use_composition:=false
   ├ map_server                ← 静态地图（必须先确认录包用的是哪张）
   └ 7 个节点 + fake_vel_transform → /cmd_vel
        ▼
   kinematic_sim.py            /cmd_vel + 执行器模型 → 积分位姿
                               50 Hz 发 odom→base_footprint TF 与 /odometry
```

**不回放** `/tf`、`/odometry`、`/plan`、`/cmd_vel*`、两个 costmap——它们由台架产生，
回放会和仿真车抢 TF，也会把旧数据混进指标。回放端只给「世界点云」。

## 三、安全红线

回放会真的发 `/cmd_vel`。**实车栈必须先停**：

```bash
./ros_ws/tools/stop_ros_stack.sh
pgrep -af 'usb_bridge|cmd_vel_to_unif'   # 必须为空
```

`run_variant.py` / `sweep.py` 启动前会自己 `pgrep` 检查，发现实车桥在跑就拒绝启动
（`real_stack_running()`）。但这只是双保险，先手停一层。

## 四、文件职责

| 文件 | 职责 |
|---|---|
| `kinematic_sim.py` | 订阅 `/cmd_vel`，按 `plant_*.json` 积分位姿并发 TF + `/odometry` |
| `cloud_crop.py` | 点云按仿真车距离裁剪后转发 |
| `goal_schedule.py` | `NavigateToPose` 客户端，按场景文件的 `t` 发目标 |
| `replay.launch.py` | map_server + lifecycle + 静态 `map→odom` + description + nav2 + sim + crop |
| `run_variant.py` | 跑单个变体：生成参数副本 → 起栈 → 等 lifecycle → 播包 → 发目标 → 录制 → 打分 |
| `sweep.py` | 串行跑一组变体，输出 Markdown + CSV 对照表 |
| `metrics.py` | 指标函数；**同一个脚本也给实车录包打分**，保证口径一致 |
| `variants.yaml` | 各变体覆盖的参数（只读 `nav2_params.yaml`，写副本） |
| `scenario_*.json` | 场景：录包路径、目标序列、实录参照指标 |
| `wait_active.py` | 单进程等 lifecycle 全 active（`ros2 lifecycle get` 轮询会超时） |
| `clock_preroll.py` | 起栈后先把 `/clock` 推进，让 autostart 在播包前完成 |

## 五、执行器模型（`plant_measured.json`）

底盘模型是**从代码推出来的**，不是拟合的（录包里稳态样本只有 775/14191，`vy` 几乎无激励）：

| 参数 | 值 | 来源 |
|---|---|---|
| `v_gain` | 0.55 | 弱拟合（slope 0.637、corr 0.52），早前实测区间 0.44–0.64，标注不确定 |
| `wz_gain` | 0.52 | 代码精确：注入 0.01 rad/s/count × `CHASSIS_YAW_HOLD_RC_SEN` 0.0052 |
| `deadband_v` | 0.1406 m/s | 25 counts × 0.005625 m/s per count |
| `deadband_w` | 0.25 rad/s | 25 counts × 0.01 rad/s |
| `frame_yaw_deg` | −45 | 死区作用在**旋转后**的底盘通道上，不是 ROS 帧 |
| `lag_tau` | 0.15 s | 一阶滞后假设，未辨识 |
| `latency` | 0.05 s | 未辨识 |

**死区的作用帧很关键**：主机侧先把 `(vx, vy)` 旋转 `frame_yaw`，固件的
`rc_deadband_limit()` 作用在旋转后的通道上。所以纯 `+x` 指令在 **0.199 m/s**
（= 0.1406 / cos45°）以下会被整体清零，沿对角线才是 0.1406。
这解释了「低速不走」的一阶现象。

`--plant ideal` 把所有增益置 1、死区/滞后置 0，用来分离「参数效应」与「底盘缺陷」。

## 六、场景文件

`scenario_*.json` 记录一个可复现的场景：

```json
{
  "bag": "/home/gzu-prink/bags/dyn_1029_1712",
  "map": "corridor_1029_run1.yaml",
  "goals": [
    {"t": 0.0,  "x": 13.78, "y": -1.08, "yaw": 0.0},
    {"t": 77.2, "x": 0.03,  "y": 0.07,  "yaw": 3.14159}
  ],
  "reference_metrics": { "...": "metrics.py 对实录包打出的同一套指标" }
}
```

- 现有两个场景：`scenario_a.json`（`dyn_base_open`，2026-10-25，**X2 之前**）、
  `scenario_b.json`（`dyn_1029_1712`，2026-10-29，**X2 之下**）。
- `reference_metrics` 必须是 `metrics.py` 直接对录包跑出来的，不要手抄。

## 七、怎么跑

```bash
source ~/2026_new_sentry/ros_ws/install/setup.bash      # 见踩坑 1，必须
cd ~/2026_new_sentry/ros_ws/tools/replay

# 单跑一个变体
python3 run_variant.py --variant G0 --scenario scenario_b.json \
    --map ~/2026_new_sentry/maps/corridor_1029_run1.yaml \
    --plant measured --tag 描述 --force

# 跑一组变体（G0–G6）
python3 sweep.py --scenario scenario_b.json --variants G0,G1,G2,G3,G4,G5,G6 \
    --map ~/2026_new_sentry/maps/corridor_1029_run1.yaml --tag 描述
```

常用开关：`--plant ideal`、`--no-crop`、`--out`（默认 `/tmp/replay/results`）、
`--rviz`、`--manual`（改成 RViz 手发目标）。

单次约 **4.5 分钟**：起栈 5 s + 播包（录包时长）+ 收尾约 85 s。

## 八、指标口径与门槛

`metrics.py` 对录包和回放用**同一套函数**打分，避免口径漂移：

| 指标 | 含义 | R2 门槛 |
|---|---|---|
| `duration_s` | 终点到达总时长 | 实录 ±25% |
| `path_length_m` | 里程 | 实录 ±20% |
| `jitter_p90_m` | 相邻 `/plan` 在 0.5–3.0 m 段的横向差 p90 | 实录 ±50% |
| `max_x` / `final_pose` | 覆盖范围与终点 | 越接近越好 |
| `replan_hz` | `/plan` 频率 | 与参数配置一致即可 |
| `longest_stop_s` / `stops_ge_05s` | 停车 | 越少越好 |
| `cmd_accel_p95_ms2` | `/cmd_vel` 相邻加速度 p95 | ≤2.5（实车口径） |

## 九、2026-10-29 实测（scenario B，G0 基线）

实录参照：166.5 s / 34.47 m / 2.55 Hz / 抖动 p90 0.125 m / 最大东进 13.52 m / 0 次停车。

| 跑 | 时长 | 里程 | 抖动 p90 | 最大东进 | 终位姿 | ≥0.5 s 停车 |
|---|---|---|---|---|---|---|
| measured（裁剪开） | 166.25 | 28.53（−17.2%） | 0.140 | 13.50 | (0.31, 0.11) | 2 |
| measured（`--no-crop`） | 167.22 | 27.72 | 0.114 | 13.50 | (0.22, 0.09) | 3 |
| ideal（无死区/增益/滞后） | 118.38 | 27.76 | 0.159 | 13.55 | (0.10, 0.06) | 3 |
| 修复前（标称 dt 积分） | 167.15 | 24.82（−28%） | 0.142 | 12.12 | (0.64, 0.14) | 4 |

结论：

1. **保真度闸门通过**（时长、里程、抖动三项都在门槛内）。
2. **裁剪开/关只差 2.8% 里程**，说明裁剪没引入系统性偏差，可以默认开着。
3. **理想执行器把同一路线跑成 118 s（比实录快 29%）**，里程 27.76 m 与 measured 的
   28.53 m 基本一致 ⇒ 底盘缺陷主要吃**时间**（移动均速 0.378 vs 实录 0.210），
   不吃**路线形状**。所以规划器/控制器的几何类结论对执行器模型不敏感，
   而时间类结论必须用 measured 模型。
4. 仍存偏差，读结论时要记住：
   - 重规划 4.3 Hz vs 实录 2.55 Hz——仿真没有实车的定位/计算开销；
   - 仿真在 goal1 提前 10 s 到达，只好等到 77.2 s 的目标切换，产生 14.7 s 等待；
   - 指令加速度 p95 0.21 vs 实录 1.73——仿真 odom 无噪声，控制器不需要补偿抖动。

## 十、踩过的坑

1. **环境被别的工作区覆盖。** shell 里若已 `source` 了另一个工作区（如 `printk_ws`），
   `sentry_description` 不在搜索路径，launch 直接 `PackageNotFoundError`。
   **每次跑之前显式 `source ros_ws/install/setup.bash`**。
2. **map_server 起栈竞态。** `nav2_lifecycle_manager` 只等几毫秒就发 CONFIGURE，
   与 `map_server` 同刻创建会打空、永久挂起（`/map` 不发布、静态层永远空的）。
   `replay.launch.py` 用 `TimerAction(period=3.0)` 包住 map 的 lifecycle。
3. **相邻两跑之间残留节点。** 上一跑结束后立刻起下一跑，旧 `map_server` 未退干净，
   新的会卡在 `Configuring map_server` 之后无输出、`lifecycle get` 报 `inactive`。
   现象是静态层刷 `no map received`。**起跑前 `pgrep` 确认无残留，失败就单独重试。**
4. **`ros2 bag play` 的 `-r` 不是 remap。** argparse 把 `--ros-args -r` 当成 `--rate`
   的缩写，报 `... is not the valid type (float)`。改用 `--remap A:=B`。
5. **仿真车系统性慢 20%（已修）。** `kinematic_sim` 原先按**标称** `dt = 1/50` 积分，
   但 `--clock` 下定时器实际 25 ms 一拍，于是每 25 ms 只推进 20 ms 的运动。
   判据：**位置积分速度 / 自身 `/odometry` twist 恒为 0.806**。改成按真实时钟差积分后
   比值精确回到 1.000，里程 24.82 → 28.53 m、最大东进 12.12 → 13.50 m。
   **以后任何"仿真车比它自己报告的慢"的现象，先查积分 dt。**
6. **录包用的是哪张地图，必须实测不能猜。** 用录包首帧全局代价地图的致命格去比对各 PGM
   的占据格：`corridor_1029_run1` 命中 83.2%，`corridor_1025_tape` 只有 6.4%。
   差一次就是整场回放对不上。
7. **先确认录包属于哪个参数版本。** `/plan` 2.55 Hz + 代价地图 1.7–1.8 Hz ⇒ 已含 X2，
   基线用 G0/C0；若是 0.9 Hz + 1.0 Hz ⇒ X2 之前，必须用 `variants.yaml` 里的 F0。
8. **`/tmp` 会被清空。** 机器重启后 `/tmp/replay`、临时分析脚本全部消失。
   台架文件放仓库，`/tmp` 只放结果；分析脚本要留就写进仓库或重新生成。
9. **`goal_schedule` 的完成判据。** 曾用「所有目标已发送」判完成，被抢占目标回的
   `CANCELED` 结果会把整个跑提前 74 s 截断。现在用 `pending` 计数。
