# nav2 调参手册（给 agent 用的操作流程）

> 读者：接手本工程导航调参的 agent 或人。目标不是"讲清 nav2 原理"，而是
> **拿到一个症状，知道该看哪个指标、动哪个参数、怎么验证、怎么回滚**。
> 台架原理见 [`bag_replay_tuning.md`](bag_replay_tuning.md)，参数取值见
> [`parameters.md`](parameters.md)，踩坑见 [`pitfalls.md`](pitfalls.md)。
> 协作规则见工程根目录 `CLAUDE.md`（14 条，最高权威）。

---

## 一、先记住四条铁律

1. **动车先问**（规则 1）：使能底盘、发 `cmd_vel`、发导航目标、起整栈之前，说明要做什么
   并问清车体物理状态（着地 / 架空 / 周围障碍）。
2. **改关键链路先问**（规则 2）：`nav2_params.yaml`、`usb_bridge/`、`stm32_ws/` 固件，
   改前讲清影响面与回滚方式。回滚命令：
   ```bash
   cd ~/2026_new_sentry && git checkout -- ros_ws/src/sentry_navigation/pb2025_nav_bringup/config/reality/nav2_params.yaml
   ```
3. **车体运动调试主动结束对话**（规则 11）：准备好检测手段、把命令交给用户，
   等「任务执行完成」再读数据。
4. **注释和文档只在用户宣布阶段成功后更新**（规则 13）。

**能用回放台就不要上实车。** 回放台不碰底盘、可重复、单次 4~6 分钟，
适合回答"哪个参数更好"；实车只用来确认"回放台的结论在实物上成立"。

---

## 二、调参前必须知道的三条链路

参数改错，通常不是"值不合适"，而是**改在了不归它管的链路上**。三条链互相独立：

| 链路 | 输入 | 输出 | 主要参数 | 谁看得见 |
|---|---|---|---|---|
| 感知/代价 | `/terrain_map`、`/terrain_map_ext` | `/local_costmap`、`/global_costmap` | `intensity_voxel_layer.*`、`inflation_layer.*`、`terrain_analysis.*` | 规划器、控制器 |
| 规划/控制 | costmap + goal | `/cmd_vel_controller` → `/cmd_vel` | 全局/局部 planner 参数、`FollowPath.*`、`velocity_smoother.*` | 底盘 |
| 建图 | `/terrain_map_ext` → `pointcloud_to_laserscan` → `/obstacle_scan` | `/map` | `pointcloud_to_laserscan.min_height` 等 | 只有 SLAM |

**建图看不见 ≠ 导航看不见**，反之亦然。低矮障碍常常"导航能绕、地图上没有"，
就是 `min_height: 0.16` 这道独立的门（详见 [`parameters.md`](parameters.md) 的高度闸门一节）。

### 已核实的物理事实（调参时当常数用）

| 事实 | 数值 | 影响 |
|---|---|---|
| 底盘速度死区 `deadband_v` | 0.1406 m/s（纯 +x 时 0.199） | 低于它的指令被零化，末端一顿一顿 |
| 角速度死区 `deadband_w` | 0.25 rad/s | 低于它的转向指令无效 |
| 速度增益 `v_gain` | 0.55 | 实际速度约为指令的 35~55% |
| `frame_yaw` | −45° | 指令在进固件前被旋转，`vx/vy` 不能按世界系理解 |
| `/terrain_map` 频率 / IMU | 11.4 Hz / 112 Hz | 代价地图 `update_frequency` 再高也没新数据 |
| 走廊典型净宽 | 1.65 m（含 y=0 的最窄横向通道） | `inflation_radius` 的硬约束 |

`frame_yaw` 与死区这两条已经导致过"前进变横移 90°"和"全链路通但只跑到 35%"，
改动速度/方向相关参数时先回看记忆里的这两条。

---

## 三、工具速查

全部工具都在 `ros_ws/tools/`，纯读，不碰车。

| 工具 | 一句话 | 典型用法 |
|---|---|---|
| `nav_tune/lint.py` | 改前自查参数文件 | `python3 tools/nav_tune/lint.py` |
| `nav_tune/doctor.py` | 跑完自查这一次的包健康与否 | `python3 tools/nav_tune/doctor.py ~/bags/xxx` |
| `nav_tune/report.py` | 这次到底跑成什么样（形状级） | `python3 tools/nav_tune/report.py ~/bags/xxx --plot /tmp/r.png` |
| `replay/metrics.py` | 标量评分（扫参数用的唯一口径） | `python3 tools/replay/metrics.py <bag> --json /tmp/m.json` |
| `replay/run_variant.py` | 跑一个参数变体 | 见下 |
| `replay/sweep.py` | 连跑多个变体出对比表 | `python3 tools/replay/sweep.py --variants B0,I1,R1` |
| `nav_tune/params.py` | 跑着的栈 vs 仓库 yaml；`--set` 改参并回读 | `python3 tools/nav_tune/params.py --diff` |
| `nav_tune/compare.py` | 多份包同表 A/B，带相对基线的百分比 | `python3 tools/nav_tune/compare.py 基线包 变体包 …` |
| `plotjuggler`（2026-10-30 装入，`ros-humble-plotjuggler-ros 2.3.1`） | 时间序列同屏对照，看滞后/死区/削峰 | `ros2 run plotjuggler plotjuggler` → Data→Load a file → 选 bag 目录 |

### lint.py —— 改参数前 5 秒自查

```bash
python3 ros_ws/tools/nav_tune/lint.py                    # 仓库里的 nav2_params.yaml
python3 ros_ws/tools/nav_tune/lint.py other.yaml --map ~/2026_new_sentry/maps/corridor_XXXX.yaml
```

它会抓这些**运行时不会报错**的问题：

- 高度带 `min > max`、曲率闸门 `min > max`、加速度符号写反；
- `mark_threshold × z_resolution` 要求的竖直厚度大于低矮障碍，障碍被静默吞掉；
- `min_approach_linear_velocity` 低于实测死区（末端指令被吃掉）、
  `velocity_smoother.deadband_velocity` 为 0 却不帮忙兜底；
- `inflation_radius ≥ 通道半宽`：整条通道都有代价，规划路径由代价微差决定 ⇒ 蛇形；
- 死参数（`w_heuristic_cost`、`how_many_corners`、`lookahead_dist`）写着但不起作用；
- BT 文件是否存在、`smoother_server` 是否被 BT 引用（配了没人用）；
- `install` 是软链还是拷贝（拷贝则改源码不生效，必须 `colcon build`）。

ERROR 退出码为 1，可以拿来做门禁。

### doctor.py —— 每次跑完先问"栈本身健康吗"

```bash
python3 ros_ws/tools/nav_tune/doctor.py ~/bags/crash_repro
python3 ros_ws/tools/nav_tune/doctor.py /tmp/replay/results/B0__.../bag --robot-radius 0.30
```

检查项：关键 topic 频率、`/odometry` 连续性、**车体中心格是否致命**（= 规划器会拒绝起点的
撞车签名）、0.12 m / 0.30 m 两圈的近场致命格（区分幽灵噪点与真障碍）、有平移指令却不动、
代价地图/规划间断、`map->odom` 是否存在。它决定"这次数据能不能拿来调参"——
栈本身不健康时，指标没有意义。

### plotjuggler —— 想看"指令 vs 实际"的波形时

装好即可用（`ros-humble-plotjuggler-ros` 提供 ROS 2 bag 读取插件）：

```bash
source /opt/ros/humble/setup.bash
ros2 run plotjuggler plotjuggler     # Data → Load a file → 选 bag 目录
```

调参时最常同屏看的几条：`/cmd_vel/twist/linear/x`（控制器输出）、
`/odometry/twist/twist/linear/x`（车实际速度）、`/cmd_vel/twist/angular/z` 与
`/odometry/twist/twist/angular/z`。**指令与实际的幅度比就是增益、相位差就是滞后、
低位被压成 0 的台阶就是死区**——这三件事决定了参数表里的死区和速度上限能不能信。

### report.py —— 形状级归因

```bash
python3 ros_ws/tools/nav_tune/report.py ~/bags/crash_repro --plot /tmp/corner.png --json /tmp/corner.json
```

在 `metrics.py` 的标量之上，补上解释为什么慢的几项：

| 字段 | 含义 | 读法 |
|---|---|---|
| `serpentine_max_m` / `_ratio` | 每帧路径相对"起点→终点"弦的最大横偏 | >0.3 m 或 >10% 弦长 ⇒ 规划器在摇摆 |
| `plan_cost_median` / `plan_frac_cost_gt0` | 路径压在代价格上的程度 | 中位 >0 说明没有零代价通道 |
| `speed_at_curv_*` | 不同曲率段的实际速度 | 高曲率段速度被压到一半 ⇒ 曲率闸门在起作用 |
| `inplace_rotation_frac` | 原地转向时间占比 | >10% 通常来自 `use_rotate_to_heading` |
| `yaw_reversals` | 角速度换向次数 | 与抖动、停车强相关 |
| `ctrl_vs_smoothed_p95` | 控制器输出与平滑器输出的差 | 差大 ⇒ 平滑器在削峰，参数不匹配 |
| `legs` | 按目标切换切分的每段时长/里程/均速 | 一眼看出哪一段拖慢整体 |

---

## 四、标准流程（从症状到结论）

```
0. lint            参数文件本身没写错
1. 定症状          从包里说清"慢在哪一步"（规划摇摆 / 原地转向 / 死区 / 碰撞）
2. doctor          这次数据可用吗
3. report          形状级归因，锁定是规划器还是控制器
4. 回放台保真      基线复现实录，否则扫出来的排序不可信
5. 单变量扫        一次只改一个，跑 run_variant/sweep
6. 组合验证        把各自有效的组合起来，确认不互相抵消
7. 实车 A/B        规则 1/11：先讲影响面、给检测方案、结束对话
8. 入库            用户宣布成功后，更新 parameters.md / pitfalls.md
```

### 第 4 步：回放台保真（最容易自欺的一步）

基线跑不出实录的形状，变体的排序就没有意义。跑基线时**必须**把 `map->odom` 从包里量出来
喂进去，不能吃台架默认值：

```bash
# 从包里量 map->odom 的中位数（x y z yaw）
python3 - <<'EOF'
import sys; sys.path.insert(0, "ros_ws/tools/nav_tune")
import common as C, numpy as np
d = C.load("/home/gzu-prink/bags/crash_repro", topics=["/tf"])
m = np.array(d["map2odom"])
print("median x %.3f y %.3f z %.3f yaw %.4f" % tuple(np.median(m[:,1:5], axis=0)))
EOF

python3 tools/replay/run_variant.py --variant B0 --scenario scenario_c.json \
  --map ~/2026_new_sentry/maps/corridor_XXXX.yaml --plant measured \
  --map-odom-x -0.383 --map-odom-y 0.045 --map-odom-z -0.220 --map-odom-yaw -0.0148 \
  --tag myrun --force
```

`map_odom_z` 不是小事：体素层的 `min_obstacle_height: 0.0` 是 **map 系**下的闸门，
z 抬高 6 cm 就会把地面点和自车近场点标成致命格，仿真车当场卡死、规划器开始报
`Either of the start or goal pose are an obstacle!`。实录里这个 z 是负的（约 −0.22）。

保真合格线（对 `scenario_c.json` 的 `reference_metrics`）：

| 指标 | 合格线 |
|---|---|
| `duration_s` | ±25% |
| `path_length_m` | ±20% |
| `jitter_p90_m` | ±50% |
| `replan_hz` | ±30% |
| 有无卡死 | 卡死即判失败，先查 `map_odom_z`、地图选择、环境残留 |

### 第 5 步：单变量扫

变体写在 `tools/replay/variants.yaml`，每个变体只写与基线不同的键：

```yaml
  I1:
    label: 全局膨胀半径 1.04→0.70
    changes:
      global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius: 0.70
```

```bash
# 一次跑多个
python3 tools/replay/sweep.py --variants B0,I1,I2 --scenario scenario_c.json \
  --map ~/2026_new_sentry/maps/corridor_XXXX.yaml \
  --map-odom-x -0.383 --map-odom-y 0.045 --map-odom-z -0.220 --map-odom-yaw -0.0148
```

`sweep.py` 会把实录基线和你跑的变体放在同一张表里，指标口径完全一致
（同一份 `metrics.py`）。**读相对排序，不读绝对值。**

---

## 五、症状 → 指标 → 根因 → 参数

| 症状（在 report/metrics 里看到的） | 先查的指标 | 可能根因 | 对应的旋钮 |
|---|---|---|---|
| 路径左右蛇形，每 3 Hz 换一条 | `serpentine_max_m`、`plan_frac_cost_gt0` | 膨胀半径覆盖整条通道 | `inflation_layer.inflation_radius`、`cost_scaling_factor`、`GridBased.w_traversal_cost` |
| 抖动指标高但路径直 | `jitter_p90_m`、`ctrl_vs_smoothed_p95` | 平滑器没参与 / 参数打架 | BT 加 `SmoothPath`；`velocity_smoother.*` |
| 每次掉头先停再转 | `inplace_rotation_frac`、`longest_stop_s` | `use_rotate_to_heading` 在方向反转时生效 | `FollowPath.use_rotate_to_heading`（全向底盘可关） |
| 全程只有一半速度 | `speed_moving_median_ms`、`speed_at_curv_*` | 曲率闸门长期触发 | `curvature_min/max`、`reduction_ratio_at_high_curvature` |
| 末端一顿一顿、到不了点 | `goal_reaction`、`frac_speed_lt_005` | 指令低于底盘死区 | `min_approach_linear_velocity`、`velocity_smoother.deadband_velocity` |
| 撞上障碍还在往前推 | `doctor` 的"车体半径内致命格" FAIL | 低矮障碍没进代价地图 | 高度闸门四件套，见 [`parameters.md`](parameters.md) |
| 规划器报 `start or goal are an obstacle` | doctor 的致命格 + `/plan` 断流 | 车体中心落在原始致命格（已接触） | 先退回，再查感知；nav2 无法自恢复 |
| 规划很慢/无路 | `replan_hz` 掉到 0、`global_costmap` 间断 | 未知区域被当障碍 / 地图畸形 | `GridBased.allow_unknown`、静态地图质量 |
| 地图上看不到障碍 | `/obstacle_scan` 有无效距离 | 建图高度带把它滤了 | `pointcloud_to_laserscan.min_height/min_intensity` |

**注意代价地图的发布值**：`costmap_2d` 发布的是压缩后的 0–100，致命格是 **100（或 99）**，
不是内部值 254。按 254 判断会永远看不到致命格。

---

## 六、判据与合格线

调完不能只说"感觉顺了"，按下表给数：

| 指标 | 合格线 | 出处 |
|---|---|---|
| 到点滞后（每目标） | 不劣于基线，且每个目标都 SUCCEEDED | 见下方"时长在这个回放台上是伪指标" |
| 时长 `duration_s` | **只做一致性检查**，不能当改善证据 | 见下方 |
| 移动均速 `speed_moving_median_ms` | 上升 ≥15% | 同上 |
| 抖动 `jitter_p90_m` | ≤ 基线 | 同上 |
| 指令加速度 `cmd_accel_p95_ms2` | ≤ 2.5（先按下方口径去重，原始值除以 1.6） | 实测门限 |
| 停车 `longest_stop_s` / `stops_ge_05s` | 不劣于基线 | 同上 |
| 最小间距 `min_clearance_m` | ≥ 0.15 m（实车按车体半径另算） | 安全 |
| 碰撞 | 0 次 | 一票否决 |

**时长在这个回放台上是伪指标。** `goal_schedule.py` 按**录包里的时刻**发目标
（`t_origin + goal["t"]`），不等上一个目标结束，所以一次回放的总时长由时刻表决定，
不由车跑得快不快决定。2026-10-30 的五个变体实测：总时长 201.6–206.1 s（差 2.2%），
而"移动中位速"相差 89%（0.249 → 0.470 m/s）——速度翻倍了，总时长纹丝不动。

能用的替代指标是**每目标到点滞后**：`lag_k = (s_k − s_0) − (t_k − t_0)`，
其中 `s_k` 是第 k 个 `Goal succeeded` 的日志时刻、`t_k` 是场景文件里的计划时刻。
滞后持续为正 = 跟不上录包时的节奏；有目标根本没 `Goal succeeded` = 该变体不合格。

```bash
grep -E "Goal succeeded" /tmp/replay/results/<变体>/launch.log   # 取 bt_navigator 那几行的时间戳
```

**`cmd_accel` 的口径要注意。** `metrics.py` 的 `cmd_accel()` 是对 `seq` 相邻两帧直接求导，
而 `/cmd_vel` 是 20 Hz 内容被 96 Hz 发布架着发出来的——实测 **83% 的相邻帧完全相同**，
真实的"变一下"发生在 0.05 s 的边界上，除到的是 0.01 s 的帧间隔。所以原始 p95 被系统性
放大 **约 1.6 倍**。判断"车跟不跟得动"要看去重后的值：

```python
ch = [cmd[0]]
for r in cmd[1:]:
    if (r[1], r[2], r[3]) != (ch[-1][1], ch[-1][2], ch[-1][3]):
        ch.append(r)          # 只保留真正变化的帧
# 再对 ch 逐对 hypot(dvx/dt, dvy/dt) 取 p95
```

2026-10-30 实测对照（去重后 / 原始）：基线 1.27 / 1.91、max_accel 3.0 的变体 3.04 / 4.05。
去重后与各变体自己设的 `max_accel` 对得上（矢量模长允许到单轴上限的 √2 倍），
原始值则会凭空造出"超过底盘能力"的假警报。

**目前还没有做过同配置重复跑的重复性实验**，所以低于 5% 的差异一律不算改善；
任何结论至少要在两个场景（或实车）上复现一次。

---

## 七、已知陷阱（踩过，别再踩）

1. **回放台 `map_odom_z`**：默认 +0.060 会把地面标成障碍，必须用从包里量出的真实值。
2. **`$(find-pkg-share)` 指向 install**：`nav2_params.yaml` 与 BT 都走这条路。
   symlink-install 时改源码即生效；换成拷贝安装后必须 `colcon build`（`lint.py` 会提示）。
3. **环境残留 `printk_ws`**：它会抢 `pb2025_nav_bringup` 等同名包，跑台架前显式
   `source ~/2026_new_sentry/ros_ws/install/setup.bash`，必要时把 `printk_ws` 从
   `AMENT_PREFIX_PATH` 等变量里剥掉。
4. **回放台 `/clock`**：`clock_preroll.py` 到点后**保持**时间不退出，退出会让 sim time
   冻结、生命周期永远不 active（表现为 `lifecycle never active`）。
5. **静态图冻结**：车不动时 `/map` 是 0×0，代价地图会报 malformed，规划器永久卡死。
   录包做调参时车必须动起来。
6. **录包用的是哪张地图**：先确认再回放，否则 `static_layer` 与点云对不上。
7. **`ros2 bag play -r` 改倍速会改变控制器行为**，对比实验一律 1.0。
8. **同名旧进程**：`pgrep -af 'usb_bridge|cmd_vel_to_unif'` 必须为空才允许跑回放台，
   `run_variant.py` 会自己拦。

---

## 八、工具清单（2026-10-30 核对过一遍）

这台机器 `DISPLAY=:0`，GUI 可以直接从终端起，不需要另开桌面操作。

| 用途 | 工具 | 版本 | 起法 / 用法 |
|---|---|---|---|
| 时间序列对照 | `plotjuggler` + `plotjuggler-ros` | 3.17.2 / 2.3.1 | `ros2 run plotjuggler plotjuggler` |
| 运行时改参（GUI） | `rqt_reconfigure` | 1.1.3（本来就有） | `ros2 run rqt_reconfigure rqt_reconfigure` |
| 运行时改参（命令行） | `ros2 param get/set/dump/load` + `nav_tune/params.py` | — | 见下 |
| bag 评分 / 归因 | `replay/metrics.py`、`nav_tune/{report,doctor,compare}.py` | 本仓库 | 见第三节 |
| 地图保存 | `nav2_map_server/map_saver_cli` | 1.1.20（本来就有） | `ros2 run nav2_map_server map_saver_cli -f maps/xxx` |
| 点云降采样 / 去噪 | `pcl-tools`（`pcl_voxel_grid`、`pcl_statistical_outlier_removal`） | 1.12.1（本来就有） | 见下 |
| 点云/先验地图查看与裁剪 | `cloudcompare` | 2.11.3 | `cloudcompare` |
| 行为树可视化 | Groot | 1.0.0 AppImage | `~/.local/bin/Groot.AppImage`（FUSE 不可用时加 `--appimage-extract-and-run`） |
| 实时话题查看 | `foxglove_bridge` | 3.5.0（本来就有） | `ros2 run foxglove_bridge foxglove_bridge` |
| 画图 | `matplotlib` | 3.5.1 | `report.py --plot` |

没装的：`python3-open3d`（400 MB+，pcl-tools 已够）、Groot2（面向 BT.CPP v4，
本工程是 v3）。

### params.py：确认"跑的"和"写的"是不是一回事

```bash
# 跑着栈的时候：列出文件里有、但节点从没声明的参数（写了也不生效）
python3 tools/nav_tune/params.py --diff

# 改一个参数并回读（nav2 的 costmap/controller 有一部分支持运行时改）
python3 tools/nav_tune/params.py --set controller_server.FollowPath.v_linear_max=0.6
python3 tools/nav_tune/params.py --get controller_server.FollowPath.v_linear_max

# 把每个节点的实时参数落盘，事后对比
python3 tools/nav_tune/params.py --dump /tmp/live_params
```

`--diff` 比 `lint.py` 更硬：lint 里的死参数清单是人工维护的，
`--diff` 直接问节点"你有没有这个参数"。

### pcl-tools：先验点云清噪/降采样

`maps/corridor_XXXX_prior.pcd` 有 110 MB，直接喂 relocalization 又慢又噪：

```bash
# 5 cm 体素降采样
pcl_voxel_grid maps/corridor_XXXX_prior.pcd /tmp/prior_5cm.pcd -leaf 0.05,0.05,0.05
# 统计离群点剔除（近邻 30 个、标准差倍数 1.0）
pcl_statistical_outlier_removal /tmp/prior_5cm.pcd /tmp/prior_clean.pcd -mean_k 30 -std_dev_mul 1.0
```

## 九、还没答案的问题（别当成结论用）

- `lidar_z` 与 `min_height` 该怎么成对调，真实离地高度（0.13 / 0.20 m）尚未定案。
- 小障碍物在实录中一次复现成功、一次没放上去，**"代价地图持续标记"仍需一次受控实验**。
- 全向底盘关掉 `use_rotate_to_heading` 后，终点朝向精度是否会退化，只有回放台数据，
  没有实车 A/B。

---

## 十、2026-10-30 定案：线速度提到 1.2 m/s

场景 `scenario_c`（5 m 走廊往返，8 个目标，`plant=measured`，`crash_repro` 点云），
同一张 `corridor_XXXX` 地图。基线 = 当时的 0.9 m/s + 现用 BT。

### 决定性数据

| 变体 | 移动中位速 | 抖动 p90 | 指令加速度 p95（原始/去重） | 间距<车体半径占比 | 到点 |
|---|---|---|---|---|---|
| B0 基线 | 0.249 | 0.116 | 1.91 / 1.27 | 0.710 | 8/8 |
| W2（代价权重 10→2） | 0.302 (+21%) | 0.112 | 1.73 / — | 0.754 | 7/8 |
| L1（1.2 + 加速度 3.0） | 0.339 (+36%) | 0.144 | 4.05 / 3.04 | 0.770 | 8/8 |
| **U1（1.2，加速度保持 2.5）** | **0.339 (+36%)** | 0.144 | 4.42 / 2.74 | 0.771 | **8/8** |
| U2（U1 + 代价权重 2） | 0.439 (+76%) | 0.150 | 3.39 / 2.68 | — | 8/8 |
| A1/A2（角速度 1.9 / 2.5） | 0.248 / 0.249 | 0.118 / 0.119 | 1.78 / 2.00 | 0.711 / 0.703 | 8/8 |

### 采纳的改动

| 文件 | 参数 | 从 → 到 |
|---|---|---|
| `nav2_params.yaml` | `FollowPath.v_linear_max` / `v_linear_min` | `0.9` → `1.2` / `-0.9` → `-1.2` |
| `nav2_params.yaml` | `velocity_smoother.max_velocity` / `min_velocity` | `[0.9,0.6,1.3]` → `[1.2,0.6,1.3]`，负向同步 |
| `usb_bridge.launch.py` | `max_vx` | `0.9` → `1.2` |

`max_accel` / `max_decel`（2.5 / 4.0）、`max_vy` 0.6、`max_wz` 1.3 **不动**。
2026-10-30 实车跑过，主观与到点都正常。

### 明确没采纳的四条（别再重复试）

1. **角速度抬到 1.9 / 2.5 —— 零效果。** 实测 `|wz|` 峰值 0.68，远没碰到 1.3 的闸；
   真正的上游限制是 `heading_pid_`（`rotation_kp: 1.0`），改 `max_velocity` 的角分量
   只是把一个够不到的闸挪得更远。
2. **`w_traversal_cost` 10→2 —— 提速明显但吃安全余量。** 路径间距小于车体半径的占比
   0.710 → 0.754，换来 +21%；再降到 **0 直接报废**：规划器穿障碍走，
   控制器 `isCollisionDetected` 连拒 12 次，8 个目标只到 1 个、里程 2.24 m。
   「路径弄直会不会撞」的答案是：**不会撞，但会僵住不动**。
3. **重规划 3 Hz→1 Hz —— 抖动翻 2.5 倍**（p90 0.116 → 0.295），到点 7/8。
   路径换得慢不等于路径稳，反而每次新路径与车身偏差更大。
4. **加速度同步抬到 3.0 / 3.5 —— 没必要。** U1 证明保持 2.5 就能拿到同样的 +36%。

### 还没解决的

提速只把**指令**抬上去了。真车实际速度仍受速度环死区/增益压制（见
`pitfalls.md` 速度环那一条）：`cmd 0.875 → 实测 0.390 m/s`，按 `0.55×cmd − 0.14`
推算 1.2 的指令对应约 0.52 m/s。回放台的 plant 模型**没有最高速饱和**，
所以台架测出来的永远是上限而非真车速——真车能跑多快只能上车量。

---

## 十一、2026-11-13 定案：过弯横偏（曲率减速 + 前瞻，必须成对改）

场景：`maps/corridor_1102`，去 `(4.643, 0.100)`、回 `(0.069, 0.077)`，
`use_wheel_fusion:=true`、`alpha 0.2`、`wz_gain:=1.0`，实车逐轮录包。
问题现象：回程弯口（x≈3.2–3.7）车被带低 20–30 cm，擦上 □ 挡板，第二轮 ABORT。

### 测量口径（三条，缺一条就会得出相反结论）

1. **横偏在 map 系算**：先套 `map→odom` 再与 `/plan` 求最近距离。直接拿 `/odometry`
   比 map 系 plan，会带上 `map→odom` 的平移偏置（本轮 −0.014…−0.068 m）。
2. **方向差在底盘系算**：`use_gimbal: False` 时 base 系 = 底盘系，指令本身已减去偏航；
   再减一次会凭空造出 `ψ` 的误差（见 `pitfalls.md` #33）。
3. **前瞻看实测模长**：统计 `/lookahead_point` 的 `hypot(x,y)`，不要只信参数值
   （会被 `min_lookahead_dist` 钳住，见 `pitfalls.md` #32）。

### 决定性数据（回程弯道，跟踪段；第四轮起无恢复动作）

| 轮次 | 改动 | 弯道前瞻中位 | 弯道指令速度 | 横偏峰值 | 横偏 p50 / p90 | 规划拒绝 / backup | 结果 |
|---|---|---|---|---|---|---|---|
| round02 | 基线（curv 1.0/0.75，look 1.0/0.5） | 0.69 m | 0.68 m/s | 27.1 cm | — | 3 / 1 | ABORT |
| round03 | curv 0.7/0.5 | 0.56 m | 0.45 m/s | 20.4 cm | 7.3 / 14.4 cm | 3 / 1 | 擦墙后恢复成功 |
| **round04** | + look 0.7/0.4 | **0.40 m** | 0.45 m/s | **9.4 cm** | **2.5 / 5.4 cm** | **0 / 0** | SUCCEEDED ×2 |

对照：round01（纯 LIO、同配置）横偏峰值 14.6 cm；11-13 参考融合包 28.1 cm（靠 backup 脱困）。
说明横偏是**配置性**问题，不是某一次异常。

### 机制（已交叉核对）

- 纯跟踪瞄准前方 `v × lookahead_time` 处的 carrot，车→carrot 的**弦线切弯内**，
  这是横偏主因；旋转滞后只是次要项（底盘系口径下中位 −5…+1°、p90 10–15°）。
- 曲率减速生效后速度下降 → 速度缩放前瞻随之变短 → 横偏先降到 20.4 cm。
- 再把 `lookahead_time` / `min_lookahead_dist` 成对下调，前瞻压到 0.40 m，
  横偏进 10 cm，不再触发规划拒绝与恢复。

### 采纳的改动

| 参数 | 从 → 到 | 备注 |
|---|---|---|
| `FollowPath.curvature_max` | 1.0 → 0.7 | 对齐北极熊 |
| `FollowPath.reduction_ratio_at_high_curvature` | 0.75 → 0.5 | 对齐北极熊 |
| `FollowPath.lookahead_time` | 1.0 → 0.7 | 必须与下一行成对改 |
| `FollowPath.min_lookahead_dist` | 0.5 → 0.4 | 否则被钳回 0.5 |

`alpha`、`enable_rotation`、`max_lookahead_dist`、规划器代价、BT、恢复行为**均未动**。

### 代价与后续

- **直道轻微摆头**：第一腿直道横偏 p95 4.3 → 6.9 cm、偏航角速度 std 0.31 → 0.35 rad/s、
  同段路径 3.18 → 3.40 m。绝对量只有 2–3 cm，弯道收益（−11 cm）更大，本轮接受。
- 若以后觉得直道晃：把 `use_velocity_scaled_lookahead_dist` 关掉、固定
  `lookahead_dist ≈ 0.45 m` 再 A/B；不要只继续降 `lookahead_time`（会被 min 兜住）。
- 未做的事：`enable_rotation: false`（北极熊原样）没有试——训练底盘没有独立云台，
  雷达随车体转，需要自转保持朝向；若换成正式云台底盘应重新评估。

---

## 十二、2026-11-13 定案：速度天花板在桥接钳位，不在底盘

### 现象与误导

第五轮把 `v_linear_max` 0.9 提到 1.2 后，底盘实测只从 0.74/0.91 涨到 0.79/0.99 m/s，
看上去像"底盘速度环饱和"。差点据此去改固件 PI/电流限幅。

**真实原因**：`cmd_vel_to_unified` 先把 `/cmd_vel` 旋转到固件车体系（`frame_yaw=-135°`），
**再逐轴钳位** `max_vx 1.2 / max_vy 0.6`。一条 ROS +x 的直行指令在这套坐标系里是
`(±0.707v, ±0.707v)`，0.85 m/s 以上就有一轴顶到 0.6。

round05b 录包 `/unified_cmd` 实证：`/cmd_vel (+1.20,−0.03)` → 下位机 `(−0.87,−0.60)`，
模长 1.06（−12%）、方向偏约 10°。这同时解释了当时回程弯道横偏突然变大——**方向被带歪了**。

### 修正与验证

- `usb_bridge.launch.py`：`max_vy 0.6 → 1.2`（与 `max_vx` 对称）。真实横移仍受
  `velocity_smoother` 的 0.6 约束，不会被这条改动放大。
- 6A 速度扫（有界往返，`/cmd_vel` 直发，不经 Nav2）：

| 指令 | 轮速 | LIO | 每轮电流 |
|---|---|---|---|
| 0.4 | 0.47 | 0.43 | 0.5–1.0 A |
| 0.8 | 0.80 / 0.72 | 0.80 / 0.76 | 0.8–1.4 A |
| 1.2 | **1.16 / 0.99** | **1.24 / 1.01** | **1.2–2.0 A** |

  满速时电流只有 1.2–2.0 A（限幅 4.5 A），**固件 PI 积分与电流限幅都不是瓶颈**，
  6B（改固件）取消。
- 6C/6D 实车（1.2 m/s + 弯道补偿）：实测 p90 1.04、峰值 1.26 m/s；两腿 8+11 s、
  9+11 s（原 10+12 s）；6D 加 `curvature_min 0.3` / `reduction 0.4` 后不再触发规划拒绝。

### 采纳的改动

| 文件 | 参数 | 从 → 到 |
|---|---|---|
| `usb_bridge.launch.py` | `max_vy` | 0.6 → 1.2 |
| `nav2_params.yaml` | `FollowPath.v_linear_max/min` | ±0.90 → ±1.2 |
| `nav2_params.yaml` | `velocity_smoother.max_velocity` | [0.90,0.60,0.8] → [1.2,0.60,0.8] |
| `nav2_params.yaml` | `FollowPath.curvature_min` | 0.4 → 0.3 |
| `nav2_params.yaml` | `FollowPath.reduction_ratio_at_high_curvature` | 0.5 → 0.4 |

### 离线回放复核（6E，scenario_1102，四变体）

| 变体 | 时长 | jitter p90 | 窄口去程 p50/p90 | 窄口回程 p50/p90 | 结论 |
|---|---|---|---|---|---|
| C0 基线 | 41.5 s | 0.302 | 2.1 / 3.3 cm | 5.8 / 8.9 cm | 维持 |
| E2 固定前瞻 0.45 | 40.6 s | 0.305 | 2.6 / **6.4** | 6.5 / 9.7 | 否决（更差） |
| E3 `w_traversal_cost 5` | 41.5 s | **0.273** | 2.3 / 3.5 | **5.3** / 9.1 | 小幅优，可单独 A/B |
| E4 弯道 0.60 | 41.2 s | 0.282 | **2.0 / 3.3** | 7.0 / 8.7 | 混合，不采纳 |

回放只用于**相对排序**：世界点云是单视角、仿真底盘无打滑/转动动力学，
绝对值（包括 `cmd_accel_p95 ≈ 8.3`）不能与实车比。

