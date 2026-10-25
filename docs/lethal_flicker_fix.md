# 致命代价闪烁修复方法（IntensityVoxelLayer）

> 现象：车静止时，全局/局部代价地图里的 LETHAL 格不断「换位置」——数量看着不多，
> 但每一帧都换一批，路径带上的噪点尤其明显；车前方真实障碍会整块消失一帧再出现。
>
> 适用：`pb_nav2_costmap_2d::IntensityVoxelLayer` + CMU `terrain_analysis*` 地形点云。
> 日期：2026-10-25。参数最终值见文末「推荐配置」。

## 一、机制（已逐条核实）

1. **地形源本身每帧覆盖不同。** `terrain_analysis_ext` 的 `decayTime: 0.2 s`
   （20 Hz 输出，约 2 帧激光）⇒ 每帧点云只含最近约 0.2 s 看到的点，
   实测每帧点数 853–3387 之间跳，相邻帧 5 cm 体素 IoU 只有 0.76。
   24 s 内出现过 18,024 个体素，其中 **84% 只在 ≤20% 的帧里出现**。
2. **代价地图层每个周期清空重画。** `IntensityVoxelLayer::updateBounds` 开头就
   `resetMaps()`（`ros_ws/src/sentry_navigation/pb_nav2_plugins/src/layers/intensity_voxel_layer.cpp:130`，
   与母工程同写法），周期内先射线清除、再逐点标记。
3. **默认没有时间持久化。** `observation_persistence` 默认 0.0 ⇒ 每周期只吃最新一帧。
4. **结论：每张代价地图 = 单帧地形点云的投影**，源头的帧间抖动被原样放大成闪烁。
   实测全局致命格相邻帧 IoU 仅 0.32、局部 0.42，比输入点云还差。

## 二、怎么量（不要只看格数）

格数稳定 ≠ 不闪烁：阶段 U 改完后格数 869–901（8/10 帧），看着很稳，
但逐格比对 IoU 只有 0.35——**每帧换了一批新格子**。

用两个指标：

- **格身份 IoU** = `|A∩B| / |A∪B|`，A/B 为相邻两张代价地图的 LETHAL 集合。
  1.0 = 完全不动；<0.5 = 至少一半格子换了。只看数量会被骗。
- **路径带统计**：以车体为原点，取前方 x∈[0,3] m、`|y|≤0.30 m` 的带内致命格数与 IoU，
  直接对应「路径上的噪点」。

取数方式（车静止，不使能底盘）：

```bash
# 起栈（enable_motion:=false，不发 cmd_vel、不发目标）
./scripts/start_sentry.sh slam:=true enable_motion:=false visualization:=headless
# 确认 /cmd_vel 无消息后再录
ros2 bag record -o /tmp/terrain_flicker -e '.*(terrain_map|terrain_map_ext|costmap)$'
# 25 s 后 Ctrl-C 停录，再 SIGINT 停栈
```

离线分析脚本（在本机 `~/sentry_diag/`，未入库）：
`bag_analyze.py`（点云/代价地图逐帧 IoU、体素出现频率）、
`simulate_marks.py`（按层逻辑离线重放，预判参数效果）、
`lethal_probe.py`（在线探针，含路径带与各源点支撑）。
注意 `simulate_marks.py` 的采样率必须设成代价地图的**实际发布频率**，否则结论会偏乐观。

## 三、两个旋钮（语义已核实）

| 参数 | 语义 | 调大的效果 | 代价 |
|---|---|---|---|
| `observation_persistence`（按观测源声明） | 标记时取**最近该时间窗内**的全部观测，20 Hz 地形云下 1 s ≈ 20 帧取并集 | 帧间并集变稳，闪烁下降 | 动态障碍消失延迟 = persistence，身后留同长时间尾迹 |
| `mark_threshold` | 传给 `VoxelGrid::markVoxelInMap()`，返回**该 (x,y) 格内被占据的 z 体素数 > 阈值** | 只保留有多个高度层的物体，单层薄物被丢 | 格数下降，孤立点/薄边缘消失 |

注意：`mark_threshold` 数的是**高度层数，不是点数**；
`unknown_threshold` 在本层里声明了但从未读取，调它无效。
`z_voxels`/`z_resolution` 决定 z 向分辨率，不解决闪烁。

## 四、关键陷阱：发布频率必须与持久化窗口对齐

第一轮把 persistence 设 0.5 s 后，局部达标（发布 2 Hz）而全局仍只有 0.46。
原因是全局 `publish_frequency: 1.0`，**相邻两张全局图相隔 1 s，持久化窗口 0.5 s
的两个窗口互不重叠**，等于没共享帧。

规律：`publish_frequency ≥ 1 / observation_persistence` 是下限，
最好让窗口至少覆盖 2 个发布间隔。我们最终的组合：

- 全局：publish 2.0 Hz + persistence 1.5 s
- 局部：publish 2.0 Hz + persistence 1.0 s

另一个实测事实：全局代价地图**实际**只能跑到 ~1.2 Hz（设 2.0），
因为一个更新周期约 0.8 s。调参数时按实际频率算，不要按配置值算。

## 五、实测结果（车静止 25 s，同一位置）

| 指标 | 修复前 | persistence 0.5 | pers 1.0 + 全局发布 2 Hz | **最终（全局 1.5 / 局部 1.0）** |
|---|---|---|---|---|
| 全局致命格 IoU | 0.32 | 0.46 | 0.67 | **0.72**（最低 0.62） |
| 局部致命格 IoU | 0.42 | 0.64 | 0.77 | **0.75**（最低 0.66） |
| 路径带 IoU（全局/局部） | 0.51 / 0.63 | — | 0.74 / 0.77 | **0.77 / 0.75** |
| 实物区（x 1–2.6）每帧致命格 | 0–59，有空帧 | 31–88 | 93–117 | 101–132，无空帧 |
| 全局致命格数 | 297–888 | 307–652 | 510–799 | 846–977 |

最后一行偏高不是过猛：离线仿真只数地形层，实测代价地图还叠加 `static_layer`
（SLAM 地图里的墙，约 180–200 格）。路径带与实物区都在合理范围。

离线仿真的预测（按实际发布频率采样）与实测在两档参数上都吻合到 0.02–0.03，
可作为下次调参的先验：
ext 源 `pers 1.0 → 0.67`、`1.25 → 0.71`、`1.5 → 0.75`、`2.0 → 0.79`。

## 六、推荐配置

`ros_ws/src/sentry_navigation/pb2025_nav_bringup/config/reality/nav2_params.yaml`：

```yaml
local_costmap:
  intensity_voxel_layer:
    mark_threshold: 1
    terrain_map:
      observation_persistence: 1.0

global_costmap:
  publish_frequency: 2.0        # 与持久化窗口对齐
  intensity_voxel_layer:
    mark_threshold: 1
    terrain_map_ext:
      observation_persistence: 1.5
```

配套前提（2026-10-25 之前已改，别回退）：

- `min/max_obstacle_height: 0.0/2.0`（层与源各两处）
- `use_footprint_filter: true` + `footprint_padding: 0.09`
- `min/max_obstacle_intensity: 0.1/2.0`

## 七、代价与未验证项

- **动态障碍尾迹**：被持久化的旧帧点每个周期都会被重新标记，清除射线只擦「到当前点
  的视线」，所以障碍离开后最多 persistence 秒内仍在图上（全局 1.5 s，按上限 0.45 m/s
  约 0.68 m）。**静止测试无法验证**，动车时要观察规划器是否会绕旧位置；若不能接受，
  把全局 ext 退回 1.0（IoU 0.67）。
- **自车最近 LETHAL ≠ footprint 失效**：实测 0.33–0.47 m 的近距离致命格来自
  `static_layer`（它不做 footprint 过滤），车正停在净宽 0.62 m 的窄口里，墙就在 0.31 m。
  只算地形层时最近格心 0.44 m，`use_footprint_filter` 正常。
- **unknown 仍占 97%+**：与本问题无关（缺少 freespace 广播），另开。
- **全局实际发布 ~1.2 Hz**，未达配置的 2.0；若后续做性能优化，IoU 还会再升。

## 八、回滚

**不要** `git checkout -- nav2_params.yaml`（该文件累计了阶段 U/V 的多处改动）。定点回滚：

1. 两处 `observation_persistence` 删除或改回 0.0；
2. 两处 `mark_threshold` 改回 0；
3. `global_costmap.publish_frequency` 改回 1.0；
4. 重启栈（不需要编译，`install/` 下是该 yaml 的符号链接）。

改前备份副本在本机 `/tmp/nav2_params_before_V2.yaml`、`/tmp/nav2_params_before_V3.yaml`。
