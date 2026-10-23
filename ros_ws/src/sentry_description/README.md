# sentry_description

机器人描述包。默认加载**圆形全向轮模型**
`urdf/omni_round.urdf.xacro`（建图/导航链路默认也用它）；旧的三个模型保留
为备用，可用 `chassis_model` 切换。

```bash
colcon build --symlink-install --packages-up-to sentry_description
source install/setup.bash

# 圆形全向轮（默认）
ros2 launch sentry_description description.launch.py use_rviz:=true

# 备用：2026 正式底盘 + 四轴云台
ros2 launch sentry_description description.launch.py use_rviz:=true chassis_model:=sentry_2026

# 备用：训练车静态麦轮模型
ros2 launch sentry_description description.launch.py use_rviz:=true chassis_model:=mecanum

# 备用：更早的四舵轮 + 两级云台模型
ros2 launch sentry_description description.launch.py use_rviz:=true chassis_model:=sentry
```

`description.launch.py` 对 `.xacro` 文件会自动展开（xacro 处理），普通
`.urdf` 直接读取；`robot_description_file:=<绝对路径>` 可临时指定任意模型。

## 圆形全向轮模型（默认，`chassis_model:=omni_round`）

`urdf/omni_round.urdf.xacro`，单独看结构用 `display_omni.launch.py`：

```bash
ros2 launch sentry_description display_omni.launch.py
```

| 项 | 值 |
|---|---:|
| 车体半径 | 0.26 m（圆形） |
| 车体顶面 / 底面离地 | 0.06 / 0.02 m（点云约束，见下） |
| 轮半径 / 轮宽 | 0.075 / 0.045 m |
| 轮心圆周半径 | 0.26 m |
| 四轮方位角 | ±45°、±135°（轮轴沿半径方向，切向滚动） |
| 雷达位置 | `(0, -0.20, 0.12)`：从车头绕圆心**顺时针 90°**，车体右侧两轮之间；3 cm 立杆托在车板上 |
| 雷达朝向 | yaw = **−90°**（安装时相对车体顺时针转了 90°；RViz 里红箭头指向车体右侧是正常的，那是雷达自己的 +X） |
| 雷达到地面 | **0.12 m**（2026-10-23 点云标定，见下） |

雷达高度标定：只看 `/livox/lidar/pointcloud` 原始点云，取雷达下方的大平面
（地面）拟合。结果：地面在雷达坐标系下方 **0.120 m**（拟合残差 1.0 cm，
1814 个内点，覆盖 1.1–4.6 m、约 180° 方位，各距离段无系统性偏移）。
这也和 MID360 只向下看 7° 吻合——地面大约从 1.1 m 外才出现在点云里。
雷达安装相对地面还歪了约 1.4°（pitch 0.75°、roll 1.17°），太小，未写进关节。
`lidar_z` 是**到地面**的高度，关节里用 `lidar_z - wheel_radius`。

车体高度用点云定上界（**0.30 m 是错的，已删**）：MID360 只向下看 7°，半径
0.26 m 的车体板若顶面比雷达光心低 h，其表面会从 `h / tan7°` 处开始出现，一直到
板的最远边（距雷达 0.46 m）。h < 0.056 m 时，r = 0.16–0.46 m 的环带会被整片
画出；实际点云在 r = 0.2–0.6 m 只有 36 个点，所以**车体顶面必须 ≤ 离地
0.064 m**，此时板对雷达是掠射，基本无回波。模型取顶面 0.06 m、底面 0.02 m。
旧的 0.30 m 是假设值，会把雷达整个包进车体里。

近距离还有一簇 r = 0.10–0.21 m、最高离地 0.163 m 的 679 个点，按现在的雷达位姿
换算到车体系落在**车体中心附近**（圆心周围 0–0.14 m、高 0.11–0.16 m），是一块约
0.24 × 0.14 m 的近似水平面——大概率是中心立柱/支架顶板，也解释了它为什么只出现在
雷达的一侧。暂未建模（不影响 TF 和导航，只是 RViz 里看不到）。

TF 树（与建图/导航链路要求的帧名一致）：

```text
base_footprint -> base_link -> gimbal_yaw -> front_mid360
                            -> imu_link
                            -> wheel_front_left / wheel_front_right
                             / wheel_back_left  / wheel_back_right
```

- `base_link` 在**轮轴平面圆心**（离地 0.075 m）；车体板在 base_link 的
  z = -0.055 ~ -0.015（即离地 0.02 ~ 0.06 m），轮子顶端（离地 0.15 m）露在
  板上面；
- 轮心在 `base_link` 的 z=0 平面，轮底与地面相切，所以
  `base_footprint` 在 `base_link` 下方 0.075 m；
- `gimbal_yaw` 是兼容帧（本车没有云台），固定在 `base_link`：Nav2 的
  `robot_base_frame`、slam_toolbox / small_gicp 都用它，`fake_vel_transform`
  再从这里发布 `gimbal_yaw_fake`；
- `imu_link` 是兼容帧，usb_bridge 的 `/imu/data` 用这个名字，暂与 `base_link`
  重合；
- 四个轮子是 `continuous` 关节，关节名与 usb_bridge 的 `WHEEL_JOINTS`
  （`wheel_front_left_joint` 等）一致，由 `serial/joint_states_raw` 驱动；
- 模型没有 ros2_control，不参与仿真；几何参数集中在 xacro 顶部，改完不用
  重新编译（symlink install）。

RViz 配置里 RobotModel 必须显式写 `Description Source: Topic` +
`Description Topic: /robot_description` + `Durability Policy: Transient Local`：
Humble 的默认源是 File，只写旧字段 `Robot Description: robot_description`
时模型不渲染且不报错；durability 写成 Volatile 则收不到
`robot_state_publisher` 的 latched 消息，同样什么都不显示。
`rviz/omni_round.rviz` 与 `pb2025_nav_bringup/rviz/*.rviz` 都已按此配置。

## 2026 正式模型（`chassis_model:=sentry_2026`）

来源：`哨兵总装_urdf.zip`（SolidWorks 导出：`sentry.urdf` + 6 个 STL）。
原始导出只有底盘上板和四轴云台总成，**没有车轮**，因此本包把它作为静态
TF/可视化描述，固定关节角为 0。

只保留导航栈实际使用的坐标系，其余链路帧按导出原样保留：

| 导出里的 link | 本包里的名字 | 谁在用 |
|---|---|---|
| `base` | `base_link` | 底盘本体 |
| —（新增） | `base_footprint` | Nav2 `base_frame`、slam_toolbox、small_gicp |
| `link1_big_yaw` | `gimbal_yaw` | Nav2 `robot_base_frame`、fake_vel_transform、teleop、pursuit |
| `ref_lidar` | `front_mid360` | Livox `frame_id`、sensor_frame、loam / small_gicp |

TF 链：

```text
base_footprint -> base_link -> gimbal_yaw -> front_mid360
base_footprint -> base_link -> imu_link
base_footprint -> base_link -> ref2
gimbal_yaw -> link3_roll -> link4_small_yaw -> link5_pitch
gimbal_yaw -> link2_laser -> front_mid360
```

关键静态变换（原样取自导出）：

| 变换 | xyz (m) | rpy (rad) |
|---|---|---|
| `base_link -> gimbal_yaw` | 0, 0, 0.0901 | 0, 0, 0 |
| `link2_laser -> front_mid360` | 0.06028, -0.11428, 0.11386 | **0.383972**, 0, 0 |
| `base_link -> front_mid360`（合成） | 0.06028, -0.11428, 0.20396 | 0.383972, 0, 0 |

- 雷达在导出里带 **22° pitch**，不是水平安装；TF 已照实保留。
- 假设 CAD `base` 原点在地面：此时雷达到地面 0.204 m，与旧车实测 0.21 m
  接近。若实车安装高度不同，只改 `base_footprint_to_base_link` 的 z。
- `gimbal_yaw` 固定不动（暂无云台反馈），`gimbal_yaw_fake` 仍由
  `sentry_navigation/fake_vel_transform` 运行时发布。
- `imu_link` 位置未实测，暂时与 `base_link` 重合（usb_bridge 的 `/imu/data`
  用这个 frame）。

mesh 在 `meshes/sentry_2026/`，通过
`package://sentry_description/meshes/sentry_2026/...` 引用。
原始导出 `urdf/sentry_2026_export.urdf` 一并保留（它用相对 mesh 路径，
只作对照，不直接加载）。

## 备用：训练车麦轮模型（`chassis_model:=mecanum`）

`urdf/mecanum_static.urdf`：四个轮子为 `continuous`（由 USB 桥的
`serial/joint_states_raw` 驱动），`base_link`、`gimbal_yaw`、`imu_link` 为 fixed：

| 项 | 值 |
|---|---:|
| 轮半径 | 0.045 m |
| 轮宽 | 0.049 m |
| 轮心 x | ±0.1716 m |
| 轮心 y | ±0.1362 m |
| 底盘外形 | 0.450 × 0.250 × 0.086 m |
| `base_link` 高度 | 0.045 m（轮轴） |
| MID360 x / y | 0.15 / 0 m |
| MID360 离地 z | **0.21 m** |
| MID360 yaw | **0**（X 轴朝车头；实测原 -π/2 会把物理前方映射到右扇区） |

TF 链：

```text
base_footprint -> base_link -> gimbal_yaw -> front_mid360
base_footprint -> base_link -> imu_link
base_footprint -> wheel_front_left / wheel_front_right / wheel_back_left / wheel_back_right
```

- 当前车没有云台，`gimbal_yaw` 是 `base_link` 的固定子帧，仅用于兼容 Nav2
- 所有几何参数集中在 `urdf/mecanum_static.urdf` 顶部注释，实测后直接改数值

## 备用：旧实车模型（`chassis_model:=sentry`）

`urdf/robot.urdf` 是四舵轮底盘 + 两级云台（大 yaw → 小 yaw → pitch），
12 个带几何的 link + 7 个无质量的 `ref_*` 接口坐标系，共 19 link / 18 joint。
mesh 在 `meshes/`，通过 `package://sentry_description/meshes/...` 引用。

导出的三个关键改名必须保持：`base` → `base_footprint`、
`link9_big_yaw` → `gimbal_yaw`、`ref_ref4` → `front_mid360`。

## 关节数据接口

`config/robot_state_publisher.yaml` 给 `joint_state_publisher` 配了：

```yaml
source_list: ["serial/joint_states_raw"]
```

2026 模型全部是 fixed joint，不依赖该话题；麦轮模型的轮子会消费它。
