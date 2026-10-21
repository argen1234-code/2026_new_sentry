# sentry_description

机器人静态描述包。默认加载**当前训练车的静态麦轮模型**
`urdf/mecanum_static.urdf`；通过 `chassis_model:=sentry` 可切换回原实车
四舵轮 + 两级云台模型 `urdf/robot.urdf`。

两个模型都是普通 `.urdf`，由 launch 直接读取，不经过 xacro。

```bash
colcon build --symlink-install --packages-up-to sentry_description
source install/setup.bash

# 当前训练车（默认）
ros2 launch sentry_description description.launch.py use_rviz:=true

# 旧实车舵轮 + 云台模型
ros2 launch sentry_description description.launch.py use_rviz:=true chassis_model:=sentry
```

## 当前训练车：静态麦轮模型

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
| MID360 yaw | -π/2（初值来自原麦轮模型，需在 RViz 验证） |

TF 链：

```text
base_footprint -> base_link -> gimbal_yaw -> front_mid360
base_footprint -> base_link -> imu_link
base_footprint -> wheel_front_left / wheel_front_right / wheel_back_left / wheel_back_right
```

- 当前车没有云台，`gimbal_yaw` 是 `base_link` 的固定子帧，仅用于兼容 Nav2
- `gimbal_yaw_fake` 仍由 `sentry_navigation/fake_vel_transform` 运行时发布
- 所有几何参数集中在 `urdf/mecanum_static.urdf` 顶部注释，实测后直接改数值

## 实车模型（`chassis_model:=sentry`）

`urdf/robot.urdf` 是四舵轮底盘 + 两级云台（大 yaw → 小 yaw → pitch），
12 个带几何的 link + 7 个无质量的 `ref_*` 接口坐标系，共 19 link / 18 joint。
mesh 在 `meshes/`，通过 `package://sentry_description/meshes/...` 引用。

导出时做过的改名——**这三个名字必须保持**，导航栈各处按它们引用：

| URDF 里的 link | 改名后 | 谁在用 |
|---|---|---|
| `base` | `base_footprint` | rviz Fixed Frame、nav2 base_frame、slam / small_gicp |
| `link9_big_yaw` | `gimbal_yaw` | nav2 robot_base_frame、fake_vel_transform、teleop、pursuit 控制器 |
| `ref_ref4` | `front_mid360` | 雷达驱动 frame_id、nav2 sensor_frame、loam / small_gicp |

关键坐标系链路：

```text
map -> odom -> base_footprint -> gimbal_yaw -> front_mid360
                             └─ link2_wheel1_steer -> link1_wheel1_drive   (×4 舵轮)
gimbal_yaw -> link11_small_yaw -> link10_pitch
```

`gimbal_yaw_fake` 不来自模型，由 `sentry_navigation/fake_vel_transform` 发布。

## 关节数据接口

`config/robot_state_publisher.yaml` 给 `joint_state_publisher` 配了：

```yaml
source_list: ["serial/joint_states_raw"]
```

下位机桥往这个话题发真实关节角度即可，关节名见对应 URDF。
当前麦轮模型全部是 fixed joint，不依赖该话题；没有数据时不会影响静态 TF。

## 保留的接口坐标系

除三个关键帧外，旧实车模型自带的这些坐标系全部保留，留给后续接真实传感器：

`link11_small_yaw`、`link10_pitch`、`ref_ref1`（base）、`ref_ref2`、
`ref_ref3`（IMU，挂 gimbal_yaw）、`ref_ref5`、`ref_ref6`（相机，挂 pitch）、
`ref_ref7`，以及 8 个舵轮 link。
