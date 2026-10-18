# sentry_description

实车使用的机器人描述包。直接加载 `urdf/robot.urdf`（由 SolidWorks 装配体经
OVO-URDF 导出），交给 `robot_state_publisher` 发布模型 TF，并由
`joint_state_publisher` 驱动关节。

```bash
colcon build --symlink-install --packages-up-to sentry_description
source install/setup.bash
ros2 launch sentry_description description.launch.py use_rviz:=true
```

## 模型

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

```
map -> odom -> base_footprint -> gimbal_yaw -> front_mid360
                             └─ link2_wheel1_steer -> link1_wheel1_drive   (×4 舵轮)
gimbal_yaw -> link11_small_yaw -> link10_pitch
```

`gimbal_yaw_fake` 不来自模型，由 `sentry_navigation/fake_vel_transform` 发布。

## 关节数据接口

`config/robot_state_publisher.yaml` 给 `joint_state_publisher` 配了

```yaml
source_list: ["serial/joint_states_raw"]
```

下位机桥往这个话题发真实关节角度即可，关节名见 `urdf/robot.urdf`。
话题没人发时所有关节默认为 0——即云台相对底盘不转（底盘 yaw 跟随大 yaw 的简化）。
以后接真实角度反馈不需要动 URDF。

## 保留的接口坐标系

除三个关键帧外，新模型自带的这些坐标系全部保留，留给后续接真实传感器：

`link11_small_yaw`、`link10_pitch`、`ref_ref1`（base）、`ref_ref2`、
`ref_ref3`（IMU，挂 gimbal_yaw）、`ref_ref5`、`ref_ref6`（相机，挂 pitch）、
`ref_ref7`，以及 8 个舵轮 link。
