# 底盘速度与方向参考

> **适用范围（重要）**：本文只对**当前这一台底盘 + 当前 `stm32_ws/chassis_fw` 固件**有效。
> 更换底盘、改变雷达安装位置或角度、或修改固件中的底盘标定（`chas_inv_cal`、`chas_for_cal`、
> `CHASSIS_OMNI_ROTATE_RADIUS`）之后，本文的速度与方向结论全部作废，必须重新实测。
>
> 最后实测：2026-10-23，使用 `ros_ws/scripts/chassis_keyboard_teleop.py` 直连下位机。

## 1. 常用速度（直接照用）

| 场景 | vx (m/s) | vy (m/s) | wz (rad/s) | 说明 |
|---|---:|---:|---:|---|
| 键盘直控 / 起步测试 | 0.40 | 0.40 | 0.80 | 推荐值，能可靠前进、平移、自转 |
| 略慢 | 0.30 | 0.30 | 0.60 | 接近下限，可能间歇性不走 |
| 略快 | 0.50–0.60 | 0.50 | 1.00–1.20 | 场地内测试用 |

```bash
cd ~/2026_new_sentry/ros_ws
python3 scripts/chassis_keyboard_teleop.py --vx 0.40 --vy 0.40 --wz 0.80
```

**太小车不动是已知现象，不是故障。** 实测 vx 约 0.25 m/s 以下基本不动，超过后
稳态轮速也只有目标的约 35%（详见 [`pitfalls.md`](pitfalls.md) 第 2 条）。
因此 `vx/vy` 建议不低于 0.40，`wz` 建议不低于 0.80。

## 2. 方向约定（重点）

以下“顺时针/逆时针”统一按**俯视车体、从车顶正上方往下看**定义：

```text
              +x（车头）
                 ^
                 |
   逆时针 (+wz)  |  顺时针 (-wz)
                 |
   <-------------+------------->
                 |
              -x（车尾）
```

| 操作 | 下发的控制量 | 现实转向（2026-10-23 实测） |
|---|---|---|
| 键盘 `Q` | `wz > 0`（+0.80 rad/s） | **逆时针**（车头向左转） |
| 键盘 `E` | `wz < 0`（-0.80 rad/s） | **顺时针**（车头向右转） |

对应关系：

- **+wz = 逆时针 = 数学正方向**，与 ROS `geometry_msgs/Twist.angular.z` 的
  REP-103 约定一致。ROS 侧不要再取负。
- **-wz = 顺时针**。
- 固件 `chas_inv_cal` 内部已有一次取负
  （`wz_speed = -wz_set * CHASSIS_OMNI_ROTATE_RADIUS`），那是为了抵消电机安装方向，
  上层看到的就是上表的关系，**不要重复补偿**。
- 平移：`+vx` 车头方向，`+vy` 车体左侧；键盘 `W/S` 对应 `±vx`，`A/D` 对应 `±vy`。

## 3. 实车运动方向反馈（以雷达安装方向为基准）

2026-10-23 直连键盘实测。基准 0° = **雷达安装方向**，俯视、逆时针为正。

| 按键 | 控制量 | 现实运动方向 |
|---|---|---|
| `W` | `+vx` | 雷达方向**逆时针 135°** |
| `S` | `-vx` | 雷达方向**顺时针 45°**（等价逆时针 315°） |
| `A` | `+vy` | 雷达方向**顺时针 135°**（等价逆时针 225°） |
| `D` | `-vy` | 雷达方向**逆时针 45°** |

一致性检查：`W`/`S` 相差 180°，`A`/`D` 相差 180°，`W`/`D` 相差 90°，四向构成正交系。
等价描述：**车体系相对雷达安装方向整体逆时针偏 135°**（或雷达相对车体顺时针偏 135°）。

安装说明（用户描述）：雷达放置在**两个轮子之间**，两轮相对雷达轴线各偏 45°（45°/45°）。

使用含义：

- 键盘直控适合验证链路、使能、转向和轮速，**不要用它判断“车头朝向”**：
  按 `W` 车不会沿雷达正前方走，而是偏向逆时针 135°。
- ROS 导航链路的平移方向由 `cmd_vel_to_unified` 的 `frame_yaw` 修正，与本节直连实测
  不是同一条路径；调整 `frame_yaw` 前必须重新实测，不能直接套用本表。

## 4. 键盘直控与 ROS 链路的差别

| 链路 | 路径 | 平移补偿 |
|---|---|---|
| 键盘直控 | 脚本 → USB CDC → 固件 | **无** `frame_yaw` 补偿 |
| ROS 导航 | `/cmd_vel` → `cmd_vel_to_unified` → USB → 固件 | `frame_yaw = -45°`（见 `cmd_vel_to_unified.py`） |

因此：

- 键盘直控适合验证**链路、使能、转向、轮速**。
- 按 `W` 走斜是预期现象，具体角度见第 3 节，**不代表底盘故障**；
  全局平移方向以 ROS 导航链路为准。
- 判断转向方向时优先看自转（`Q/E`），不受该补偿影响。

## 5. 协议单位与超时

| 字段 | 单位 | 换算 |
|---|---|---|
| `vx_mm_s` | mm/s | `m/s × 1000` |
| `vy_mm_s` | mm/s | `m/s × 1000` |
| `wz_mdeg_s` | 0.001 °/s | `rad/s × 180000 / π` |

- 下发频率 50 Hz（20 ms）。
- 固件在 100 ms 内收不到命令即停（`LEGACY_BRIDGE_CMD_TIMEOUT_MS`）。
- 驱动底盘需要 `chassis_status` 同时置 `CMD_VALID(0x0001) | CMD_MOTION_ENABLE(0x0002)`，
  且 `chassis_mode = 2`（NoFollow）。键盘脚本会自动置位；ROS 侧对应
  `enable_motion:=true`。

## 6. 相关命令

```bash
# 串口识别（应看到 0483:5740 -> /dev/ttyACMn）
python3 scripts/chassis_keyboard_teleop.py --list-ports

# 不动机器，只看按键与帧
python3 scripts/chassis_keyboard_teleop.py --dry-run

# 实车直控
python3 scripts/chassis_keyboard_teleop.py --vx 0.40 --vy 0.40 --wz 0.80

# ROS 导航链路（需要显式开运动门控）
ros2 launch sentry_bringup rviz_navigation.launch.py slam:=false enable_motion:=true
```

## 7. 依据

- 键盘脚本：[`ros_ws/scripts/chassis_keyboard_teleop.py`](../ros_ws/scripts/chassis_keyboard_teleop.py)
- 协议与状态帧：[`ros_ws/src/sentry_hardware/usb_bridge/usb_bridge/protocol.py`](../ros_ws/src/sentry_hardware/usb_bridge/usb_bridge/protocol.py)
- 固件命令门控：[`stm32_ws/chassis_fw/App/comm/legacy_bridge.c`](../stm32_ws/chassis_fw/App/comm/legacy_bridge.c)
- 速度环死区与转向修正历史：[`docs/pitfalls.md`](pitfalls.md)
