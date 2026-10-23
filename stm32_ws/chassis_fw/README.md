# chassis_fw —— 全向轮底盘最小固件

只做两件事：

1. **全向轮驱动**：照搬 `sentinel/` 的底盘代码（`Omni_chassis.c`、
   `chassis_task.c`、`chassis_calculate.c`、`bsp_fdcan.c`、`pid.c`、
   `user_lib.c`），通过 FDCAN1 驱动 4 个 M3508；
2. **与当前 PC 通信**：复用 `sentinel/` 的 USB OTG_HS CDC 外设，按旧
   `usb_bridge` 的 `AA 55` 协议收发，ROS 侧无需任何改动。

云台、IMU、遥控器、功率计、uproto 全部不参与编译。底盘控制范围是
**速度直驱（no-follow）**：不依赖姿态，也不做航向闭环。

## 目录

| 路径 | 来源 | 说明 |
|---|---|---|
| `App/chassis/` | `sentinel/User/APP{, _Support}/chassis` | 底盘任务与全向轮运动学，仅改 `__packed` 写法 |
| `App/bsp/bsp_fdcan.c` | `sentinel/User/BSP` | FDCAN1/2/3 初始化、0x205–0x208 反馈、0x200 电流下发 |
| `App/algorithm/` | `sentinel/User/Algorithm` | PID 与斜坡/滤波工具 |
| `App/common/` | `sentinel/User` | `robot_param.h`、`struct_typedef.h` |
| `App/comm/` | 新写 | 旧协议解析（复用 `legacy_proto.c`）、USB 接收环形缓冲 |
| `App/stubs/` | 新写 | 云台 / IMU / 遥控器 / 检测任务的最小桩 |
| `App/main.c`、`App/freertos_app.c` | 新写 | 精简入口与两个 FreeRTOS 任务 |
| `USB_DEVICE/` | `sentinel/USB_DEVICE` | CubeMX 生成的 CDC 设备，去掉未使用的 `crc8_crc16.h` |
| `Core/`、`Drivers/`、`Middleware/` | 不复制 | 构建时直接引用 `../sentinel/` 下的生成代码 |
| `startup_stm32h723xx.s`、`*.ld`、`cmake/` | `legacy/Boards` | 同型号 MCU 的 GCC 启动文件、链接脚本与工具链配置 |

> 外设初始化代码（`sentinel/Core/Src`、HAL、USB 库、FreeRTOS 内核）**不复制**，
> 构建时直接编译 `../sentinel/` 下的原文件，避免两份 `.ioc` 输出漂移。
> 唯一替换的是 `main.c` 与 `freertos.c`。

## 控制链路

```
ROS /cmd_vel
  -> cmd_vel_to_unified (50 Hz, CMD_VALID | CMD_MOTION_ENABLE, mode=2)
  -> USB CDC (AA 55 0x10)
  -> legacy_bridge: 100 ms 新鲜度 + 500 ms 链路检测
  -> chassis_behaviour（桩）: 有效 -> CHASSIS_VECTOR_SPIN，无效 -> NO_MOVE
  -> chassis_set_contorl / chassis_control_loop（S 曲线 + 速度环 + 限流）
  -> FDCAN1 0x200 -> 4 x M3508
```

使用上游 `CHASSIS_VECTOR_SPIN` 分支是因为它是唯一不做角度闭环、直接采用
`wz_set` 的模式；行为层桩替换了上游的 `chassis_spin_control()`，所以
`wz` 来自主机而不是固定常值。上位机命令为 `mode=2`（NoFollow）时才使能，
其余模式按停车处理。

上行按旧格式回发：

- `0x81` MCU 状态：姿态/云台字段为 0，`status=0x01`；
- `0x83` 四轮状态：速度、输出轴角度、温度、力矩；
- `0x84` IMU 状态：全 0，`valid_mask=0`（本工程无 IMU）；
- `0x82` 心跳。

## 坐标系与底盘参数

主机侧严格按 ROS REP-103：x 向前、y 向左、z 向上，wz 逆时针为正。

2026-10-23 实车标定：相对上游内部系（x 向右、y 向前），本车需要把平移量
**顺时针旋转 90°**，落地为

```c
rear  = -vx + r*wz    right = +vy - r*wz
front = -vx - r*wz    left  = +vy + r*wz
```

`chas_for_cal` 对应 `vx = -½(rear+front)`、`vy = ½(right+left)`。

同一天用 MID360 陀螺仪（与 LIO 无关的独立信源）验证自转：下发 `+1.0 rad/s`
时 `gyro.z` 为负，车体实际顺时针转——轮端分布 `(+,-,-,+)` 对，但整体符号
反了，所以 `wz_speed = -wz_set * r`，`chas_for_cal` 的 `wz` 同步取负。
`App/chassis/Omni_chassis.c` 里前馈与制动补偿的两处 `wheel_force[]` 也用同一
套映射（偏航力矩前馈一并取负），改一处必须同步另一处。

wz 的符号集中在：

- `App/comm/host_cmd.h` 的 `HOST_CHASSIS_WZ_SIGN`（当前 `+1.0f`），
  下行 setpoint 和 `0x81` 回传的 wz 都乘它。自转方向已在
  `chassis_calculate.c` 里按实测修正，这个系数保持 +1；换底盘后若又反了，
  只改这一个值即可。

底盘几何参数全部来自 `App/common/robot_param.h`，**目前是上游英雄车
十字全向轮的数值，不是训练麦轮车的数值**：

| 参数 | 当前值 | 说明 |
|---|---|---|
| `ROBOT_CHASSIS` | `ROBOT_CHASSIS_OMNI` | 十字全向，非麦轮 |
| `CHASSIS_HALF_WHEELBASE` | 0.28 m | 旋转半径 |
| `Wheel_Radius` | 0.075 m | 轮半径 |
| `MPS_to_RPM` | 2445.72 | 已含 3591/187 减速比 |
| `MAX_WHEEL_SPEED` | 5.2 m/s | 单轮限幅 |
| 反馈 ID / 广播 ID | 0x205–0x208 / 0x1FF | M3508 + C620 |
| 方向系数 205/206/207/208 | +1 / -1 / -1 / +1 | 与电机安装方向绑定 |

换成正式底盘（或别的轮径/轮距）时，按实测改这张表，并同步
`chas_inv_cal` / `chas_for_cal` 的轮位对应关系。训练车（DMH6215 麦轮、
0.045 m 轮、±0.1716/±0.1362）的参数在 `legacy/` 里，不能直接套用。

`0x83` 的轮位顺序是上游内部顺序 **[后 205, 右 206, 前 207, 左 208]**，
不是 ROS 侧 `usb_bridge_node.py` 的 `[RR, FL, RL, FR]`。2026 正式底盘
URDF 没有轮关节，这组 joint state 仅用于调试显示；给训练车看轮速时
需要另做映射。

## 构建

```bash
cd stm32_ws/chassis_fw
cmake -S . -B build -G Ninja \
      -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
      -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

产物：`build/chassis_fw.elf` / `.hex` / `.bin`。

## 烧录（CMSIS-DAP）

```bash
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
        -c "program build/chassis_fw.elf verify reset exit"
```

## 键盘控制

不开 ROS 也能直接测底盘（脚本自己按 `UnifiedCmd` 发帧）：

```bash
python3 ros_ws/scripts/chassis_keyboard_teleop.py --port /dev/ttyACM0
```

| 键 | 动作 |
|---|---|
| `W` / `S` | 前进 / 后退 |
| `A` / `D` | 左移 / 右移 |
| `Q` / `E` | 顺时针 / 逆时针 |
| `+` / `-` | 所有轴倍率 +0.1 / −0.1（0.1x–2.0x，实时生效） |
| `空格` | 立即停 |
| `Ctrl-C` | 停止并退出 |

默认 `vx=0.40 m/s`、`vy=0.30 m/s`、`wz=1.00 rad/s`，可用 `--vx/--vy/--wz`
调整。按住方向键移动（依赖系统按键重复），松开约 0.6 s 后自动停；
`--hold` 可改这个窗口。运行前先停掉 `usb_bridge` 节点，否则串口被占用。

底盘仍在台架/低速试车阶段时，用 `-` 把整体倍率降下来再动，确认方向无误后
再逐步加。注意 `+`/`-` 改动的是三个轴同一个倍率。

## 上电验证顺序

1. 只上 USB，`ros2 run usb_bridge usb_bridge_node` 应持续收到
   `0x81/0x82/0x83/0x84`，且每类帧的 `seq` 连续；
2. `enable_motion:=false` 时四轮保持静止（`0x83` 力矩为 0）；
3. `enable_motion:=true` 后低速给 `/cmd_vel`，确认前进/左移/自转方向与
   `base_link` 一致；
4. 拔掉 USB 后 500 ms 内底盘应自动停转。

## 与上游的差异

- `__packed` 改为 GCC 的 `struct __attribute__((packed))`，`__weak` 在
  `struct_typedef.h` 中映射；
- 0x83 的轮位置输出**减速箱后的轮轴角度**（`ecd/8192*2π/19.2032`），
  与 URDF 轮关节一致；上游兼容通道输出的是转子角。注意它仍是**单圈**
  值（每圈在 0.327 rad 处回绕），没有跨圈累计，只适合调试显示；ROS 侧
  `wheel_position_wrap_rad` 的 25 rad 解缠对它是无效的；
- 无云台 / IMU / 遥控器 / 功率限制，相关桩集中在 `App/stubs/`；
- 运动学按主机系（+x 向前、+y 向左），平移量为上游内部系顺时针 90°
  （前后轮组吃 -vx、左右轮组吃 +vy）；wz 只剩 `HOST_CHASSIS_WZ_SIGN`
  一个符号（当前 `+1.0f`）；上游兼容通道没有做这步换算；
- `App/main.c` 替换了上游 `Core/Src/main.c`，因此必须自带
  `HAL_TIM_PeriodElapsedCallback()`：HAL 的 1 ms 时基由 TIM23 提供，
  缺失它会让 `HAL_GetTick()` 恒为 0，USB 初始化里的 `HAL_Delay()` 死等；
- `USB_DEVICE/App/usbd_cdc_if.c` 修正了上游发送环形缓冲的出队记账：
  完成回调改为按“已提交字节数”出队。上游按当时的 `usb_tx_count` 重新
  推算，会把传输期间新入队的帧一并丢弃（实测只剩 `0x81` 能上来）。
