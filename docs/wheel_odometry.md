# 外置轮式里程计：使用说明、精度与清零

> **适用范围**：车上的**外置里程计模块 + 达妙 DM-MC02 控制板（STM32H723）+ 当前 `chassis_fw`**。
> 更换模块、改接线或改安装位置后，本文的偏置、波特率与精度结论需要重新确认。
>
> 参考实现：[`reference/工创底盘里程计例程/`](../reference/工创底盘里程计例程/)
> （Keil MDK 工程，CubeMX 工程名 `CtrlBoard-H7_WS1812`，与本车控制板同款）。
> 首次实测：2026-11-12，车辆静止，仅验证接收链路。

## 1. 当前状态（2026-11-12）

| 项 | 状态 |
|---|---|
| UART7 接收 | **已实现并验证**（`stm32_ws/chassis_fw/App/bsp/bsp_odom_uart.c`） |
| 帧解析 / CRC | 已移植参考工程逻辑，实测全部通过 |
| 清零指令下发 | **已实现并验证**（首帧后 2 s 触发，见 §4.1） |
| 接入底盘控制 | **未接入**（只上报，不参与 `chassis_task` 控制环） |
| 上报给 ROS | **已接入**：`LEGACY_ID_ODOM_STATE`(`0x85`) → `usb_bridge` → `/wheel_odom` |
| 参与定位 | **已接入**：`wheel_odom_fusion` 用轮式预测 + LIO 校正发布 `odom→base_footprint`，默认关闭，`use_wheel_fusion:=true` 开启（见 §10） |

## 2. 接线与通信参数

| 项 | 值 |
|---|---|
| 串口 | UART7（AF7） |
| 控制板引脚 | **PE7 = UART7_RX**，**PE8 = UART7_TX** |
| 接线 | 模块 TX → PE7；模块 RX → PE8；**必须共地** |
| 电平 | 3.3 V TTL（不要接 5 V 电平） |
| 波特率 | **921600**，8 数据位、无校验、1 停止位、无流控 |
| 接收方式 | DMA 空闲中断（`HAL_UARTEx_ReceiveToIdle_DMA`），RX DMA = `DMA2_Stream0` |
| 缓冲位置 | RAM_D1（`0x24000000`），**不能放 DTCM——H7 的 DMA 访问不到 DTCM** |

> 控制板把 UART7 同时留给 sentinel 的 HWT101 陀螺仪（115200），所以 `chassis_fw` 采用
> **运行时覆盖波特率**（先调 `MX_UART7_Init()`，再把 `huart7.Init.BaudRate` 改为 921600
> 并重新 `HAL_UART_Init()`），不改 sentinel 的 `.ioc` 和生成代码。

## 3. 数据帧格式（模块 → 控制板，28 字节）

| 偏移 | 长度 | 字段 | 类型 | 单位 | 说明 |
|---:|---:|---|---|---|---|
| 0 | 1 | 同步头 | `0xA5` | — | |
| 1 | 1 | 同步头 | `0x5A` | — | |
| 2 | 4 | x | float32 LE | mm | 相对模块清零点的 X |
| 6 | 4 | y | float32 LE | mm | 相对模块清零点的 Y |
| 10 | 4 | yaw | float32 LE | deg | **绝对航向**（模块上电起算） |
| 14 | 4 | gyro | float32 LE | deg/s | 角速度 |
| 18 | 4 | body_vx | float32 LE | mm/s | 车体系前向速度 |
| 22 | 4 | body_vy | float32 LE | mm/s | 车体系左向速度 |
| 26 | 2 | crc16 | uint16 LE | — | 覆盖字节 0..25 |

- CRC：**CRC-16/Modbus**，初值 `0xFFFF`，反射多项式 `0xA001`，无最终异或，按位计算。
- 车体系约定：`vx` 向前、`vy` 向左；`wz` 与里程计 yaw 正方向一致。
- 安装点相对车体中心偏置：**X = +14.5 mm，Y = 0 mm**（参考工程常量）。换算公式：
  `center = mount - R(相对yaw) · offset`，其中相对 yaw = `wrap(yaw − 清零时 yaw)`。
  注意：**XY 清零不改变 yaw**，所以偏置旋转必须用相对航向。

## 4. 清零指令（模块 ← 控制板）

```
A5 5A C3 00 | crc_lo crc_hi        （共 6 字节）
```

- `crc16` 覆盖前 **4** 个字节（`A5 5A C3 00`），低字节在前。
- 参考工程用阻塞发送：`HAL_UART_Transmit(&huart7, frame, 6, 20)`。
- 参考工程的上电流程（`mecanum_chassis.c:245-279`）：
  1. 上电后等待 **11 000 ms**（`MECANUM_PATH_ODOM_INIT_WAIT_MS`，“上电后里程计稳定等待时间”）；
  2. 发送清零帧；
  3. 等待 **100 ms**（`MECANUM_PATH_ODOM_SETTLE_MS`）并确认收到新帧；
  4. 丢弃清零前的有效性标志与坐标原点，重新记录首帧为原点。
- **清零的语义（依据参考工程注释）**：清零的是位置 XY，**不清航向**。航向是模块上电后的
  绝对角，需要在软件里自行取偏移；若需要“航向归零”，只能断电重启模块或在软件侧做差。

### 4.1 本固件的实现（2 s，首帧锚点）

参考工程的 11 s 是从 **MCU 复位**起算的固定延时，模块比主控晚上电时会被提前清零。
本固件把锚点改到**首帧到达**，等待压到 2 s：

1. 中断里解析到第一帧 CRC 通过的数据时记 `s_zero_boot_ms`（此前为 0，表示“未武装”）；
2. `bsp_odom_uart_tick()` 在 `ODOM_ZERO_STATE_WAIT` 里检查：**已收到首帧**且
   `now - s_zero_boot_ms >= ODOM_ZERO_WAIT_MS`(2000) 才发 `C3`；
   期间若帧间隔超过 `ODOM_TIMEOUT_MS`(50) 就把锚点后移到当前时刻，
   保证是“**连续** 2 s 有数据”才清零，不会向刚好断掉的链路发清零帧；
3. 发帧后 100 ms（`ODOM_ZERO_SETTLE_MS`）检查 `seq` 是否变化，未变则重发，
   最多 `ODOM_ZERO_MAX_RETRY` 次，超限进入 `FAILED`；
4. 只失效 XY 原点，`yaw` 继续用模块绝对角。

选 2 s 的依据：模块本身 2 s 即稳定；比赛允许赛前提前上电，车静止时清零在数据上不可见
（两个原点重合，yaw 从不清零）。

**实测时序（2026-11-12，OpenOCD 读 `uwTick` 与 `s_zero_boot_ms`）**：

| 复位后 | `uwTick` | 首帧 tick | `zero_state` | `zero_sent` |
|---|---|---|---|---|
| 1.0 s | 999 | 622 | WAIT(0) | 0 |
| 2.2 s | 2200 | 622 | WAIT(0) | 0 |
| 3.0 s | 3003 | 622 | **DONE(2)** | **1** |

清零命中时刻 = 首帧 + **2000 ms**（精确到 1 ms）。注意 2 s 是“首帧之后”，
**MCU 复位到 UART 就绪还要 0.44–0.62 s**（两次复位实测 443 / 622 ms，与启动耗时有关），
所以“复位 → 清零完成”端到端约 **2.45–2.72 s**。要压到 2 s 以内就把
`ODOM_ZERO_WAIT_MS` 降到约 1400。

**已知限制**：模块**运行中**掉电重启不会再次清零（`s_origin_valid` 仍为 1），
此时 x/y 会整体跳变，由 ROS 侧 `wheel_odom_fusion` 的连续跳变检测重锚定；
`ODOM_ZERO_RETRY_MS` 在 `robot_param.h` 里有定义但**从未使用**，实际重试间隔由
`ODOM_ZERO_SETTLE_MS` 决定。

## 5. 使用方法

### 5.1 参考工程的做法（闭环路径跟踪）

- 上电 11 s → 清零 → 100 ms 稳定 → 把当前 yaw 作为巡航航向 → 位置环（PID）以
  `odometer_x/y` 为反馈、`body vx/vy` 为速度反馈，最大平移 0.35 m/s，到点容差 3 mm / 1°。
- 这意味着参考工程认为里程计的**短时位置分辨率足够做毫米级闭环**。

### 5.2 我们固件的用法

- 初始化：`bsp_odom_uart_init()` 在 `App/main.c` 中调用，位于 `osKernelInitialize()` 之前。
- 读最新值：`bsp_odom_uart_get(&state)`，唯一消费者是 `legacy_bridge.c` 的 `0x85` 上报
  （`/chassis/state` 与 `/wheel_odom` 的数据源）。
- 读调试计数（OpenOCD，本机版本 `mdw` 不输出，用 `read_memory` + `puts stdout`）：

```bash
cd stm32_ws/chassis_fw
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
  -c "init; reset halt; resume; sleep 3000; halt; \
      puts stdout \"SNAP [read_memory 0x24000000 32 20]\"; resume; shutdown"
```

字段顺序（共 20 字）：

```
magic, rx_events, rx_bytes, frames_ok, crc_err, sync_miss, uart_errors,
rearm_fail, last_rx_ms, x_mm, y_mm, yaw_deg, gyro_dps, vx_mms, vy_mms, valid, seq,
zero_state, zero_sent, zero_fail
```

后 6 个 float 是 IEEE-754；`magic` 应为 `0x4F444F4D`。
`zero_state` 取值 `0=WAIT` / `1=SETTLE` / `2=DONE` / `3=FAILED`。
静态清零相关变量也可直接读：`s_zero_boot_ms`(`0x20007d30`)、`s_zero_last_frame_ms`、
`s_zero_state`(`0x20007d44`)、`uwTick`(`0x200079a4`)，地址以 `nm build/chassis_fw.elf` 为准。

### 5.3 常见判读

| 现象 | 含义 |
|---|---|
| `rx_bytes/rx_events/frames_ok/seq` 递增 | 链路正常 |
| `rx_bytes` 增但 `crc_err` 同步增 | 波特率/电平/协议不匹配 |
| 全 0 且 `magic` 正常 | 模块没发数据（供电、TX 未接 PE7、波特率） |
| `uart_errors` 持续增长 | 帧错误/溢出，查共地与电平 |
| `rearm_fail` 增长 | DMA 未重挂，接收会停 |

## 6. 上电是否需要校准

**参考工程没有做任何显式校准**（全工程搜不到 gyro 零偏/标定代码），它只做了两件事：

1. **上电等待 11 s** 再清零——说明模块上电后需要一段稳定时间；
2. **清零后等 100 ms** 再采信数据。

据此得出的实践建议：

- 上电时车必须**静止、无外力**，否则清零会把错误位置当成原点；
- 参考工程的 11 s 是保守值：模块 2 s 即稳定（用户确认），本固件已按 **2 s + 首帧锚点**
  实现并实测通过（§4.1）；**没有再缩短**，也不再使用 11 s；
- 清零后读到的 x/y 是**车体原点**相对量，不是模块开机原点；
- 静止时航向会缓慢漂移（§7.2 实测 ≈0.003 °/s），融合时由 LIO 拉回，不需要单独补偿；
- **上电后 0.44–0.62 s 内没有任何里程计数据**（MCU 启动到 UART 就绪），这段要由 LIO 或
  `wheel_odom_fusion` 的纯 LIO 模式兜住。

## 7. 精度

### 7.1 协议层面的分辨率

- 位置单位 mm、角度单位 deg，均为 float32；数值分辨率远小于 1 mm（1 m 处 float32 约
  0.06 µm），**但真实精度取决于模块内部编码器/陀螺仪，协议不携带精度、时间戳或置信度**。

### 7.2 实测记录（2026-11-12，车辆静止）

| 指标 | 实测值 |
|---|---|
| 稳态帧率 | **≈1005 帧/s**（28 KB/s，921600 下占空比约 24%） |
| 断帧/CRC 错 | 1 s 内 3388 个事件、3391 帧，`crc_err=0`、`sync_miss=0` |
| 重挂失败 | 0 |
| 串口错误 | 0–1 次（仅出现在复位后启动阶段，之后不增长） |
| 静止位置漂移 | x 恒为 0.000 mm；y 约 −0.0016 → −0.0034 mm（1 s 内 ≈0.002 mm） |
| 静止航向漂移 | yaw 0.0062° → 0.0135°（1 s 内 ≈+0.007°，约 **0.4°/min**） |
| 静止 gyro 读数 | −0.000 ~ +0.135 deg/s 抖动 |

补充说明：

- 复位后**第 1 秒只有约 379 帧**，第 2 秒起恢复满速；现已查明原因：MCU 启动到 UART 就绪
  需要 0.44–0.62 s（§4.1），这段时间完全收不到帧，之后即满速。**不是模块的启动瞬态**。
- `x` 在所有快照中都是精确的 `0.000000`，而 `y` 有小幅漂移，这个不对称值得后续单独确认
  （可能只是模块内部 x 通道恰好为零，也可能是 x 字段未更新）。
- **运动精度**已在 2026-11-12 15:19 标定包（`odom_cal_20261112_151911`，27.3 s，闭环运动）中
  测过：与 LIO 对齐后 Δxy 均值 10.6 / p95 38.0 / max 71.6 mm（融合前口径），
  自转段最大约 200 mm 且事后回归，属系统性打滑而非随机噪声——这正是 §10 置信度门控要压的部分。

### 7.3 静止实测（2026-11-12，整车着地 65 s，中途复位一次 MCU）

| 指标 | 实测 |
|---|---|
| `/wheel_odom` 频率 | 50.0 Hz（`usb_bridge` 重采样后；模块原始约 1 kHz） |
| 静止 xy 极差（50 s） | x **0.034 mm**、y **0.029 mm**；标准差各 **0.007 mm** |
| 静止 yaw 漂移 | 50 s 内 0.139° → **0.0028 °/s** |
| MCU 复位影响 | 断流 **1.04 s**（`usb_bridge` 1 s 重连节拍），融合 0.26 s 退化纯 LIO、重连后 0.06 s 回到融合 |
| 融合输出单步（复位窗口） | max **8.6 mm**，无跳变 |
| 融合输出 50 s 静止抖动 | x 标准差 **3.07 mm**、y **1.55 mm**；单步 p95 1.98 mm、max 3.71 mm |
| 同期 `/lidar_odometry` 抖动 | x 标准差 **6.25 mm**、极差 42 mm；单步 p95 10.34 mm、max 15.83 mm |

结论：轮式本身的静止噪声可忽略（0.007 mm），融合输出的 3 mm 抖动**全部来自 LIO 静止抖动**，
融合把它**减半**了。若要进一步压，方向是 point_lio 的静止抖动，不是轮式。

**运动精度后续基准**（尚未做）：
1. 直行 5 m（卷尺测真值），比较里程计 `x` 的尺度误差；
2. 原地正反转各 10 圈，比较 `yaw` 回零误差与线性度；
3. 与 `point_lio` 轨迹对齐，比较 30 s 内的相对漂移。

## 8. 轮式里程计的优缺点

### 优点

- **高频、低延迟**：实测约 1 kHz，远高于 MID360 点云（约 11 Hz）和 LIO 的输出频率，
  适合做速度环/预测。
- **短时位置精度高**：参考工程直接用 3 mm 到点容差做闭环。
- **不依赖环境特征**：长走廊、空旷区、光照变化都不影响。
- **直接给车体速度**：`vx/vy/wz` 可直接用于底盘速度反馈。
- **算力开销低**：只占一个 UART + 一次 CRC。

### 缺点

- **累积漂移**：轮径误差、打滑、地面材质、安装角误差都会积分成位置/航向误差；
  静止 1 s 已见约 0.007° 的航向漂移。
- **无绝对参考**：必须配合清零指令使用；被碰撞或外力推动后原点失效。
- **依赖安装标定**：偏置 14.5 mm 与安装角必须准确，否则平移/旋转会串扰。
- **全向轮/麦轮特有**：横移时的侧向打滑比普通轮更难建模，地面越滑误差越大。
- **协议无时间戳/协方差**：融合时只能自己估计延迟与噪声。
- **航向不能清零**：C3 只清 XY，yaw 需要软件偏移或模块断电重启。

### 与 `point_lio` 的互补关系

| 维度 | 轮式里程计 | point_lio + small_gicp |
|---|---|---|
| 频率 | ≈1 kHz | 约 11 Hz（点云）+ TF |
| 短时精度 | 高（mm 级） | 高 |
| 长时漂移 | 持续累积 | 有先验图时被 GICP 拉回 |
| 退化场景 | 不受场景影响，但打滑/碰撞失效 | 长走廊、空旷、动态障碍下可能退化 |
| 绝对参考 | 无 | 有（先验地图） |

结论：两者应当**融合**而不是二选一——轮式里程计补 LIO 的高频与退化段，LIO 补里程计的
长期漂移。接入前必须先核对符号：本车固件的 `wz` 与 `vx` 有过取反/45° 偏角的历史
（见 [`pitfalls.md`](pitfalls.md) 第 1 条），里程计帧的 `vx/vy` 约定是“前/左”，
**融合前必须实测方向一致性**，不能直接拿参考工程的符号照搬。

## 9. 后续接入建议：现状

| 建议 | 状态 |
|---|---|
| 1. 清零指令 + 上电等待 + 失败重试 | **已完成**（2 s + 首帧锚点，§4.1） |
| 2. `legacy`/USB 上报给 ROS | **已完成**（`0x85` → `/wheel_odom`） |
| 3. 融合轮式与 LIO 输出 `odom→base` | **已完成**（`wheel_odom_fusion`，见 §10；用互补滤波而非 EKF） |
| 4. 标定：直线尺度、偏置、安装角、静态零漂 | **部分完成**（安装角 −135°、尺度 1.00、静态零漂已测，见 §7.2/§7.3） |
| 5. Nav2 中用轮式做高频预测、减少 TF 抖动 | **已完成**（`odom→base` 由融合节点以 50 Hz 发布） |
| 6. 用轮式当独立标尺判断 LIO 退化（健康度） | 未实施（下一阶段） |

## 10. 融合节点 `wheel_odom_fusion`

> 代码：`ros_ws/src/sentry_localization/wheel_odom_fusion/`
> 开启方式：`sentry.launch.py use_wheel_fusion:=true`（默认 `false`，关闭时仍由
> `sensor_scan_generation` 独占 `odom→base_footprint`）。
> 两者互斥由 `navigation_launch.py` 保证，**任何时刻只允许一个发布者**。

### 10.1 数学模型

- 轮式位姿 `W` = `/wheel_odom` 相对锚点的位姿（`anchor_` 在进入融合时取 LIO 时刻的插值）；
- LIO 位姿 `P_L` = `lidar_odometry` 经静态外参转到 `base_footprint`；
- 偏移 `O`（表达在 `odom` 帧，左乘）：残差 `R = P_L ∘ W⁻¹`；
- 输出 `P = O ∘ W`，以轮式频率（50 Hz）发布 `odom→base_footprint` 与 `/odometry`；
- LIO 每来一帧（20 Hz）更新 `O ← blend(O, R, alpha_eff)`。

**`alpha` 的语义是“多快采纳 LIO”**：轮式越不可信，`alpha` 越应**增大**（更信 LIO）。
门控只会把 `alpha` 从基值（0.2）往上抬，永不低于基值。这一点写反了会静默变差。

### 10.2 置信度门控（`alpha_eff`）

判据用 **0.2 s 滑动窗**（50 Hz 下 10 帧），不用单帧比值：

| 判据 | 触发条件 | 效果 |
|---|---|---|
| 自转门控 | `alpha*(1 + 5*max(0, |wz| - deadband))`，`wz` 在 LIO 时间戳处插值 | 连续抬高，封顶 `alpha_max`(1.0) |
| 零速失配 | 窗口内 twist 平均线/角速度都≈0，但位姿路径 ≥ 2 cm / 0.05 rad | 抬到 `alpha_mismatch`(0.5) |
| 一致性失配 | 位姿路径 / twist 积分路径 的比值 > 3（平移与 yaw 各查一次） | 抬到 `alpha_mismatch` |

真静止时两条路径都低于 `min_motion`，不评估比值，**不会误触发**。

### 10.3 跳变判据与快速恢复

- 单步判跳：`|Δxy| > 5·dt + 0.05`、`|Δyaw| > 8·dt + 0.1`、或
  `|Δyaw − wz·dt| > 0.05`（与陀螺比对，实测 p99 仅 0.0059 rad）；
- 孤立尖峰：丢弃该样本，缓冲不动；
- **链路切换 / 原点重置**：连续 3 帧被判跳**且**被拒样本彼此连续 → 立即重锚定，
  恢复时间 0.5 s → **60 ms**；
- 断流兜底：`dt > max_wheel_dt_s`(0.5) 时重锚定；轮式 0.2 s 无数据退化为纯 LIO，
  LIO 0.5 s 无数据停止发布（不让轮式无监督漂移）。

### 10.4 实测效果（标定包离线回放，27.3 s，50 Hz）

| 指标 | 基值 α=0.2 | 门控后 |
|---|---|---|
| 与 LIO 的 Δxy 均值 / p95 / max | 10.6 / 38.0 / 71.6 mm | **7.5 / 15.6 / 33.0 mm** |
| Δyaw p95 / max | 0.49° / 1.44° | 0.53° / 1.07° |
| 单步 xy max | 13.3 mm | 16.8 mm |
| 二阶差 p95（平顺性） | 6.44 mm | 8.17 mm |

注入用例（合成 bag）：原点整体跳 0.594 m 时，开重锚定单步 max 9.6 mm、Δxy max 18 mm；
关重锚定单步 **602 mm**、Δxy max **596 mm**。陀螺冻结 + 轮子打滑时，
一致性门控把与 LIO 的 Δyaw 从 17.19° 压到 **6.88°**。

### 10.5 参数

全部在 `pb2025_nav_bringup/config/reality/nav2_params.yaml` 的 `wheel_odom_fusion` 小节，
新增项均有默认值：`alpha_spin_gain`(5.0)、`alpha_spin_deadband_rps`(0)、`alpha_max`(1.0)、
`alpha_mismatch`(0.5)、`gate_window_s`(0.2)、`gate_still_speed_mps`(0.05)、
`gate_still_yaw_rps`(0.1)、`gate_min_motion_m`(0.02)、`gate_min_motion_rad`(0.05)、
`gate_ratio_max`(3.0)、`jump_reanchor_consecutive`(3)、`jump_linear_margin_m`(0.05)、
`jump_yaw_margin_rad`(0.1)、`yaw_jump_tolerance_rad`(0.05)。

## 11. 相关文件

- 固件实现：`stm32_ws/chassis_fw/App/bsp/bsp_odom_uart.c` / `.h`
- 上报链路：`stm32_ws/chassis_fw/App/comm/legacy_bridge.c`（`0x85`）、
  `ros_ws/src/sentry_hardware/usb_bridge/usb_bridge/usb_bridge_node.py`（`/wheel_odom`）
- 融合节点：`ros_ws/src/sentry_localization/wheel_odom_fusion/`
  （`fusion_core.hpp` 是纯函数与门控，`wheel_odom_fusion_node.cpp` 是 ROS 壳）
- 参数：`stm32_ws/chassis_fw/App/common/robot_param.h`（`ODOM_*`）
- 链接脚本：`stm32_ws/chassis_fw/STM32H723VGTx_FLASH.ld`（`.odom_dma`）
- 参考工程：`reference/工创底盘里程计例程/User/APP/mecanum_chassis.c`
- 相关： [`localization_roadmap.md`](localization_roadmap.md)、[`hardware.md`](hardware.md)、
  [`pitfalls.md`](pitfalls.md)
