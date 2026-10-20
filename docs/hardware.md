# 硬件接线记录

## 底盘与电机

| 项 | 值 |
|---|---|
| 底盘 | 麦克纳姆轮，4 轮 |
| 控制板 | 达妙 DM-MC02 / STM32H723 |
| 电机 | 达妙 DMH6215 |
| CAN | 经典 CAN，1 Mbps |
| 已验证模式 | 原生速度模式（命令 ID = 电机 ID + 0x200）、MIT 模式 |

## 电机映射

| 位置 | CAN 总线 | 电机 ID | 反馈 ID | 正指令方向 | 车身前进方向符号 | 状态 |
|---|---|---:|---:|---|---:|---|
| 右后 RR | FDCAN1 | 1 | 0x51 | 车身后退 | -1 | ✅ 20 s 低速前进测试通过 |
| 左后 RL | FDCAN1 | 3 | 0x53 | 车身前进 | +1 | ✅ 20 s 低速前进测试通过 |
| 前左 FL | FDCAN1 | 2 | 0x52 | 车身前进 | +1 | ✅ 20 s 低速前进测试通过 |
| 前右 FR | FDCAN1 | 4 | 0x54 | 车身后退 | -1 | ✅ 20 s 低速前进测试通过 |

说明：

- “正指令方向”指原生速度模式下发正数时车轮的转向对应的车身运动方向。
- “车身前进方向符号”是底盘运动学输出到该轮时需要乘的符号。
- 右后轮实测：`+3.0` 使车身后退，所以前进时该轮取 `-1`。
- 左后轮实测：CAN1 / 电机 ID 3 / 反馈 ID `0x53`；低速保守测试见下方记录。
- 2026-10-20 四轮悬空 20 s、`0.3 rad/s` 前进测试确认：左侧 FL/RL 正转对应
  车身前进，右侧 RR/FR 反转对应车身前进。前进符号为 RR `-1`、FL `+1`、
  RL `+1`、FR `-1`。
- 四台电机统一使用发送 ID `1..4`、反馈 ID `0x51..0x54`，与
  `Devices/Src/motor.c` 的 MIT 反馈范围 `0x51..0x58` 对齐。
- ID 通过 DM 参数帧修改：CAN ID `0x7FF`，`RID=8` 写 ESC_ID、`RID=7` 写 MST_ID，
  修改后必须发送保存命令，否则掉电恢复旧 ID。

### CAN ID 配置记录（2026-10-19）

四台电机的发送/反馈 ID 已按上表重配，避免发送 ID 和反馈 ID 冲突。
修改使用达妙 `0x7FF` 参数帧（经典 CAN，8 字节）：

| 操作 | CAN ID | D[0] | D[1] | D[2] | D[3] | D[4..7] |
|---|---:|---:|---:|---:|---:|---|
| 写 MST_ID（反馈 ID） | `0x7FF` | 旧 ESC_ID 低字节 | 旧 ESC_ID 高字节 | `0x55` | `0x07` | 新反馈 ID，uint32 小端 |
| 写 ESC_ID（发送 ID） | `0x7FF` | 旧 ESC_ID 低字节 | 旧 ESC_ID 高字节 | `0x55` | `0x08` | 新发送 ID，uint32 小端 |
| 保存参数 | `0x7FF` | 新 ESC_ID 低字节 | 新 ESC_ID 高字节 | `0xAA` | `0x01` | 0 |

注意：写 ESC_ID 后，后续帧必须使用新的 ESC_ID；保存命令按新 ESC_ID 发送。
保存前必须先发失能命令并等待约 10 ms；保存后等待 100 ms 以上再断电。
未先失能或保存后立即掉电时，ID 可能只在 RAM 生效，重启后回退到旧值。
改完必须断电重启，确认 ID 已真正写入 Flash。

四台同时上电验证（2026-10-20）：反馈 ID 只出现 `0x51/0x52/0x53/0x54`，
发送 ID 为 1/2/3/4，`motor_mask=0x1E`，`TEC/REC=0`，总线上不再出现默认的 `0x00` 反馈 ID。
上电后反馈状态位为 `0` 只表示电机尚未使能，不是故障；使能后应变为 `1`。

### 控制模式与使能偏移

- 控制模式是参数 `CTRL_MODE`（RID=10）：1=MIT，2=位置速度，3=速度，4=位置力控。
- 读：`0x7FF` + `[ID,0,0x33,0x0A,0,0,0,0]`，应答 `data[4..7]` 为模式值。
- 写：`0x7FF` + `[ID,0,0x55,0x0A,mode,0,0,0]`；保存：`0x7FF` + `[ID,0,0xAA,0x01,...]`。
- `0xFC` 是使能、`0xFD` 是失能、`0xFE` 是回零、`0xFB` 是清错，不是模式切换命令。
- 使能/失能必须发到 `ID + 模式偏移`：MIT `+0x000`、位置速度 `+0x100`、
  速度 `+0x200`、位置力控 `+0x300`。
- 反馈中的 12 位速度/力矩中点 `0x7FF` 对应 0，不是最大值。

### 四轮前进测试记录（2026-10-20）

- 模式：四台统一 `CTRL_MODE=3`（速度模式）并保存。
- 参数：`0.3 rad/s`、20 s、300 ms 斜坡、悬空测试。
- 方向：RR `-1`、FL `+1`、RL `+1`、FR `-1` 时四轮均朝车身前进方向旋转。
- 结束：零速 + MIT 零力矩 + 失能。
- 安全注意：速度模式在通信中断后会保持最后速度。调试器 halt、拔 CAN 或
  调试服务异常都可能让电机继续转；测试后必须烧回 stop-only 固件，
  对每个 ID 的 `+0x000/+0x100/+0x200/+0x300` 全部发送失能。

### 底盘 DM 速度环验证（2026-10-20）

target：`sentry_2026_chassis`，底盘控制周期 2 ms，台架注入 20 s、`vx=0.0135 m/s`（轮端 `0.3 rad/s`）。

修复前后数据：

| 项 | 修复前 | 修复后 |
|---|---|---|
| `dm_mode_value` | 未设置 | `{3,3,3,3}` |
| 左后 ID3 `state` | `0`（失能） | `1`（使能） |
| 左后 20 s 净转角 | `-0.014` rad | **`+6.01` rad** |
| 其余三轮净转角 | `±6.0` rad | `±6.0` rad |
| 反馈在线 mask | `15` | `15` |
| 使能发送失败 | `0` | `0` |

结论：

- 左后“在线但不动”是 `CTRL_MODE` 丢失，不是反馈、接线或使能丢帧。
- 底盘启动必须先写/读回/保存 `CTRL_MODE=3`，再使能。
- 运行中每 500 ms 重发一次使能，单帧丢失最多 0.5 s 恢复。
- 四轮全在线门限为 4；反馈超时和故障使能延时在台架验证后需按实车收紧。

## 测试方法

单轮悬空测试使用 `can1_dmh6215_test` 选项：

```bash
cd stm32_ws/Boards
cmake -S . -B build/can_test -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug -DROBOT_TARGET=can1_dmh6215_test
cmake --build build/can_test
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
  -c "program build/can_test/CtrlBoard-H7_IMU.elf verify reset exit"
```

测试固件会扫描 ID 1–8，锁定第一个反馈电机，并以原生速度模式持续运行。
停止测试时烧回 `sentry_2026_bringup`（无电机输出）。

指定 ID 的首次试转使用保守模式：MIT 速度控制，`Kp=0`、`t_ff=0`，
力矩上限约为 `Kd * v_des`；速度先斜坡上升，再限时保持，最后自动零速并失能。
同时有反馈中断、位置变化和堵转兜底：

```bash
cd stm32_ws/Boards
cmake -S . -B build/can_safe -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DROBOT_TARGET=can1_dmh6215_test \
  -DAPP_CAN_TEST_CONSERVATIVE=1 \
  -DAPP_CAN_TEST_TARGET_ID=3 \
  -DAPP_CAN_TEST_SAFE_VEL_RAD_S=0.5F
cmake --build build/can_safe
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
  -c "program build/can_safe/CtrlBoard-H7_IMU.elf verify reset exit"
```

默认参数：目标 `0.5 rad/s`、`Kd=0.5`（最大力矩约 `0.25 Nm`）、
斜坡 `500 ms`、保持 `1000 ms`、位置变化上限 `1.0 rad`、总超时 `3 s`。
测试结束后固件停在 `DONE` 状态；**复位/重新上电会重新执行测试**，
离开台架前请烧回 stop-only 或 `sentry_2026_bringup`。

> 新增电机前务必确认：CAN 总线、电机 ID、反馈主 ID、正指令对应的车身方向。
> 这些事实不要只留在临时测试固件里，必须同步更新本表。

### 左后轮测试记录（2026-10-19）

- CAN1 / 电机 ID 3 / 反馈 ID `0x53`。
- 速度模式 `+1.0`、`+3.0`、`-3.0` 下，反馈位置都停在约 `-12 rad` 附近，
  电机发出刺耳声音，未形成持续转动。
- 当时判断为“顶在位置/机械限位”，**该判断后来被证明不准确**。
- 已发送零速 + MIT 零力矩 + 失能，反馈状态位变为 `0`（disabled），`TEC/REC=0`。

### 左后轮“在线但不动”的真正根因（2026-10-20 确认）

三轮以上排查后确认：左后电机的 `CTRL_MODE` 没有保持在 3（原生速度模式）。

- 模式不是 3 时，电机仍正常发送 `0x53` 反馈，所以 `online`、`fb_count` 都正常；
- 但它会忽略 `ID + 0x200` 的使能和速度帧，反馈状态位 `state` 一直是 `0`（失能）；
- 表现为“反馈在线、使能发送成功、但轮子完全不动”，容易被误判成接线或机械故障。

修复方式：每次启动先写 `CTRL_MODE=3` 并读回确认，再保存、再使能：

```text
0x7FF  [ID,0,0x55,0x0A,3,0,0,0]   写 CTRL_MODE=3
0x7FF  [ID,0,0x33,0x0A,0,0,0,0]   读回，应答 data[4..7] 应为 3
0x7FF  [ID,0,0xAA,0x01,0,0,0,0]   保存
0x200+ID  0xFC                     速度模式使能
```

判据：反馈 `state = data[0] >> 4`，`0` 表示失能，`1` 表示使能；不能只看反馈是否在线。

### 左后轮保守低速测试记录（2026-10-19）

- 台架状态：CAN1 上只挂左后电机，电机 ID 3，反馈 ID `0x53`。
- 固件：`can1_dmh6215_test` + `APP_CAN_TEST_CONSERVATIVE=1`，参数 `0.5 rad/s`、
  `Kd=0.5`、斜坡 `500 ms`、保持 `1000 ms`、总超时 `3 s`。
- 结果：状态 `DONE`，原因 `RUN_DONE`；收到 244 帧反馈，`TEC/REC=0`。
- 位置：起始约 `0.0006 rad`，结束约 `0.3336 rad`，累计变化约 `0.305 rad`。
- 结论：正速度指令使解算位置增大，电机能低速连续转动；测试结束已执行
  零速 + MIT 零力矩 + 失能。
- 注意：本次测试时 `V_MAX` 仍沿用 ±200 rad/s，MIT 速度指令和反馈都偏小约 4.4 倍；
  已按说明书 7.1.6 修正为 `P_MAX=±12.5 rad`、`V_MAX=±45 rad/s`、`T_MAX=±10 Nm`，
  下次测试的速度读数和指令比例才是真实值。

## 安全停止

达妙原生速度模式在通信中断后会**保持最后一次速度设定**。切换固件、烧录 bring-up
或断电前，必须显式停止：

1. 向 `电机 ID + 0x200` 发送 float 零速；
2. 向电机 ID 发送 MIT 零力矩帧；
3. 向电机 ID 发送失能命令（`DmMotorDisable`）。

停止专用固件构建：

```bash
cd stm32_ws/Boards
cmake -S . -B build/can_stop -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DROBOT_TARGET=can1_dmh6215_test \
  -DAPP_CAN_TEST_STOP_ONLY=1
cmake --build build/can_stop
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
  -c "program build/can_stop/CtrlBoard-H7_IMU.elf verify reset exit"
```

`sentry_2026_bringup` 只保证 MCU 不再发控制帧，**不能**让已经运行的达妙速度模式电机停转。
