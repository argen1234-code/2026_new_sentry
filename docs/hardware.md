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
| 右后 RR | FDCAN1 | 1 | 0x00 | 车身后退 | -1 | ✅ 已实测 |
| 左后 RL | FDCAN1 | 3 | 0x53 | 待确认 | 待确认 | ✅ 已识别，短时 1 rad/s 驱动 |
| 前左 FL | 待定 | 待定 | 待定 | 待测 | 待定 | 待接入 |
| 前右 FR | 待定 | 待定 | 待定 | 待测 | 待定 | 待接入 |

说明：

- “正指令方向”指原生速度模式下发正数时车轮的转向对应的车身运动方向。
- “车身前进方向符号”是底盘运动学输出到该轮时需要乘的符号。
- 右后轮实测：`+3.0` 使车身后退，所以前进时该轮取 `-1`。
- 左后轮实测：CAN1 / 电机 ID 3 / 反馈 ID `0x53`，已用 `+1.0` 短时运行 3 秒；方向待确认。
- 两台电机的反馈主 ID 不一致：右后 `0x00`，左后 `0x53`。当前 `Devices/Src/motor.c`
  只把 `0x51..0x58` 当作 MIT 反馈 ID；右后 `0x00` 不会被设备层解码。
  接整车控制前，要么用达妙工具把右后主 ID 改成 `0x51`，要么扩展设备层解码。

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

> 新增电机前务必确认：CAN 总线、电机 ID、反馈主 ID、正指令对应的车身方向。
> 这些事实不要只留在临时测试固件里，必须同步更新本表。

### 左后轮测试记录（2026-10-19）

- CAN1 / 电机 ID 3 / 反馈 ID `0x53`。
- 速度模式 `+1.0`、`+3.0`、`-3.0` 下，反馈位置都停在约 `-12 rad` 附近，
  电机发出刺耳声音，未形成持续转动。
- 判断：电机顶在位置/机械限位，速度环持续出力导致异响。
- 已发送零速 + MIT 零力矩 + 失能，反馈状态位变为 `0`（disabled），`TEC/REC=0`。
- 后续禁止再直接向限位方向发速度指令。必须先：
  1. 手动确认轮子能否自由转动；
  2. 用达妙工具确认电机位置范围、控制模式和零点；
  3. 只做小角度、低增益、限时的 MIT 位置测试，出现异响立即失能。

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
