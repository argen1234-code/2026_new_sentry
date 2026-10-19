# DM-balanceV1 云台任务数据流与操作指南 (H7_new)

面向新接手同学，梳理云台（直连/联动/双轴）相关模块的任务位置、输入输出、模式切换与调试要点。

## 主要文件与职责
- 任务：`User/APP/gimbal_task.[ch]`
  - FreeRTOS 任务主体，周期性读取传感/指令，调用行为、解算、PID，下发 CAN。
- 行为：`User/APP/gimbal_behaviour.[ch]`
  - 根据模式选择控制方式（陀螺角度/编码器/自瞄/校准/停止），处理模式切换时的设定值锁存。
- 机构解算：
  - 直连：`User/gimbal/yaw_pitch_direct.[ch]`
  - 双轴：`User/gimbal/double_yaw_pitch.[ch]`
  - 联动：`User/gimbal/yaw_pitch_linkage.[ch]`
- 参数：`User/APP/robot_param*.h`（电机 CAN ID、方向、行程/限幅、PID 参数），`User/ROBOT/*.c` 选择机型。
- 依赖：`User/APP/INS_task`（姿态）、`remote_control`（模式/目标）、`auto_aim`（目标点）、`shoot`（联动开火），`Algorithm/PID`。

## 数据流
1) 输入采集
   - 姿态：`INS_task` 提供当前 yaw/pitch（或陀螺角/编码器角）。
   - 指令：`remote_control` 提供手动微调/模式开关；`auto_aim` 提供目标角与开火许可。
   - 状态：`detect_task` 的在线状态，`shoot` 联动信号（开火允许/热量）。

2) 模式决策（`gimbal_behaviour`）
   - 模式：陀螺模式、编码器模式、自瞄模式、校准、停止。
   - 切换时将当前角度锁存为新设定，避免跳变。

3) 目标生成
   - 自瞄：使用 `auto_aim` 目标 yaw/pitch；否则用遥控增量叠加当前角。
   - 角度限幅：根据机械行程/防撞范围限制设定。

4) 控制
   - 误差计算：设定角/速 - 当前角/速。
   - PID：`Algorithm/PID/pid.[ch]`，双环（角度→速度→电流）。
   - 可选前馈：根据陀螺角速补偿。

5) 下发
   - 形成 CAN 电流，调用 `can_bsp` 发送至 yaw/pitch 电机。
   - 输出状态（模式/目标/误差）可写入 UI/通信。

## 典型链路
- 手动云台：DBUS → `remote_control` → `gimbal_behaviour`（陀螺/编码器模式）→ `yaw_pitch_direct` → PID → CAN。
- 自瞄云台：自瞄目标 → `auto_aim` → `gimbal_behaviour`（自瞄模式）→ PID → CAN → 将锁定状态通知 `shoot`。
- 双轴/联动：替换解算文件（double_yaw_pitch / yaw_pitch_linkage），保持任务入口不变。

## 关键配置
- 电机 ID/方向/减速比/行程：`robot_param*.h`
- PID 参数：`robot_param*.h` 或 gimbal_init 内设置
- 模式宏/机器人类型：`ROBOT_TYPE`（影响参数选择）

## 调试建议
- 先单轴验证方向与编码值，确认正转正反馈，否则调整方向宏。
- 开启小步长手动控制，观察目标/实际角度是否一致；必要时降低 PID。
- 自瞄模式下确认目标坐标系一致，防止 yaw/pitch 反向。
- 校准模式慎用，确保限位与机械零点匹配。
