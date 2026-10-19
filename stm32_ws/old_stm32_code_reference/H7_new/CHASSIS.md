# DM-balanceV1 底盘数据流与操作指南 (H7_new)

面向新接手同学，梳理底盘相关模块（麦轮/全向/舵轮/平衡思路）的任务位置、输入输出与调试要点。

## 主要文件与职责
- 任务入口：`User/APP/chassis_task.[ch]`
  - 创建/运行底盘控制任务，调用行为决策、功率限制、运动学解算与 PID。
- 行为决策：`User/APP/chassis_behaviour.[ch]`
  - 根据遥控/自瞄/云台状态选择模式（跟随云台、小陀螺、纯遥控、自动等），输出目标速度/角速度。
- 功率限制：`User/APP/chassis_power_control.[ch]`
  - 依据裁判功率与电容电压，对底盘输出进行限幅。
- 运动学实现：
  - 麦轮：`User/chassis/Mecanum_chassis.[ch]`
  - 全向：`User/chassis/Omni_chassis.[ch]`
  - 舵轮：`User/chassis/steering_chassis.[ch]`
  - 平衡/轮腿思路：`User/chassis/Balance_chassis.[ch]`（可选）
- 配置/参数：`User/APP/robot_param*.h`（电机 CAN ID、方向、PID、限幅等），`User/ROBOT/*.c` 选择不同整机。
- 公共算法：`User/Algorithm/PID/pid.[ch]`、`User/Algorithm/VMC/VMC_calc.[ch]`。
- 硬件抽象：`User/Bsp/can_bsp.[ch]`、`bsp_rc.[ch]`（遥控）、`bsp_dwt.[ch]`（时基）。

## 数据流总览
1) 输入采集  
   - 遥控：`APP/remote_control`（经 `bsp_rc`）提供 vx/vy/wz 或模式开关。  
   - 姿态：`APP/INS_task` 输出 yaw/角速度供“跟随云台”“小陀螺”模式。  
   - 裁判：`APP/referee` 提供功率/电容电压，供功率限幅。  
   - 云台状态：`APP/gimbal_task` 的 yaw，可用于“跟随云台”。

2) 行为决策（`chassis_behaviour`）  
   - 依据模式选择目标：  
     - 跟随云台：目标 yaw 来自云台，底盘 vx/vy 从遥控。  
     - 小陀螺：设定固定角速度 wz，叠加遥控 vx/vy。  
     - 纯遥控/自瞄辅助：直接用遥控或上位机给定。  
   - 输出：期望 vx, vy, wz（机体坐标系）。

3) 功率/电流限制（`chassis_power_control`）  
   - 输入期望速度与当前功率/电容信息。  
   - 计算允许的电流/功率上限，对后续 PID 输出进行缩放或限幅。

4) 运动学与动力学  
   - 麦轮/全向：将 (vx, vy, wz) 映射为四轮目标线速度/转速。  
   - 舵轮：求解每个舵向角与轮速；通常需先解耦角度再限速。  
   - 平衡/轮腿：结合 VMC 或姿态反馈做力矩分配（如果启用）。  
   - 输出：每个轮子的目标速度（或角度+速度）。

5) 控制与下发  
   - 对每个轮子做速度环 PID（`Algorithm/PID`），可选前馈。  
   - 形成 CAN 电流指令，通过 `can_bsp` 发送到对应电机 ID。  
   - 定期将状态（电流、速度、模式、功率）写入 `message_task`/UI。

## 典型链路（按底盘类型）
- 麦轮/全向：  
  DBUS → `remote_control` → `chassis_behaviour` (vx,vy,wz) → `chassis_power_control` → `Mecanum_chassis` / `Omni_chassis` → PID → CAN。

- 舵轮：  
  DBUS/自瞄目标 → `chassis_behaviour` → `chassis_power_control` → `steering_chassis` (舵角+轮速) → 分别 PID 控制转向与驱动 → CAN。

- 平衡/轮腿思路：  
  DBUS + IMU → `chassis_behaviour` → `Balance_chassis`/`VMC_calc` → 力/力矩分配 → PID → CAN。需要 INS 精确姿态，调参工作量大。

## 关键宏与配置
- 机器人类型：`ROBOT_TYPE`（见 `User/APP/struct_typedef.h` / 编译宏），决定使用的参数/模式。
- 电机 ID、方向、减速比、轮径：`User/APP/robot_param*.h`。  
- PID 参数：同样在 `robot_param*.h` 或各模块初始化中设置。  
- 功率保护开关/阈值：`chassis_power_control` 内部常量/宏。

## 调试建议
- 确认 CAN ID 与电机方向：在不带轮胎空载旋转时检查正反，必要时调整方向宏或 PID 符号。
- 逐步放开：先单轮/单轴验证编码器读数和 PID，再上运动学解算。
- 功率限制：接入裁判前可临时放宽限幅，但务必上线前恢复。
- 模式切换：关注 `chassis_behaviour` 中模式锁存与过渡，避免切换时速度突变。
- 观测：`profiler/task_profiler_sampling` 可量测周期；必要时在 UI/串口输出 vx/vy/wz、轮速、电流。

## 扩展/修改指南
- 新底盘：在 `User/chassis/` 添加实现（参照现有接口），在 `chassis_task` 增加选择分支，并在 `robot_param` 配置新 ID/方向/PID。
- 上位机/自瞄控制底盘：在 `message_task` 增加字段解析，`chassis_behaviour` 增加新模式处理。
- 添加电容/功率策略：修改 `chassis_power_control`，可增加电容优先或功率再分配算法。
