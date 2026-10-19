# DM-balanceV1 杂项模块速览 (H7_new)

汇总非底盘/云台/射击/通信/自瞄/UI 之外的常用模块，便于定位入口。

## 公共算法与工具
- `User/Algorithm/`
  - `PID/pid.[ch]`: 通用 PID 结构与接口，云台/底盘/射击均在用。
  - `mahony`, `kalman`, `EKF/QuaternionEKF`: 姿态/传感器滤波，供 INS_task 选用。
  - `VMC/VMC_calc.[ch]`: 力矩/速度分配，可用于底盘动力学/平衡。
- `User/Lib/user_lib.[ch]`: 常用数学/限幅/滤波小工具。

## 设备与硬件抽象
- `User/Bsp/`
  - `can_bsp`: CAN 发送/接收封装；电机控制的唯一出口。
  - `bsp_rc`: 遥控器(DBUS)解析。
  - `bsp_usart`, `bsp_usb`: 串口/USB CDC 驱动封装。
  - `bsp_pwm`, `bsp_flash`, `w25q64`, `bsp_dwt`: PWM 驱动、片上/外部 flash、DWT 时基。
- `User/Devices/`
  - `BMI088`: IMU 驱动+中间层（寄存器、SPI、温补）。
  - `DM_Motor`: 电机驱动封装。
  - `oled`: OLED 显示与字库。

## 系统与任务辅助
- `User/APP/detect_task.[ch]`: 在线检测各设备/任务，故障时切保护模式。
- `User/APP/calibrate_task.[ch]`: 传感器/电机标定（零偏、方向）。
- `User/APP/idle.[ch]`, `test_task.[ch]`: 空闲与测试任务，可用于调试。
- `User/profiler/task_profiler_sampling.[ch]`: 任务性能/周期采样。

## 机器人配置
- `User/ROBOT/*.c`: 不同机型（Balance, Engineer, Hero, Infantry_robot, Sentinel）的专用逻辑或入口。
- `User/APP/robot_param*.h`: CAN ID、方向、PID、限幅、行程等参数；`ROBOT_TYPE` 宏决定生效配置。

## 其他通信/应用挂点
- `User/APP/message_task.[ch]`: 汇总状态（底盘/云台/射击/功率等），与 Communication 栈对接发送；解析回包时也在此更新共享状态。
- `User/APP/observer.[ch]`: 观测器/状态估计（如果启用）。
- `User/APP/led_flow_task.[ch]`: LED 流水灯/指示。
- `User/APP/usb_task.[ch]`: USB CDC 通道（调试/通信）。
