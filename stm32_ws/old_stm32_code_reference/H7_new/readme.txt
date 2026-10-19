DM-balanceV1 (H7_new) - User layer operations and data flow
===========================================================

Overall task topology
---------------------
- RTOS entry: Core/Src/freertos.c creates tasks from User/APP (INS, gimbal, chassis, shoot, referee/comm, UI, detect, calibrate, idle/test).
- Shared types/helpers: APP/struct_typedef.h, Lib/user_lib.[ch], Algorithm/* (PID, filters, VMC).
- Inputs: BMI088 IMU (attitude/gyro), RC DBUS, referee serial, upper/auto-aim link (Communication/), board sensors (encoders, currents).
- Outputs: CAN motor currents (chassis/gimbal/shoot), PWM (servo), USART/USB for comm/debug, referee UI drawings.

Key data producers
------------------
- INS_task (APP/INS_task.[ch]): initializes BMI088 (Devices/BMI088), runs attitude filter (Algorithm/mahony, kalman, EKF), publishes attitude/gyro/acc to globals used by gimbal/chassis.
- remote_control (APP/remote_control.[ch], Bsp/bsp_rc): parses DBUS; provides mode switch states, target velocities/angles, shoot triggers.
- detect_task (APP/detect_task.[ch]): online check for motors/sensors; can force safe modes on fault.
- referee (APP/referee.[ch], referee_usart_task.[ch]): parses referee frames, provides power/energy/bullet/quota/game states to other modules and UI.

Chassis control (mecanum / omni / steering / balance)
-----------------------------------------------------
- Task: APP/chassis_task.[ch]
  - Inputs: RC commands (vx, vy, wz), referee power limit, gyro yaw (from INS), gimbal yaw (for follow), behaviour decisions.
  - Flow: chassis_behaviour.[ch] picks mode (follow-gimbal, gyro-spin, remote-direct, auto-aim assist) -> motion target -> chassis_power_control.[ch] limits power/current -> kinematics -> wheel targets -> PID -> CAN via can_bsp.
- Kinematics implementations:
  - Mecanum: chassis/Mecanum_chassis.[ch]; mapping (vx, vy, wz) -> four wheel speeds; direction/ID configured in APP/robot_param*.h.
  - Omni: chassis/Omni_chassis.[ch]; similar mapping for omni layout.
  - Steering: chassis/steering_chassis.[ch]; solves steering angle + drive speed per module; outputs go to paired steering/drive motors over CAN.
  - Balance/legged idea: chassis/Balance_chassis.[ch] and ROBOT/Balance.[ch]; uses attitude feedback for self-balance torque distribution (optional if not used).
- Outputs: CAN currents to chassis motors, plus status to message_task/UI.

Gimbal (direct two-axis)
------------------------
- Files: gimbal/yaw_pitch_direct.[ch]; Task: APP/gimbal_task.[ch]; Behaviour: APP/gimbal_behaviour.[ch].
- Inputs: INS attitude, RC or auto-aim targets (yaw/pitch), shoot linkage flags, detect status.
- Flow: behaviour decides mode (gyro/encoder/auto-aim/calibration/stop) -> set yaw/pitch setpoints -> PID (Algorithm/PID) -> CAN currents to yaw/pitch motors (IDs from robot_param*.h). Mode switches latch current angles to avoid jumps.
- Alternative mechanisms: gimbal/double_yaw_pitch.[ch], yaw_pitch_linkage.[ch] if using dual-axis or linkage; swap in via behaviour/config.

Shooting
--------
- Task/state: APP/shoot.[ch]; shared types in APP/shoot.h.
- 3508 shooter: shoot/shoot_3508.[ch] controls friction wheels and a single feeder/bullet motor; handles jam clear, speed hold.
- Dual-layer shooter: shoot/shoot_double.[ch] coordinates dual feeders/dual motors; syncs friction and feeding.
- Inputs: RC shoot command, auto-aim lock/allow, referee power/heat limits, motor feedback (currents/encoder).
- Flow: state machine (idle/spin-up/feed/jam-clear) -> friction PID to target RPM, feeder position/speed control -> CAN currents -> status exported to UI/comm.

Auto-aim
--------
- Module: APP/auto_aim.[ch]; consumed by gimbal_task and shoot.
- Inputs: target info from comm stack (Communication/), RC switch to enable, current attitude/position estimate.
- Flow: parse target -> compute desired yaw/pitch (and optional fire-permit) -> publish to gimbal_behaviour; shoot reads lock/permit to decide feeding.
- Outputs: gimbal targets, fire enable flag; can also report status back over comm.

Communication (board-to-board / auto-aim link)
----------------------------------------------
- Stack: Communication/core/{comm.c, uproto.c, CRC8_CRC16.c, config.h, platform.h}; utilities in comm_utils.h.
- Channels: Communication/channel/{camera, gimbal, time_sync} as templates; example host/device in Communication/example/*.
- Integration: APP/message_task.[ch] builds `comm_app` payloads (robot status, pose, chassis, gimbal, shooter) -> core/comm packs via uproto/CRC -> sent over selected physical (USART/CAN/USB). Incoming frames parsed -> update shared structs used by auto_aim/gimbal/chassis.
- For auto-aim: reuse same stack; define a channel to carry targets and feedback (latency, fire result).

UI (referee overlay)
--------------------
- Files: APP/UI.[ch]; drawing libs: ui/, ui_back/ (ui_interface, ui_types, ui_g).
- Inputs: referee parsed data (power/HP/heat), robot runtime (mode, targets, bullets, errors), optional auto-aim status.
- Flow: assemble graphic commands -> send via referee USART according to protocol limits; throttling needed to fit bandwidth.
- Outputs: client-side overlay showing HP/energy/mode/aim/shoot states; color/shape defined in ui_g/ui_interface.

Other supporting modules
------------------------
- Algorithm/: PID (PID/pid.[ch]); filters (mahony, kalman, EKF/QuaternionEKF); VMC/VMC_calc for chassis force/velocity allocation.
- Bsp/: CAN (can_bsp), RC (bsp_rc), USART/USB (bsp_usart, bsp_usb), PWM, flash/w25q64, DWT timing.
- Devices/: BMI088 driver/middleware; DM_Motor; oled (OLED.c/h, oledfont.h).
- profiler/: task_profiler_sampling.[ch] for timing; detect_task for health; calibrate_task for sensor/motor calibration; led_flow_task for status LEDs; usb_task for CDC debug.

Robot type and parameters
-------------------------
- ROBOT/{Balance, Engineer, Hero, Infantry_robot, Sentinel} select model-specific behaviour.
- Parameters in APP/robot_param.h and robot_param_backup.h: CAN IDs, motor directions, PID gains, mechanical limits. Macro ROBOT_TYPE chooses active set at compile time.

Typical data paths (to trace code)
----------------------------------
- Chassis (mecanum/omni): DBUS -> remote_control -> chassis_behaviour (mode) -> chassis_task (vx,vy,wz) -> Mecanum_chassis/Omni_chassis -> PID -> can_bsp Tx currents.
- Chassis (steering): same front-end, then steering_chassis computes steer angle + drive speed -> two CAN channels (steer/drive).
- Gimbal (direct): DBUS/auto-aim -> gimbal_behaviour -> yaw_pitch_direct PID -> CAN yaw/pitch.
- Shooting (3508/double): DBUS/auto-aim permit -> shoot state machine -> friction RPM PID + feeder position/speed -> CAN currents -> status to UI/comm.
- Auto-aim link: sensors/pose -> comm_app pack -> Communication/core send; incoming target -> auto_aim -> gimbal/shoot.
- UI: referee_usart_task parse -> UI build graph list -> referee serial send to client.

If you extend or debug
----------------------
- Add a new chassis/gimbal/shooter: implement under corresponding folder, wire into behaviour/task, register CAN IDs, tune PID in robot_param.
- For comm: define message schema in Communication/channel or example/shared, hook send/recv in message_task and auto_aim.
- For UI: add shapes in ui_g/ui_interface; respect referee bandwidth; gate updates on state changes.
