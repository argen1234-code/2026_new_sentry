/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "struct_typedef.h"
#include "can_bsp.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
#include "INS_task.h"
#include "robot_param.h"
#include "VMC_calc.h"

// 底盘控制模式枚举
typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   // 底盘跟随云台 yaw 相对角度
  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  // 底盘按自身 yaw 角度闭环控制
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       // 底盘按旋转速度控制
  CHASSIS_VECTOR_RAW,                 // 电流直接发送到 CAN 总线
  // 舵轮专用模式（兼容 F4 工程定义）
  CHASSIS_VECTOR_NO_MOVE,             // 底盘不动（无力模式）
  CHASSIS_VECTOR_SPIN,                // 原地小陀螺
  CHASSIS_RT_FOLLOW_GIMBAL_YAW_SPIN,   // 回正到预设角度并旋转
	CHASSIS_VECTOR_DEBUG												//底盘调试模式
} chassis_mode_e;

// 通用电机数据结构
typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  fp32 angle;      // 舵轮 6020 当前角度（rad）
  fp32 angle_set;  // 舵轮 6020 目标角度（rad）
  int16_t give_current;
} chassis_motor_t;

// 全向 / 麦轮底盘的数据结构
#if ROBOT_CHASSIS == Omni_wheel || ROBOT_CHASSIS == Mecanum_wheel
typedef struct
{
  const RC_ctrl_t *chassis_RC;               // 底盘使用的遥控器指针
  const gimbal_motor_t *chassis_yaw_motor;   // 用 yaw 云台电机相对角度计算底盘欧拉角
  const gimbal_motor_t *chassis_pitch_motor; // 用 pitch 云台电机相对角度计算底盘欧拉角
  const fp32 *chassis_INS_angle;             // 获取陀螺仪解算的欧拉角指针
  chassis_mode_e chassis_mode;               // 底盘状态机
  chassis_mode_e last_chassis_mode;          // 上一次的底盘状态机
  chassis_motor_t motor_chassis[4];          // 底盘电机（3508）数据
  pid_type_def motor_speed_pid[4];           // 底盘电机速度 PID
  pid_type_def chassis_angle_pid;            // 底盘跟随角度 PID

  first_order_filter_type_t chassis_cmd_slow_set_vx;  // 一阶低通滤波，缓慢修改 vx 设定值
  first_order_filter_type_t chassis_cmd_slow_set_vy;  // 一阶低通滤波，缓慢修改 vy 设定值
//  first_order_filter_type_t chassis_cmd_slow_set_vz;

  fp32 vx;                          // 底盘实际前进速度，前为正，单位 m/s
  fp32 vy;                          // 底盘实际横移速度，左为正，单位 m/s
  fp32 wz;                          // 底盘实际角速度，逆时针为正，单位 rad/s
  fp32 vx_set;                      // 底盘设定前进速度，前为正，单位 m/s
  fp32 vy_set;                      // 底盘设定横移速度，左为正，单位 m/s
  fp32 wz_set;                      // 底盘设定角速度，逆时针为正，单位 rad/s
  fp32 chassis_relative_angle;      // 底盘相对于云台的相对角度，单位 rad
  fp32 chassis_relative_angle_set;  // 底盘相对云台的控制设定角度，单位 rad
  fp32 chassis_yaw_set;             // 底盘期望 yaw 角

  fp32 vx_max_speed;  // 前进方向最大速度，单位 m/s
  fp32 vx_min_speed;  // 后退方向最大速度，单位 m/s
  fp32 vy_max_speed;  // 左方向最大速度，单位 m/s
  fp32 vy_min_speed;  // 右方向最大速度，单位 m/s
  fp32 chassis_yaw;   // 陀螺仪+云台叠加得到的 yaw 角
  fp32 chassis_pitch; // 陀螺仪+云台叠加得到的 pitch 角
  fp32 chassis_roll;  // 陀螺仪+云台叠加得到的 roll 角

} chassis_move_t;
#endif 

// 五连杆平衡底盘

#if ROBOT_CHASSIS == Balance_wheel

typedef struct
{
  const RC_ctrl_t *chassis_RC;
  const gimbal_motor_t *chassis_yaw_motor;
  const gimbal_motor_t *chassis_pitch_motor;
  const fp32 *chassis_INS_angle;
	//角速度指针
  const fp32* acc_point;
  chassis_mode_e chassis_mode;
  chassis_mode_e last_chassis_mode;

  first_order_filter_type_t chassis_cmd_slow_set_vx;
  first_order_filter_type_t chassis_cmd_slow_set_vy;

  fp32 vx;
  fp32 wz;
  fp32 wz_set;
  fp32 chassis_relative_angle;
  fp32 chassis_relative_angle_set;
  fp32 chassis_yaw_set;
  fp32 chassis_angle_set;

  fp32 vx_max_speed;
  fp32 vx_min_speed;
  fp32 vy_max_speed;
  fp32 vy_min_speed;
  fp32 chassis_yaw;
  fp32 chassis_pitch;
  fp32 chassis_roll;

  MITMeasure_t *joint_motor[4];
  MITMeasure_t *wheel_motor[2];

  pid_type_def LegR_Pid;
  pid_type_def LegL_Pid;
  pid_type_def Tp_Pid;
  pid_type_def Turn_Pid;
  pid_type_def RollR_Pid;

  vmc_leg_t left;
  vmc_leg_t right;

  float v_set;
  float target_v;
  float x_set;
  float turn_set;
  float target_turn;
  float leg_set;
  float leg_left_set;
  float leg_right_set;
  float last_leg_set;
  float last_leg_left_set;
  float last_leg_right_set;
  float roll_set;
  float roll_target;
  float now_roll_set;

  float v_filter;
  float x_filter;

  float myPithR;
  float myPithGyroR;
  float myPithL;
  float myPithGyroL;
  float roll;
  float total_yaw;
  float theta_err;

  float turn_T;
  float leg_tp;
	
	uint8_t error_code;
  uint8_t recover_flag;
  uint8_t jump_flag;
  float jump_leg;
  uint32_t jump_time_r;
  uint32_t jump_time_l;
  uint8_t jump_status_r;
  uint8_t jump_status_l;
  uint8_t right_flag;
  uint8_t left_flag;

  //轮电机超时标志位
  uint8_t wheel_motor_timeout;
  //关节电机失能标志位
  uint8_t joint_motor_flag;


} chassis_move_t;
#endif

// 舵轮底盘（Steering_wheel）
#if ROBOT_CHASSIS == Steering_wheel

// 舵轮 6020 角度偏移结构
typedef struct
{
    fp32 now[4];      // 当前偏移角
    fp32 last[4];     // 上一次记录的角度
    fp32 initial[4];  // 初始偏移角
} wheel_angle_offset_t;

// 舵轮底盘运动数据结构
typedef struct
{
  const RC_ctrl_t *chassis_RC;               // 底盘使用的遥控器指针
  const gimbal_motor_t *chassis_yaw_motor;   // 使用 yaw 云台电机相对角度计算底盘欧拉角
  const gimbal_motor_t *chassis_pitch_motor; // 使用 pitch 云台电机相对角度计算底盘欧拉角
  const fp32 *chassis_INS_angle;             // 获取陀螺仪解算欧拉角指针
  chassis_mode_e chassis_mode;               // 底盘控制状态机
  chassis_mode_e last_chassis_mode;          // 上一次的底盘控制状态机

  chassis_motor_t chassis_3508[4];           // 3508 行走电机数据
  chassis_motor_t chassis_6020[4];           // 6020 舵向电机数据
  wheel_angle_offset_t wheel_angle_offset;   // 6020 舵向电机角度偏移量

  pid_type_def motor_speed_pid[4];           // 3508 速度 PID（命名与全向轮统一）
  pid_type_def chas_6020_angle_pid[4];       // 6020 角度环 PID
  pid_type_def chas_6020_speed_pid[4];       // 6020 速度环 PID
  pid_type_def chassis_angle_pid;            // 底盘跟随角度 PID
  pid_type_def chas_return_pid;              // 底盘回正模式 PID

  uint8_t chassis_return_flag;               // 底盘回正标志
  chassis_mode_e chassis_return_record;      // 底盘回正时的模式记录

  first_order_filter_type_t chassis_cmd_slow_set_vx; // 一阶滤波，缓慢改变 vx 设定值
  first_order_filter_type_t chassis_cmd_slow_set_vy; // 一阶滤波，缓慢改变 vy 设定值

  fp32 vx;                          // 底盘实际前进速度，前为正，单位 m/s
  fp32 vy;                          // 底盘实际横移速度，左为正，单位 m/s
  fp32 wz;                          // 底盘实际角速度，逆时针为正，单位 rad/s
  fp32 vx_set;                      // 底盘设定前进速度，前为正，单位 m/s
  fp32 vy_set;                      // 底盘设定横移速度，左为正，单位 m/s
  fp32 wz_set;                      // 底盘设定角速度，逆时针为正，单位 rad/s
  fp32 return_wz_set;               // 底盘回正时设定的角速度，逆时针为正，单位 rad/s
  fp32 chassis_relative_angle;      // 底盘与云台的相对角度，单位 rad
  fp32 chassis_relative_angle_set;  // 底盘相对云台的控制设定角度，单位 rad
  fp32 gimbal_radian_of_ecd;        // 云台 yaw 电机反馈的弧度值
  fp32 chassis_yaw_set;             // 底盘期望 yaw 角

  fp32 vx_max_speed;  // 前进方向最大速度，单位 m/s
  fp32 vx_min_speed;  // 后退方向最大速度，单位 m/s
  fp32 vy_max_speed;  // 左方向最大速度，单位 m/s
  fp32 vy_min_speed;  // 右方向最大速度，单位 m/s
  fp32 chassis_yaw;   // 陀螺仪和云台电机叠加得到的 yaw 角
  fp32 chassis_pitch; // 陀螺仪和云台电机叠加得到的 pitch 角
  fp32 chassis_roll;  // 陀螺仪和云台电机叠加得到的 roll 角

} chassis_move_t;

// 兼容旧代码中的 chas_3508_pid 写法
#define chas_3508_pid motor_speed_pid

#endif  // ROBOT_CHASSIS == Steering_wheel

// 任务接口
extern void chassis_task(void const *pvParameters);

// 遥控量 → 底盘速度设定
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

// 底盘控制设定（由 chassis_behaviour_control_set 填入 vx, vy, wz）
extern void chassis_set_contorl(chassis_move_t *chassis_move_control);

// 底盘全局运动数据
extern chassis_move_t chassis_move;
extern fp32 yaw_set;

#endif
