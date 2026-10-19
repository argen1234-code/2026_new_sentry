#ifndef _BALANCE_CHASSIS_H
#define _BALANCE_CHASSIS_H
#include "chassis_task.h"
#include "chassis_behaviour.h"
#if ROBOT_CHASSIS == Balance_wheel
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "can_bsp.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
//#include "iwdg.h"
#include "slip_control.h"
#include "robot_param.h"

#define MOTOR_DISTANCE_TO_CENTER      0.22f


#define P_MIN_3519 -12.5f		//位置最小值
#define P_MAX_3519 12.5f		//位置最大值
#define V_MIN_3519 -200.0f					//速度最小值
#define V_MAX_3519 200.0f						//速度最大值

#define KP_MIN_3519 0.0f					//Kp最小值
#define KP_MAX_3519 500.0f				//Kp最大值
#define KD_MIN_3519 0.0f					//Kd最小值
#define KD_MAX_3519 5.0f					//Kd最大值
#define TP_MAX_3519 10.0f
#define TP_MIN_3519 -10.0f

#define P_MIN_03 -12.5663704f		//位置最小值
#define P_MAX_03 12.5663704f		//位置最大值
#define V_MIN_03 -45						//速度最小值
#define V_MAX_03 45							//速度最大值
#define KP_MIN_03 0.0f					//Kp最小值
#define KP_MAX_03 500.0f				//Kp最大值
#define KD_MIN_03 0.0f					//Kd最小值
#define KD_MAX_03 5.0f					//Kd最大值
#define TP_MAX_03 60.0f			
#define TP_MIN_03 -60.0f


#ifndef JOINT_RF_ID

//相对与开发板的右前方关节电机id
#define JOINT_RF_ID 0X06
//相对与开发板的右后方关节电机id
#define JOINT_RB_ID 0X07
//相对与开发板的左前方关节电机id
#define JOINT_LF_ID 0X08
//相对与开发板的左后方关节电机id
#define JOINT_LB_ID 0X09
//相对与开发板的右边的轮毂电机id
#define WHEEL_RIGHT_ID 0X10
//相对与开发板的左边的轮毂电机id
#define WHEEL_LEFT_ID 0X11
#endif

//（改减速机）轮毂电机克服静摩擦力的值，刚好可以匀速转动
#define WHEEL_MOTOR1_FRE 0.13f
#define WHEEL_MOTOR2_FRE 0.15f

//#define WHEEL_MOTOR1_FRE 0.13f
//#define WHEEL_MOTOR2_FRE 0.13f
#define MOTOR_REDUCTION 21/19.0f

#define KEY_JUMP KEY_PRESSED_OFFSET_B 

extern void chassis_init(chassis_move_t *chassis_move_init);
extern void chassis_set_mode(chassis_move_t *chassis_move_mode);
extern void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
extern void chassis_feedback_update(chassis_move_t *chassis_move_update);
extern void chassis_set_contorl(chassis_move_t *chassis_move_control);
extern void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
extern void chassis_send_cmd(chassis_move_t *chassis_send_cmd);
extern void observer(chassis_move_t *chassis_move, KalmanFilter_t *vxEstimateKF, KalmanFilter_t *vyEstimateKF);
#endif

#endif
