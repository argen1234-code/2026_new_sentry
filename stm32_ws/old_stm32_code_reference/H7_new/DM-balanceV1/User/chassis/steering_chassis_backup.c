/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c
  * @brief     	底盘控制任务
	*
	*     000000000000000     00               00    00     00   00         00 
	*           00     0      00                00    00   00     00       00  
	*       00  00000        00000000000000      00  000000000   00000000000000
	*       00  00          00           00    00    000000000     00     00   
	*      00000000000     00  000000    00     000     00           000000    
	*     00    00   0000     00    00   00      00   000000           00      
	*       00000000000       00    00   00           000000           00      
	*       0   00    0       0000000 00 00       00    00       00000000000000
	*       00000000000       00       000       00 00000000000        00      
	*           00            00                000 00000000000        00      
	*           00  00        00          0    000      00             00      
	*      000000000000        00        000  000       00          00 00      
	*       00        00        000000000000            00            00       
	********************************************************************************/
#include "steering_chassis.h"
#include "chassis_task.h"	
#include "chassis_behaviour.h"
#if ROBOT_CHASSIS == Steering_wheel 
#include "observer.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "can_bsp.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "iwdg.h"
#include "slip_control.h"
#include "robot_param.h"
#include "CAN_receive.h"
/********************************************
 * * * * * * * * * * * * * * * * * * * * * * 
	PID计算里面若有单独加减的变量, 均为偏移量
 * * * * * * * * * * * * * * * * * * * * * * 
 ********************************************/

#if INCLUDE_uxTaskGetStackHighWaterMark
	uint32_t chassis_high_water;
#endif


#define rc_deadband_limit(input, output, dealine)	  \
{                                                		\
	if ((input) > (dealine) || (input) < -(dealine))	\
	{                                             		\
		(output) = (input);                          		\
	}                                             		\
	else                                          		\
	{                                             		\
		(output) = 0;                          					\
	}                                          				\
}


/**
  * @brief          底盘测量数据更新，包括3508电机速度、6020电机角度、欧拉角度、机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
__attribute__((used))void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	if (chassis_move_update == NULL) return;
	
//	//定义两个变量用于记录车轮当前角度和速度
//	fp32 wheel_angle_now[4], wheel_speed_speed[4];
	
	for (uint8_t i = 0; i < 4; i++)
	{
		//更新3508电机速度和加速度，加速度是速度的PID微分
		chassis_move_update->chassis_3508[i].speed = chassis_move_update->chassis_3508[i].chassis_motor_measure->speed_rpm / MPS_to_RPM;
//		wheel_speed_speed[i] = chassis_move_update->chassis_3508[i].speed;
		chassis_move_update->chassis_3508[i].accel = chassis_move_update->chas_3508_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
		//更新6020电机角度
		chassis_move_update->chassis_6020[i].angle = rad_format(chassis_move_update->chassis_6020[i].chassis_motor_measure->ecd / GM6020_Angle_Ratio);
//		wheel_angle_now[i] = chassis_move_update->chassis_6020[i].angle;
	}
	
	//底盘有陀螺仪
	chassis_move_update->chassis_yaw 	 = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET	 ));
	chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET));
	chassis_move_update->chassis_roll	 = rad_format(*(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET ));
		
//	//用运动学逆解算更新底盘纵向速度 x, 平移速度 y, 旋转速度 wz, 坐标系为右手系(前x左y上z)
//	chas_for_cal(wheel_angle, wheel_speed, &chassis_move_update->vx, &chassis_move_update->vy, &chassis_move_update->wz);
}

/**
  * @brief          初始化"chassis_move"变量，包括pid初始化, 遥控器指针初始化, 3508底盘电机指针初始化, 云台电机初始化, 陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
__attribute__((used))void chassis_init(chassis_move_t *chassis_move_init)
{
	if (chassis_move_init == NULL) return;

	//底盘3508速度环pid值
	const static fp32 chas_3508_pid_param[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	//底盘6020角度环pid值
	const static fp32 chas_6020_angle_pid_param[3] = {GM6020_MOTOR_ANGLE_PID_KP, GM6020_MOTOR_ANGLE_PID_KI, GM6020_MOTOR_ANGLE_PID_KD};
	//底盘6020速度环pid值
	const static fp32 chas_6020_speed_pid_param[3] = {GM6020_MOTOR_SPEED_PID_KP, GM6020_MOTOR_SPEED_PID_KI, GM6020_MOTOR_SPEED_PID_KD};
	//底盘角度pid值
	const static fp32 chassis_yaw_pid_param[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
	
	//回正模式pid值
	const static fp32 chassis_yaw_return_pid_param[3] = {YAW_RETURN_PID_KP, YAW_RETURN_PID_KI, YAW_RETURN_PID_KD};
	
	//底盘开机状态为原始
	chassis_move_init->chassis_mode = CHASSIS_VECTOR_NO_MOVE;
	//获取遥控器指针
	chassis_move_init->chassis_RC = get_remote_control_point();
	//获取陀螺仪姿态角指针
	chassis_move_init->chassis_INS_angle = get_INS_angle_point();
	//获取云台电机数据指针
	chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
	chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
	
	//获取底盘电机数据指针，初始化PID 
	for(uint8_t i = 0; i < 4; i++)
	{
		chassis_move_init->chassis_3508[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
		PID_init(&chassis_move_init->chas_3508_pid[i], PID_USUAL, chas_3508_pid_param, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
		chassis_move_init->chassis_6020[i].chassis_motor_measure = get_chassis_motor_measure_point(i + 4);
		PID_init(&chassis_move_init->chas_6020_angle_pid[i], PID_USUAL, chas_6020_angle_pid_param, GM6020_MOTOR_ANGLE_PID_MAX_OUT, GM6020_MOTOR_ANGLE_PID_MAX_IOUT);
		PID_init(&chassis_move_init->chas_6020_speed_pid[i], PID_USUAL, chas_6020_speed_pid_param, GM6020_MOTOR_SPEED_PID_MAX_OUT, GM6020_MOTOR_SPEED_PID_MAX_IOUT);
	}
	//初始化回正模式pid
	PID_init(&chassis_move_init->chas_return_pid, PID_USUAL, chassis_yaw_return_pid_param, YAW_RETURN_PID_MAX_OUT, YAW_RETURN_PID_MAX_IOUT);
	
	//初始化角度PID
	PID_init(&chassis_move_init->chassis_angle_pid, PID_USUAL, chassis_yaw_pid_param, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	
	//用一阶滤波代替斜波函数生成
	const static fp32 chassis_x_order_filter = CHASSIS_ACCEL_X_NUM;
	const static fp32 chassis_y_order_filter = CHASSIS_ACCEL_Y_NUM;
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, &chassis_x_order_filter);
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, &chassis_y_order_filter);
		
	//最大 最小速度
	chassis_move_init->vx_max_speed =  NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vy_max_speed =  NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	
	//底盘舵向电机角度初始化
	chassis_wheel_angle_offset_init();
	
	//底盘回正标志位初始化为 1
	chassis_move_init->chassis_return_flag = 1;
	
	//更新一下数据
	chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          用遥控器或键盘设置底盘控制模式
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
__attribute__((used))void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	if(chassis_move_mode == NULL) return;
	chassis_behaviour_mode_set(chassis_move_mode);	//in file "chassis_behaviour.c"
}

/**
  * @brief          底盘模式改变
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
__attribute__((used))void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
	if(chassis_move_transit == NULL) return;
	
	//切入跟随云台模式
	if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_MOVE) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_MOVE)
	{
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
	}
	//切入跟随云台模式
	else if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
	{
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
	}
	//切入底盘旋转模式
	else if((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_SPIN) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_SPIN)
	{
		//这个值待修正，因为云台与底盘的角度差不可能完全等于底盘陀螺仪的yaw值(yaw值还有初始偏移)
		//不仅要考虑底盘陀螺仪yaw值的改变量，还需要考虑云台的朝向(云台yaw电机编码值)
		chassis_move_transit->chassis_relative_angle_set = chassis_move_transit->chassis_yaw;
	}
	chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          设置底盘控制设置值, 三运动控制值是通过 chassis_behaviour_control_set 函数设置的
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
__attribute__((used))void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
	if (chassis_move_control == NULL) return;
	
	fp32 vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

	//获取三个控制设置值
	chassis_behaviour_control_set(&vx_set, &vy_set, &wz_set, chassis_move_control);
	wz_set = - wz_set;
	
	
	if (chassis_move_control->chassis_mode == CHASSIS_RT_FOLLOW_GIMBAL_YAW_SPIN) //底盘零点回当前云台朝向
	{
		
			//“wz_set”是旋转速度设置，此时为后面pid算出的值
		chassis_move_control->wz_set = -chassis_move_control->return_wz_set * 0.00004f;  //输出最大为 限幅 * 0.00004
		//chassis_move_control->wz_set = 0.0;
		if (fabs(chassis_move_control->wz_set) < 0.001f && chassis_move_control->chassis_return_record == CHASSIS_RT_FOLLOW_GIMBAL_YAW_SPIN)  //回正后恢复正常控制
		{
			chassis_move_control->wz_set = 0;
			chassis_move_control->chassis_return_flag = 0;  //这是在左档依旧打下时退出回正模式转入正常控制的关键
		}
		
		//为满足旋转变换而定义的数组变量
		fp32 vector[2] = {vx_set, vy_set};
		
		//对速度向量旋转变换
		vector_rotate(chassis_move_control->chassis_relative_angle_set + chassis_move_control->gimbal_radian_of_ecd -2.0f, vector);
		
		vx_set = vector[0];
		vy_set = vector[1];
		//速度限幅
		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
	
	//跟随云台模式
	if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
	{
		
		//计算旋转PID角速度
		chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
		//为满足旋转变换而定义的数组变量
		fp32 vector[2] = {vx_set, vy_set};
		//对速度向量旋转变换
		vector_rotate(chassis_move_control->gimbal_radian_of_ecd -2.0f, vector);
		
		vx_set = vector[0];
		vy_set = vector[1];
		//速度限幅
		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
	//底盘旋转
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_SPIN)
	{
		//“wz_set”是旋转速度设置
		chassis_move_control->wz_set = wz_set;
		
		if (chassis_move_control->chassis_mode == CHASSIS_RT_FOLLOW_GIMBAL_YAW_SPIN)  
		{
			chassis_move_control->wz_set = 0;
		}
		
		//为满足旋转变换而定义的数组变量
		fp32 vector[2] = {vx_set, vy_set};
		
		//对速度向量旋转变换
		vector_rotate(chassis_move_control->chassis_relative_angle_set + chassis_move_control->gimbal_radian_of_ecd + this_offset, vector);
		
		vx_set = vector[0];
		vy_set = vector[1];
		
		//速度限幅
		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
	
}

/*************************************************************
  * @brief          底盘在抑制舵向电机角度跳变下的PID计算
  * @param[in]			chassis_pid_calc	底盘运动数据结构体指针
  * @retval         none
  ************************************************************/
static void PID_Calc_Jump_Restrain(chassis_move_t *chassis_pid_calc)
{
	
	for(uint8_t i = 0; i < 4; i++)
	{
		if (chassis_pid_calc->chassis_mode == CHASSIS_RT_FOLLOW_GIMBAL_YAW_SPIN)
		{
			fp32 taget = rad_format(3.55);
			fp32 actual = rad_format(chassis_pid_calc->gimbal_radian_of_ecd);
			
			//避免死区通用思路
			if(fabs(actual - taget) > PI) //这是腾哥说的模式的pid，
			{
				if(taget < 0)
				{
					chassis_pid_calc->return_wz_set = PID_calc(&chassis_pid_calc->chas_return_pid, actual, taget + 2*PI);
				}
				else
				{
					chassis_pid_calc->return_wz_set = PID_calc(&chassis_pid_calc->chas_return_pid, actual, taget - 2*PI);
				}
			}
			else
			{
				chassis_pid_calc->return_wz_set = PID_calc(&chassis_pid_calc->chas_return_pid, actual, taget);
			}
			
  	}
		
		// 检查是否为无控制状态
    if(fabsf(chassis_pid_calc->vx_set) < 0.1f && fabsf(chassis_pid_calc->vy_set) < 0.1f && fabsf(chassis_pid_calc->wz_set) < 0.1f)
    {
			//控制模式，保持之前角度
			if(chassis_pid_calc->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
			{
				int32_t count = 0;
				if (chassis_pid_calc->chassis_return_record != chassis_pid_calc->chassis_mode)
				{
					count = 2001;
				}
				if (count > 1)
				{
					count --;
			    chassis_pid_calc->chassis_6020[i].angle_set = rad_format(-chassis_pid_calc->wheel_angle_offset.initial[i] + 1.15f + chassis_pid_calc->gimbal_radian_of_ecd);
					chassis_pid_calc->wheel_angle_offset.last[i] = chassis_pid_calc->chassis_6020[i].angle;
				}
				else
				{
				  chassis_pid_calc->chassis_6020[i].angle_set = chassis_pid_calc->wheel_angle_offset.last[i];
				}
				
//		 wheel_angle_offset[i] = -last_angle_set[i];  //下一次控制前进时以当前方向前进
				
			}
			//无力模式，刷新小车至初始状态
			else if(chassis_pid_calc->chassis_mode == CHASSIS_VECTOR_NO_MOVE)  
			{
				chassis_pid_calc->chassis_3508[i].speed_set = 0;
				chassis_pid_calc->chassis_6020[i].angle_set = chassis_pid_calc->wheel_angle_offset.last[i];  //保持之前角度
//				chassis_pid_calc->chassis_6020[i].angle_set = -chassis_pid_calc->wheel_angle_offset.initial[i]; 
		  	chassis_pid_calc->wheel_angle_offset.now[i] = chassis_pid_calc->wheel_angle_offset.initial[i];  
			}
			//小陀螺模式，恢复初始角度
			else if(chassis_pid_calc->chassis_mode == CHASSIS_VECTOR_SPIN)
			{
		  	chassis_pid_calc->wheel_angle_offset.now[i] = chassis_pid_calc->wheel_angle_offset.initial[i];
			}
    }
		//有控制时记录角度
		else
		{ 
			chassis_pid_calc->wheel_angle_offset.last[i] = chassis_pid_calc->chassis_6020[i].angle;
		}
		
		chassis_pid_calc->chassis_return_record = chassis_pid_calc->chassis_mode;
		
					//PID计算，得出3508的赋值电流值
		PID_calc(&chassis_pid_calc->chas_3508_pid[i], chassis_pid_calc->chassis_3508[i].speed, chassis_pid_calc->chassis_3508[i].speed_set);
		
		//PID计算，得出最终的6020赋值电流值
		if(fabs(chassis_pid_calc->chassis_6020[i].angle - chassis_pid_calc->chassis_6020[i].angle_set) > PI)
		{
			if(chassis_pid_calc->chassis_6020[i].angle_set < 0)
			{
			  PID_calc(&chassis_pid_calc->chas_6020_angle_pid[i], chassis_pid_calc->chassis_6020[i].angle, chassis_pid_calc->chassis_6020[i].angle_set + 2*PI);
			}
			else
			{
				PID_calc(&chassis_pid_calc->chas_6020_angle_pid[i], chassis_pid_calc->chassis_6020[i].angle, chassis_pid_calc->chassis_6020[i].angle_set - 2*PI);
			}
	  	PID_calc(&chassis_pid_calc->chas_6020_speed_pid[i], chassis_pid_calc->chassis_6020[i].chassis_motor_measure->speed_rpm, chassis_pid_calc->chas_6020_angle_pid[i].out);
		}
		else
		{
			PID_calc(&chassis_pid_calc->chas_6020_angle_pid[i], chassis_pid_calc->chassis_6020[i].angle, chassis_pid_calc->chassis_6020[i].angle_set);
			PID_calc(&chassis_pid_calc->chas_6020_speed_pid[i], chassis_pid_calc->chassis_6020[i].chassis_motor_measure->speed_rpm, chassis_pid_calc->chas_6020_angle_pid[i].out);
		}
	}
}

/******************************** 核心 ********************************
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  *********************************************************************/
__attribute__((used))void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	fp32 wheel_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	
	//底盘舵轮运动学逆解算
	chas_inv_cal(chassis_move_control_loop->vx_set,
							 chassis_move_control_loop->vy_set,
							 chassis_move_control_loop->wz_set,
							 wheel_angle, wheel_speed);
		
	//将算出的舵轮目标角度和目标速度赋值给angle_set、speed_set
	for(uint8_t i = 0; i < 4; i ++)
	{
		chassis_move_control_loop->chassis_6020[i].angle_set = wheel_angle[i];
		chassis_move_control_loop->chassis_3508[i].speed_set = wheel_speed[i];
	}
	
//	//打滑抑制
//	slip_control(chassis_move_control_loop);
	
	//底盘在抑制舵向电机角度跳变下的PID计算，定义在这个函数前面
	PID_Calc_Jump_Restrain(chassis_move_control_loop);
	
	//功率控制(在 chassis_power_control() 函数中给舵轮3508和6020电机赋值电流值)
	chassis_power_control(chassis_move_control_loop);
}

#endif

