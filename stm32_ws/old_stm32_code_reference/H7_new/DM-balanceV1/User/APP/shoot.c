/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "shoot_3508.h"
#include "main.h"

#include "cmsis_os.h"

//#include "bsp_laser.h"
#include "bsp_PWM.h"
#include "arm_math.h"
#include "referee.h"

#include "can_bsp.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"

#define shoot_fric1_on(shoot)		fric1_on((shoot))		//摩擦轮1宏定义
#define shoot_fric2_on(shoot)		fric2_on((shoot))		//摩擦轮2宏定义
#define shoot_fric_off()				fric_off()					//关闭两个摩擦轮

//#define shoot_laser_on()    laser_on()      //激光开启宏定义
//#define shoot_laser_off()   laser_off()     //激光关闭宏定义

//#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)		//微动开关IO


__weak void fric1_on(shoot_control_t *shoot);
__weak void fric2_on(shoot_control_t *shoot);
__weak void fric_off(void);
__weak void shoot_set_mode(void);
__weak void shoot_feedback_update(void);
__weak void trigger_motor_turn_back(void);
__weak void shoot_bullet_control(void);
__weak void shoot_init(void);


shoot_control_t shoot_control;			//射击数据

fp32 angle_set_for_calc;
fp32 angle_for_calc;

fp32 fric_up = FRIC_UP;
extern int8_t ddd;
extern uint8_t ddd_shoot;

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
float continue_speed =	CONTINUE_TRIGGER_SPEED;
int16_t shoot_control_loop(void)
{

	if (ddd_shoot == 1)
	{
		shoot_control.shoot_mode = SHOOT_READY_FRIC;
		ddd_shoot = 0;
	}
	

	
	if (gimbal_cmd_to_shoot_stop())
	{
		shoot_control.shoot_mode = SHOOT_STOP;
	}
	
	shoot_set_mode();        //设置状态机	
	shoot_feedback_update(); //更新数据
	
	if( shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_R || (switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && switch_is_mid(shoot_control.shoot_rc->rc.s[GIMBAL_MODE_CHANNEL])) ){
		Bombbay_on();
	}
	else if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_G || (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && switch_is_mid(shoot_control.shoot_rc->rc.s[GIMBAL_MODE_CHANNEL])) ){
		Bombbay_off();
	}

	//射击停止状态设置拨弹盘目标速度为0
	if (shoot_control.shoot_mode == SHOOT_STOP)
	{
		shoot_control.speed_set = 0.0f;
	}
	//摩擦轮准备状态设置拨弹盘目标速度为0
	else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
	{
		shoot_control.speed_set = 0.0f;
	}
	//可以发射状态设置拨弹盘目标速度为0
	else if (shoot_control.shoot_mode == SHOOT_READY_BULLET)
	{
		shoot_control.speed_set = 0.0f;
	}
	//单发状态执行单发函数
	else if (shoot_control.shoot_mode == SHOOT_BULLET)
	{
		shoot_bullet_control();
	}
	//连发状态设置拨弹盘转速为定值
	else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
	{
		shoot_control.speed_set = continue_speed;
	}
	
	//打开摩擦轮
	if(shoot_control.shoot_mode == SHOOT_READY_FRIC)
	{
		shoot_control.fric1.speed_set = fric_up;
		shoot_control.fric2.speed_set = -fric_up;
	}
	//关闭摩擦轮
	else if(shoot_control.shoot_mode == SHOOT_STOP)
	{
		shoot_control.fric1.speed_set = 0;
		shoot_control.fric2.speed_set = 0;
	}
	
	PID_Calc(&shoot_control.trigger_motor_speed_pid, shoot_control.speed, shoot_control.speed_set);
	
	#if ROBOT_TYPE == Sentinel_robot		
		shoot_control.given_current = (int16_t)shoot_control.speed_set;        //哨兵,直接用目标值,暂时将就
	#else			
		shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_speed_pid.out);
	#endif
	
	if(shoot_control.shoot_mode == SHOOT_STOP)
	{
		shoot_control.given_current = 0;
		fric_off();
	}
	else
	{
		shoot_control.shoot_time = HAL_GetTick();
	}
	
	//计算摩擦轮电机的电流值
	shoot_fric1_on(&shoot_control);
	shoot_fric2_on(&shoot_control);
	
	
	//发送摩擦轮电机的电流值
	if(shoot_control.fric1.shoot_motor_measure != NULL && shoot_control.fric2.shoot_motor_measure != NULL)
	{
		//过热保护，摩擦轮过烫时禁止运行摩擦轮
		if(shoot_control.fric1.shoot_motor_measure->temperate < 80.0f && \
			 shoot_control.fric2.shoot_motor_measure->temperate < 80.0f && \
			 shoot_control.shoot_mode > SHOOT_STOP )
		{
			#if ROBOT_FRICTION == friction_3508
				CAN_cmd_friction(shoot_control.fric1.give_current, shoot_control.fric2.give_current, 0, 0);
			#elif ROBOT_FRICTION == friction_double
				CAN_cmd_friction(shoot_control.fric1.give_current, shoot_control.fric2.give_current, shoot_control.fric1_.give_current, shoot_control.fric2_.give_current);
			#endif
		}
		else
		{
			CAN_cmd_friction(0,0,0, 0);
		}
	}
	else
	{
		CAN_cmd_friction(0,0,0, 0);
	}
	
	//遥控器中档禁止发弹
	if(gimbal_cmd_to_shoot_stop())
	{
		shoot_control.given_current = 0;
	}
	
	//拨弹盘的控制电流值
  return shoot_control.given_current;
}



//射击热量检测
void shoot_calories_detect()
{		
	shoot_control.last_calories = shoot_control.calories;
	shoot_control.calories += shoot_control.damn_calories;
	shoot_control.shoot_delay = HAL_GetTick() - shoot_control.shoot_time;
}
