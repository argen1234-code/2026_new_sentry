/**
  ****************************(C) COPYRIGHT 2025 PRINTK****************************
  * @file       shoot.c/h
  * @brief      3508射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     9-14-2018       yibu              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 PRINTK****************************
  */

#include "shoot.h"
#include "shoot_double.h"
#include "main.h"
#include "robot_param.h"
#include "cmsis_os.h"
//没有发射机构，全部置空
#if ROBOT_FRICTION == friction_none
//#include "bsp_laser.h"
#include "bsp_PWM.h"
#include "arm_math.h"
#include "referee.h"

#include "can_bsp.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"



__attribute__((used))void fric1_on(shoot_control_t *shoot){
	
}

__attribute__((used))void fric2_on(shoot_control_t *shoot){	
	
}

__attribute__((used))void fric_off(void){
	
}

	/* 函数原型声明 */
	void shoot_set_mode(void);
	void shoot_feedback_update(void);
/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
__attribute__((used))void shoot_init(void)
{
		
}

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
__attribute__((used))void shoot_set_mode(void)
{
   
}

/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
__attribute__((used))void shoot_feedback_update(void)
{

  
}

__attribute__((used))void trigger_motor_turn_back(void)
{
    
}
/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
__attribute__((used))void shoot_bullet_control(void)
{
   
}


#endif
