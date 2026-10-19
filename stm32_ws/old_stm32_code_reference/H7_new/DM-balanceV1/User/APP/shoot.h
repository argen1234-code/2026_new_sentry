/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能。
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

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "can_bsp.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"

//大小弹丸的单发热量
#define big_damn_calories 		100 
#define small_damn_calories 	10


typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY_FRIC,
    SHOOT_READY_BULLET,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_CONTINUE_BULLET,
    SHOOT_DONE,
} shoot_mode_e;

typedef struct
{
  const motor_measure_t *shoot_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} shoot_motor_t;

typedef struct
{
	shoot_mode_e shoot_mode;
	const RC_ctrl_t *shoot_rc;
	const motor_measure_t *shoot_motor_measure;

	//一级摩擦轮
	shoot_motor_t fric1;		//从后顺着枪管往前看，处于左边的摩擦轮
	shoot_motor_t fric2;		//是右边摩擦轮
	//二级摩擦轮
	shoot_motor_t fric1_;
	shoot_motor_t fric2_;

	ramp_function_source_t fric1_ramp;
	uint16_t fric_pwm1;
	ramp_function_source_t fric2_ramp;
	uint16_t fric_pwm2;
	pid_type_def trigger_motor_speed_pid;   //哨兵用的是DM用不到内环pid
	pid_type_def fric_motor_pid;
	fp32 trigger_speed_set;
	fp32 speed;
	fp32 speed_set;
	fp32 angle;
	fp32 angle_set;
	int16_t given_current;
	int8_t ecd_count;

	bool_t press_l;
	bool_t press_r;
	bool_t last_press_l;
	bool_t last_press_r;
	uint16_t press_l_time;
	uint16_t press_r_time;
	uint16_t rc_s_time;

	uint16_t block_time;
	uint16_t reverse_time;
	bool_t move_flag;

	bool_t key;
	uint8_t key_time;

	uint16_t heat_limit;
	uint16_t heat;
	uint32_t shoot_time;
	int16_t shoot_delay; //ms,发弹延时，后面需要去测试
	
	//射击热量
	fp32 last_calories;
	fp32 calories;
	uint16_t shoot_limit;		//热量限制
	uint8_t shoot_cool;			//热量冷却
	uint8_t damn_calories;	//弹丸热量
} shoot_control_t;

extern shoot_control_t shoot_control; 

//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);

#endif
