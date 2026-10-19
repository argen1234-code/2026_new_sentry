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
#if ROBOT_FRICTION == friction_3508
#include "can_bsp.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"


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
		shoot_motor_t fric1;
		shoot_motor_t fric2;
	
    ramp_function_source_t fric1_ramp;
    uint16_t fric_pwm1;
    ramp_function_source_t fric2_ramp;
    uint16_t fric_pwm2;
    pid_type_def trigger_motor_pid;
		pid_type_def fric_motor_pid;
    fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
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
		int16_t shoot_delay; //ms,发弹延时，后面需要去测试
} shoot_control_t;

//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
extern __attribute__((used))void shoot_init(void);
extern __attribute__((used))void shoot_set_mode(void);
extern __attribute__((used))void shoot_feedback_update(void);
extern __attribute__((used))void shoot_bullet_control(void);


extern void fric1_on(shoot_control_t *shoot);
extern void fric2_on(shoot_control_t *shoot);
extern void fric_off(void);

#endif

#endif
