/**
  *********************************************************************
  * @file      observe_task.c/h
  * @brief     该任务是与裁判系统进行交互，绘制UI
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "UI.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot.h"
#include "auto_aim.h"
#include "referee.h"
#include "protocol.h"

#include "ui_g.h"
// [SYNC_FROM_H] Synced UI task from H:\DM-balanceV1\User\APP
// 自瞄状态
const char aim_mode[2][4] = {" ON", "OFF"};

// 射击状态
const char shoot_mode[5][6] = {" OFF ", "FRICT", "READY", "BURST", "DONE "};

// 云台状态
const char gimbal_mode[3][6] = {" NAN ", "REAL ", "SPIN "};

// 底盘状态
const char chassis_mode[4][6] = {" NAN ", "REAL ", "C_YAW", "SPIN "};
//超电电压
int32_t super_cap_volt = 0;

fp32 get_cap_volt(void);

void UI_init(void);
void UI_set(void);
void UI_update(void);
void UI_loop(void);
void UI_send_cmd(void);

void UI_task(void const * argument){
	osDelay(500);
	UI_init();
	while(1){
		UI_set();
		UI_update();
		UI_loop();
		osDelay(UI_TIME);
	}
}

void UI_init(void)
{
	ui_init_g();
}
void UI_set(void)
{
	static uint8_t color = 0;
//	ui_self_id = get_robot_id();
	if(switch_is_down(chassis_move.chassis_RC->rc.s[SHOOT_RC_MODE_CHANNEL ]))
	{
		ui_init_g();
	}
		if(ui_self_id > 100)
	{
		color = 0;
	}
	else
	{
		color = 1;
	}
//	ui_g_Ungroup_super_capacity_num->color = color;
	
}
void UI_update(void)
{
	//获取超电电压
	super_cap_volt = (int32_t)get_cap_volt();
	ui_self_id = get_robot_id();//蓝3
	//根据情况修改显示内容
//	strcpy(ui_g_Ungroup_chassis_dis->string, chassis_mode[1]);
//	strcpy(ui_g_Ungroup_friction_mode->string, shoot_mode[0]);
//	strcpy(ui_g_Ungroup_gimbal_dis->string, gimbal_mode[0]);
//	ui_g_Ungroup_super_capacity_num->number = super_cap_volt;
}

void UI_loop(void)
{
	ui_update_g();
}

fp32 get_cap_volt(void)
{
	static fp32 i = 0.0f;
	static uint32_t last_time = 0;
	if(HAL_GetTick() - last_time > 1000){
		i = (i > 20.0f) ? 0.0f : i + 1.0f;
		last_time = HAL_GetTick();
	}
	return i;
}
