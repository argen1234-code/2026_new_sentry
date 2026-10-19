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
#include "shoot_3508.h"
#include "main.h"
#include "robot_param.h"
#include "cmsis_os.h"

#if ROBOT_FRICTION == friction_3508

//#include "bsp_laser.h"
#include "bsp_PWM.h"
#include "arm_math.h"
#include "referee.h"

#include "can_bsp.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"


extern MITMeasure_t *gimbal_DM2325_measure;


//开启1号摩擦轮
__attribute__((used))void fric1_on(shoot_control_t *shoot)
{	
	fp32 out = 0;
	if( shoot->fric1.shoot_motor_measure == NULL){
		return;
	}
	out = PID_Calc(&shoot->fric_motor_pid, shoot->fric1.speed, shoot->fric1.speed_set);
	shoot->fric1.give_current = out;
}

//开启2号摩擦轮
__attribute__((used))void fric2_on(shoot_control_t *shoot)
{
	fp32 out = 0;
	if( shoot->fric2.shoot_motor_measure == NULL){
		return;
	}
	out = PID_Calc(&shoot->fric_motor_pid, shoot->fric2.speed, shoot->fric2.speed_set);
	shoot->fric2.give_current = out;
}

//关闭摩擦轮
__attribute__((used))void fric_off(void){
	
	fp32 out = 0.0f;

	if( shoot_control.fric1.shoot_motor_measure == NULL || shoot_control.fric2.shoot_motor_measure == NULL )
	{
		return;
	}
	
	//在摩擦轮降速到一定程度后才设置电流值为0，起到斜坡的作用
	if(shoot_control.fric1.speed < 0.3f )
	{
		shoot_control.fric1.give_current = 0;
		return;
	}
	else
	{
		out = PID_Calc(&shoot_control.fric_motor_pid, shoot_control.fric1.speed, 0);
		shoot_control.fric1.give_current = out;
	}
	
	if(shoot_control.fric2.speed < 0.3f )
	{
		shoot_control.fric2.give_current = 0;
		return;
	}
	else
	{
		out = PID_Calc(&shoot_control.fric_motor_pid, shoot_control.fric2.speed, 0);
		shoot_control.fric2.give_current = out;
	}
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
	shoot_control.shoot_mode = SHOOT_STOP;
	shoot_control.shoot_limit = SHOOT_LIMIT;
	shoot_control.shoot_cool = SHOOT_COOL;
	shoot_control.damn_calories = DAMN_CALORIES;
	shoot_control.shoot_time = HAL_GetTick();
	shoot_control.shoot_delay = 0;
	static const fp32 fric_speed_pid[3] = {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD};
	//遥控器指针
	shoot_control.shoot_rc = get_remote_control_point();
	//电机指针
	shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
	shoot_control.fric1.shoot_motor_measure = get_fric1_motor_measure_point();
	shoot_control.fric2.shoot_motor_measure = get_fric2_motor_measure_point();
	//初始化PID
	PID_init(&shoot_control.fric_motor_pid, PID_POSITION, fric_speed_pid, FRIC_MAX_OUT, FRIC_MAX_IOUT);
	//更新数据
	shoot_set_mode();
	shoot_feedback_update();
	shoot_control.ecd_count = 0;
	shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
	shoot_control.given_current = 0;
	shoot_control.move_flag = 0;
	shoot_control.angle_set = shoot_control.angle;
	shoot_control.speed = 0.0f;
	shoot_control.speed_set = 0.0f;
	shoot_control.key_time = 0;
}

/**
  * @brief          射击状态机设置，左拨杆常态中间，拨到上方开启摩擦轮，拨回中间连续发射，再拨上→中则停止（哨兵无鼠标按键，直接连发）
  * @param[in]      void
  * @retval         void
  */
__attribute__((used))void shoot_set_mode(void)
{
	static int8_t last_s = RC_SW_MID;
	static uint8_t shoot_cycle = 0;    // 0: 停止态, 1: 发射已激活
	uint8_t current_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];

	/* UP → MID 切换检测：第一次启动发射，第二次停止 */
	if (switch_is_mid(current_s) && switch_is_up(last_s))
	{
		if (shoot_cycle == 0)
		{
				shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
			shoot_cycle = 1;
		}
		else
		{
			shoot_control.shoot_mode = SHOOT_STOP;
			shoot_cycle = 0;
		}
	}
	/* UP：启动摩擦轮 */
	else if (switch_is_up(current_s) && !switch_is_up(last_s))
	{
		if (shoot_cycle == 0 && shoot_control.shoot_mode == SHOOT_STOP)
		{
			shoot_control.shoot_mode = SHOOT_READY_FRIC;
		}
	}

	//键盘Q开启摩擦轮
	if ((shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
	{
		shoot_control.shoot_mode = SHOOT_READY_FRIC;
		shoot_cycle = 0;
	}
	//键盘E关闭摩擦轮
	else if ((shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
	{
		shoot_control.shoot_mode = SHOOT_STOP;
		shoot_cycle = 0;
	}

//	/* 摩擦轮准备完成检测（95%阈值）*/
//	if (shoot_control.shoot_mode == SHOOT_READY_FRIC  && \
//		 (shoot_control.fric1.speed <= FRIC_UP * 0.95f) && \
//		 (shoot_control.fric2.speed >= -FRIC_UP * 0.95f))
//	{
//		shoot_control.shoot_mode = SHOOT_READY_BULLET;
//	}

	//确保不是在 停止 或 摩擦轮准备 时执行发射
	if(shoot_control.shoot_mode >= SHOOT_READY_BULLET)
	{
		//单击左键且在松手时单发
		if (shoot_control.press_l == 0 && shoot_control.last_press_l && shoot_control.shoot_mode != SHOOT_CONTINUE_BULLET)
		{
			shoot_control.shoot_mode = SHOOT_BULLET;
		}
		//长按左键时进入连发
		if ((shoot_control.press_l_time >= PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
		{
			shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
		}
		//长按左键且松手时退出连发
		if(shoot_control.press_l == 0 && shoot_control.last_press_l && shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
		{
			shoot_control.shoot_mode = SHOOT_READY_BULLET;
			shoot_control.speed_set = 0.0f;
		}
	}

	//云台无力状态时关闭射击
	if (gimbal_cmd_to_shoot_stop())
	{
		shoot_control.shoot_mode = SHOOT_STOP;
	}

	last_s = current_s;
}

/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
__attribute__((used))void shoot_feedback_update(void)
{
	static fp32 speed_fliter_1 = 0.0f;
	static fp32 speed_fliter_2 = 0.0f;
	static fp32 speed_fliter_3 = 0.0f;

	//拨弹轮电机速度滤波一下
	static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

	//更新热量数据，减去热量冷却
	static uint32_t last_time = 0;
	uint32_t time = HAL_GetTick();
	shoot_control.calories -= shoot_control.shoot_cool * (time - last_time) / 1000.0f;
	last_time = time;

	//二阶低通滤波
	speed_fliter_1 = speed_fliter_2;
	speed_fliter_2 = speed_fliter_3;
	speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];


	#if ROBOT_TYPE   ==   Sentinel_robot
		shoot_control.speed = gimbal_DM2325_measure->fdb.vel;//暂时这样，只为了能打弹
	#else
		shoot_control.speed = shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;
	#endif

	//电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
	if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
	{
		shoot_control.ecd_count--;
	}
	else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
	{
		shoot_control.ecd_count++;
	}

	if (shoot_control.ecd_count == FULL_COUNT)
	{
		shoot_control.ecd_count = -(FULL_COUNT - 1);
	}
	else if (shoot_control.ecd_count == -FULL_COUNT)
	{
		shoot_control.ecd_count = FULL_COUNT - 1;
	}
	//计算输出轴角度
	shoot_control.angle = gimbal_DM2325_measure->fdb.pos;
	
	//计算摩擦轮电机的速度
	//2006电机是12位，3508是14位，所以需要除以4来转换
	shoot_control.fric1.speed = shoot_control.fric1.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED / 4.0f;
	shoot_control.fric2.speed = shoot_control.fric2.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED / 4.0f;
	shoot_control.fric1.accel = (shoot_control.fric1.shoot_motor_measure->last_ecd - shoot_control.fric1.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE / 4.0f * GIMBAL_CONTROL_TIME / 1000.0f;
	shoot_control.fric2.accel = (shoot_control.fric2.shoot_motor_measure->last_ecd - shoot_control.fric2.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE / 4.0f * GIMBAL_CONTROL_TIME / 1000.0f;

	//鼠标按键
	shoot_control.last_press_l = shoot_control.press_l;
	shoot_control.last_press_r = shoot_control.press_r;
	shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
	shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
	
	//长按计时
	if (shoot_control.press_l)
	{
		if (shoot_control.press_l_time < PRESS_LONG_TIME)
		{
			shoot_control.press_l_time++;
		}
	}
	else
	{
		shoot_control.press_l_time = 0;
	}

	if (shoot_control.press_r)
	{
		if (shoot_control.press_r_time < PRESS_LONG_TIME)
		{
			shoot_control.press_r_time++;
		}
	}
	else
	{
		shoot_control.press_r_time = 0;
	}

	//射击开关下档时间计时
	if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
	{

		if (shoot_control.rc_s_time < RC_S_LONG_TIME)
		{
			shoot_control.rc_s_time++;
		}
	}
	else
	{
		shoot_control.rc_s_time = 0;
	}
}

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
__attribute__((used))void shoot_bullet_control(void)
{
	//每次拨动 1/4PI的角度
	if (shoot_control.move_flag == 0)
	{
		shoot_control.angle_set = rad_format(shoot_control.angle - PI_2_ON_7);
		shoot_control.move_flag = 1;
	}
	//到达角度判断
	if(fabs(shoot_control.angle - shoot_control.angle_set) > 0.2f)
	{
		shoot_control.speed_set = TRIGGER_SPEED;
	}
	else
	{
		shoot_control.speed_set =  0.0f;
		shoot_control.move_flag = 0;
		shoot_control.last_calories = shoot_control.calories; 
		shoot_control.calories += shoot_control.damn_calories;
		shoot_control.shoot_mode = SHOOT_READY_BULLET;
	}
}


#endif
