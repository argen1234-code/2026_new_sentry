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
#if ROBOT_FRICTION == friction_double
//#include "bsp_laser.h"
#include "bsp_PWM.h"
#include "arm_math.h"
#include "referee.h"

#include "can_bsp.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "auto_aim.h"



__attribute__((used))void fric1_on(shoot_control_t *shoot){
	
	fp32 out = 0;
	if( shoot->fric1.shoot_motor_measure == NULL || shoot->fric1_.shoot_motor_measure == NULL){
		return;
	}
	out = PID_Calc(&shoot->fric_motor_pid, shoot->fric1.speed, shoot->fric1.speed_set);
	shoot->fric1.give_current = out;
	
	out = PID_Calc(&shoot->fric_motor_pid, shoot->fric1_.speed, shoot->fric1_.speed_set);
	shoot->fric1_.give_current = out;
	
}

__attribute__((used))void fric2_on(shoot_control_t *shoot){	
		fp32 out = 0;
		if( shoot->fric2.shoot_motor_measure == NULL || shoot->fric2_.shoot_motor_measure == NULL){
			return;
		}
		
		out = PID_Calc(&shoot->fric_motor_pid, shoot->fric2.speed, shoot->fric2.speed_set);
		shoot->fric2.give_current = out;
		
		out = PID_Calc(&shoot->fric_motor_pid, shoot->fric2_.speed, shoot->fric2_.speed_set);
		shoot->fric2_.give_current = out;
}

__attribute__((used))void fric_off(void){
	
		fp32 out = 0.0f;
	
		if( shoot_control.fric2.shoot_motor_measure == NULL ||shoot_control.fric2_.shoot_motor_measure == NULL || \
			  shoot_control.fric1.shoot_motor_measure == NULL ||shoot_control.fric1_.shoot_motor_measure == NULL ){
			shoot_control.fric2.give_current = 0;
			shoot_control.fric2_.give_current = 0;
			shoot_control.fric1.give_current = 0;
			shoot_control.fric1_.give_current = 0;
			return;
		}
		
		if(shoot_control.fric2.speed < 0.3f ){
			//out = PID_Calc(&shoot_control.fric_motor_pid, shoot_control.fric2.speed, 0);
			shoot_control.fric2.give_current = 0;
			
		}else{
			out = PID_Calc(&shoot_control.fric_motor_pid, shoot_control.fric2.speed, 0);
			shoot_control.fric2.give_current = out;
		}
		
		if(shoot_control.fric1.speed < 0.3f ){
			//out = PID_Calc(&shoot_control.fric_motor_pid, shoot_control.fric1.speed, shoot_control.fric1.speed);
			shoot_control.fric1.give_current = 0;
		}else{
			out = PID_Calc(&shoot_control.fric_motor_pid, shoot_control.fric1.speed, 0);
			shoot_control.fric1.give_current = out;
		}
		
		if(shoot_control.fric1_.speed < 0.3f ){
			//out = PID_Calc(&shoot_control.fric_motor_pid, shoot_control.fric1.speed, shoot_control.fric1.speed);
			shoot_control.fric1_.give_current = 0;
		}else{
			out = PID_Calc(&shoot_control.fric_motor_pid, shoot_control.fric1_.speed, 0);
			shoot_control.fric1_.give_current = out;
		}
		
		if(shoot_control.fric2_.speed < 0.3f ){
			//out = PID_Calc(&shoot_control.fric_motor_pid, shoot_control.fric1.speed, shoot_control.fric1.speed);
			shoot_control.fric2_.give_current = 0;
		}else{
			out = PID_Calc(&shoot_control.fric_motor_pid, shoot_control.fric2_.speed, 0);
			shoot_control.fric2_.give_current = out;
		}
		return;
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
		shoot_control.shoot_limit = SHOOT_LIMIT;
		shoot_control.shoot_cool = SHOOT_COOL;
		shoot_control.damn_calories = DAMN_CALORIES;
		shoot_control.shoot_time = HAL_GetTick();
		shoot_control.shoot_delay = 0;
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
		static const fp32 Fric_speed_pid[3] = {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
    //电机指针
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
		shoot_control.fric1.shoot_motor_measure = get_fric1_motor_measure_point();
		shoot_control.fric2.shoot_motor_measure = get_fric2_motor_measure_point();
		shoot_control.fric1_.shoot_motor_measure = get_fric1__motor_measure_point();
		shoot_control.fric2_.shoot_motor_measure = get_fric2__motor_measure_point();
    //初始化PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
		PID_init(&shoot_control.fric_motor_pid, PID_POSITION, Fric_speed_pid, FRIC_MAX_OUT, FRIC_MAX_IOUT);
    //更新数据
		shoot_set_mode();
    shoot_feedback_update();
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
}

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
__attribute__((used))void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP;
    uint8_t current_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];

    /* 上拨状态切换 */
    if (switch_is_up(current_s))
    {
        if (!switch_is_up(last_s)) // 上升沿检测
        {
            // 切换 STOP <-> READY_FRIC
            shoot_control.shoot_mode = (shoot_control.shoot_mode == SHOOT_STOP) ? SHOOT_READY_FRIC : SHOOT_STOP;
        }
    }
    /* 下拨触发射击 */
    else if (!switch_is_up(current_s)) 
    {
            // 在准备完成状态允许触发
            if (shoot_control.shoot_mode == SHOOT_READY_BULLET || 
                shoot_control.shoot_mode == SHOOT_READY) 
            {
                shoot_control.shoot_mode = SHOOT_BULLET;
            }
    }
		
    //可以使用键盘开启摩擦轮
    if ((shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    // 可以使用键盘关闭摩擦轮
    else if ((shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    // auto-aim: when target is valid, arm friction wheels automatically
    if (aim.auto_aim_flag == AIM_ON && aim.online && auto_aim_target_valid())
    {
        if (shoot_control.shoot_mode == SHOOT_STOP)
        {
            shoot_control.shoot_mode = SHOOT_READY_FRIC;
        }
    }

		
    /* 摩擦轮准备完成检测（95%阈值）*/
    if (shoot_control.shoot_mode == SHOOT_READY_FRIC && 
        shoot_control.fric1.shoot_motor_measure->speed_rpm >= FRIC_UP * 0.5f && 
				shoot_control.fric1.shoot_motor_measure->speed_rpm >= FRIC_UP * 0.5f)
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET;
    }
		
//    get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
		
		if(shoot_control.shoot_mode >= SHOOT_READY_FRIC)
    {		 
				if (shoot_control.press_l && shoot_control.last_press_l == 0)
        {
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
        //鼠标长按一直进入射击状态 保持连发
        if ((shoot_control.press_l_time >= PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
        {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
        }
        else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode =SHOOT_READY_BULLET;
        }
					
    }

    //下一发打蛋超热量就停
		if(shoot_control.shoot_limit - shoot_control.calories < shoot_control.damn_calories)
		{
				shoot_control.shoot_mode = SHOOT_DONE;
		}
    //如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
    // 保留原有热量和云台状态检测
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
		shoot_control.calories = (shoot_control.calories <= 0) ? 0 : shoot_control.calories;
    last_time = time;

    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

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
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
		//计算摩擦轮电机的速度
		//2006电机是12位，3508是14位，所以需要除以4来转换
		shoot_control.fric1.speed = shoot_control.fric1.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED / 4.0f;
		shoot_control.fric2.speed = shoot_control.fric2.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED/ 4.0f;
		shoot_control.fric1.accel = (shoot_control.fric1.shoot_motor_measure->last_ecd - shoot_control.fric1.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE / 4.0f * GIMBAL_CONTROL_TIME / 1000.0f;
		shoot_control.fric2.accel = (shoot_control.fric2.shoot_motor_measure->last_ecd - shoot_control.fric2.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE / 4.0f * GIMBAL_CONTROL_TIME / 1000.0f;
		
		shoot_control.fric1_.speed = shoot_control.fric1_.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;
		shoot_control.fric2_.speed = shoot_control.fric2_.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;
		shoot_control.fric1_.accel = (shoot_control.fric1_.shoot_motor_measure->last_ecd - shoot_control.fric1_.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE / 4.0f * GIMBAL_CONTROL_TIME / 1000.0f;
		shoot_control.fric2_.accel = (shoot_control.fric2_.shoot_motor_measure->last_ecd - shoot_control.fric2_.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE / 4.0f * GIMBAL_CONTROL_TIME / 1000.0f;
																																																					
    //鼠标按键
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    uint8_t aa_fire_pulse = 0;  // 自动瞄准射击脉冲信号
    // 当自动瞄准开启且在线时，获取自动瞄准射击脉冲
    if (aim.auto_aim_flag == AIM_ON && aim.online) {
        aa_fire_pulse = auto_aim_fire_pulse();
    }
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l || aa_fire_pulse;
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
		
		//打开摩擦轮
		if(shoot_control.shoot_mode == SHOOT_READY_FRIC){
			//直接高速射击
			shoot_control.fric1.speed_set = -(FRIC_UP - 4);
			shoot_control.fric2.speed_set = FRIC_UP - 4;
			//直接高速射击
			shoot_control.fric1_.speed_set = -FRIC_UP ;
			shoot_control.fric2_.speed_set = FRIC_UP ;
		}
		//关闭摩擦轮
		else if(shoot_control.shoot_mode == SHOOT_STOP){
			shoot_control.fric1.speed_set = FRIC_DOWN;
			shoot_control.fric2.speed_set = FRIC_DOWN;
			shoot_control.fric1_.speed_set = FRIC_DOWN;
			shoot_control.fric2_.speed_set = FRIC_DOWN;
		}
}

__attribute__((used))void trigger_motor_turn_back(void)
{
    if( shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }
    else
    {
        shoot_control.speed_set = -shoot_control.trigger_speed_set;
    }

    if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
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
			  shoot_control.set_angle = rad_format(shoot_control.angle + PI_TEN );
        shoot_control.move_flag = 1;
    }

    //到达角度判断
    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
    {
        //没到达一直设置旋转速度
        shoot_control.trigger_speed_set = TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else
    {
        shoot_control.move_flag = 0;
        shoot_bullet_count++;
				shoot_control.last_calories = shoot_control.calories; 
				shoot_control.calories += shoot_control.damn_calories; 
    }
		shoot_control.shoot_mode = SHOOT_DONE;
}


#endif
