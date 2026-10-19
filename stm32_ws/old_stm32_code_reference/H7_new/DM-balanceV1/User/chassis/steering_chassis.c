/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       steering_chassis.c
  * @brief      舵轮底盘控制
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
//#include "iwdg.h"
#include "slip_control.h"
#include "robot_param.h"

/********************************************
 * PID 计算里面若有单独加减的变量均为偏移量
 ********************************************/

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

extern chassis_move_t chassis_move;

// 舵轮 6020 初始偏置角设置
static void chassis_wheel_angle_offset_init(void)
{
    chassis_move.wheel_angle_offset.now[0]     = CHASSIS_6020_INIT_ANGLE_0;
    chassis_move.wheel_angle_offset.last[0]    = CHASSIS_6020_INIT_ANGLE_0;
    chassis_move.wheel_angle_offset.initial[0] = CHASSIS_6020_INIT_ANGLE_0;

    chassis_move.wheel_angle_offset.now[1]     = CHASSIS_6020_INIT_ANGLE_1;
    chassis_move.wheel_angle_offset.last[1]    = CHASSIS_6020_INIT_ANGLE_1;
    chassis_move.wheel_angle_offset.initial[1] = CHASSIS_6020_INIT_ANGLE_1;

    chassis_move.wheel_angle_offset.now[2]     = CHASSIS_6020_INIT_ANGLE_2;
    chassis_move.wheel_angle_offset.last[2]    = CHASSIS_6020_INIT_ANGLE_2;
    chassis_move.wheel_angle_offset.initial[2] = CHASSIS_6020_INIT_ANGLE_2;

    chassis_move.wheel_angle_offset.now[3]     = CHASSIS_6020_INIT_ANGLE_3;
    chassis_move.wheel_angle_offset.last[3]    = CHASSIS_6020_INIT_ANGLE_3;
    chassis_move.wheel_angle_offset.initial[3] = CHASSIS_6020_INIT_ANGLE_3;
}

// 二维向量旋转（与 F4 工程保持一致）
static void vector_rotate(fp32 angle, fp32 *vector)
{
    if (vector == NULL)
    {
        return;
    }

    fp32 x_temp = vector[0];
    // 角度规范到 [-PI, PI]
    angle = rad_format(angle);

    fp32 cos_val = arm_cos_f32(angle);
    fp32 sin_val = arm_sin_f32(angle);

    vector[0] = cos_val * x_temp - sin_val * vector[1];
    vector[1] = sin_val * x_temp + cos_val * vector[1];
}

/**
  * @brief          运动学逆解算纵享丝滑控制策略
  * @param[in&out]  wheel_angle     舵轮的目标角度
  * @param[in&out]  wheel_speed     舵轮的目标速度
  * @retval         none
  */
static void smooth_control(fp32 *wheel_angle, fp32 *wheel_speed)
{
	fp32 factor = 0.50f;

	// 定义当前的舵轮角度、舵轮目标角度与当前角度差
	fp32 angle_delta[4], Current_angle[4];
	for(uint8_t i = 0; i < 4; i++)
	{
		// 获取舵轮当前的角度
		Current_angle[i] = chassis_move.chassis_6020[i].angle;

		// 计算舵轮目标角度与当前角度差，并规范化到[-π, π]范围
		angle_delta[i] = rad_format(wheel_angle[i] - Current_angle[i]);

		// 就近转位策略
		if(angle_delta[i] > PI * factor)				//如果角度差大于90度
		{
			wheel_angle[i] -= PI;       					//目标角度减180度
			wheel_speed[i] = -wheel_speed[i];			//速度反向
		}
		else if(angle_delta[i] < -PI * factor)  //如果角度差小于-90度
		{
			wheel_angle[i] += PI;        					//目标角度加180度
			wheel_speed[i] = -wheel_speed[i]; 		//速度反向
		}

		// 确保最终角度在合理范围内
		wheel_angle[i] = rad_format(wheel_angle[i]);
	}
}

/**
  * @brief          运动学逆解算
  * @param[in]      vx_set  相对云台设置的x方向速度分量
  * @param[in]      vy_set  相对云台设置的y方向速度分量  
  * @param[in]      wz_set  底盘旋转速度
  * @param[out]     wheel_angle     舵轮目标角度指针
  * @param[out]     wheel_speed     舵轮目标速度指针
  * @retval         none
  */
void chas_inv_cal(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 *wheel_angle, fp32 *wheel_speed)
{
    if((wheel_angle == NULL) || (wheel_speed == NULL)) return;
	
    // 底盘尺寸参数 (长40，宽35)
    const fp32 half_length = 20.0f;  // 长的一半
    const fp32 half_width = 17.5f;   // 宽的一半
    
    // 计算各轮子到旋转中心的距离和角度
    // 对于矩形底盘，每个轮子的旋转分量需要单独计算
    
    // 计算各轮速度分量并求模
    fp32 vx_total[4], vy_total[4];
    
    // LF (1号电机) - 左前: 位置(-half_length, half_width)
    vx_total[LF] = vx_set - wz_set * half_width;   // -w × ry
    vy_total[LF] = vy_set + wz_set * half_length;  // +w × rx
    arm_sqrt_f32(vx_total[LF] * vx_total[LF] + vy_total[LF] * vy_total[LF], &wheel_speed[LF]);
    
    // LB (2号电机) - 左后: 位置(-half_length, -half_width)
    vx_total[LB] = vx_set + wz_set * half_width;   // -w × (-ry) = +w × ry
    vy_total[LB] = vy_set + wz_set * half_length;  // +w × rx
    arm_sqrt_f32(vx_total[LB] * vx_total[LB] + vy_total[LB] * vy_total[LB], &wheel_speed[LB]);
    
    // RB (3号电机) - 右后: 位置(half_length, -half_width)
    vx_total[RB] = vx_set + wz_set * half_width;   // -w × (-ry) = +w × ry
    vy_total[RB] = vy_set - wz_set * half_length;  // +w × (-rx) = -w × rx
    arm_sqrt_f32(vx_total[RB] * vx_total[RB] + vy_total[RB] * vy_total[RB], &wheel_speed[RB]);
    
    // RF (4号电机) - 右前: 位置(half_length, half_width)
    vx_total[RF] = vx_set - wz_set * half_width;   // -w × ry
    vy_total[RF] = vy_set - wz_set * half_length;  // +w × (-rx) = -w × rx
    arm_sqrt_f32(vx_total[RF] * vx_total[RF] + vy_total[RF] * vy_total[RF], &wheel_speed[RF]);
    
    // 计算各轮角度
    wheel_angle[LF] = atan2(vy_total[LF], vx_total[LF]) - chassis_move.wheel_angle_offset.now[LF];
    wheel_angle[LB] = atan2(vy_total[LB], vx_total[LB]) - chassis_move.wheel_angle_offset.now[LB];
    wheel_angle[RB] = atan2(vy_total[RB], vx_total[RB]) - chassis_move.wheel_angle_offset.now[RB];
    wheel_angle[RF] = atan2(vy_total[RF], vx_total[RF]) - chassis_move.wheel_angle_offset.now[RF];
    
    // 角度规范化
    for(uint8_t i = 0; i < 4; i++)
    {
       wheel_angle[i] = rad_format(wheel_angle[i]);
    }

    smooth_control(wheel_angle, wheel_speed);
}


/**
  * @brief  底盘测量数据更新，包含 3508 电机速度、6020 电机角度、
  *         欧拉角（yaw/pitch/roll）等
  * @param[out] chassis_move_update  "chassis_move" 变量指针
  */
__attribute__((used)) void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        // 3508 速度（m/s），由编码器转速转换
        chassis_move_update->chassis_3508[i].speed =
            chassis_move_update->chassis_3508[i].chassis_motor_measure->speed_rpm / MPS_to_RPM;

        // 3508 加速度（简单用速度 PID D 项乘频率近似）
        chassis_move_update->chassis_3508[i].accel =
            chassis_move_update->chas_3508_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;

        // 6020 当前角度（rad）
        chassis_move_update->chassis_6020[i].angle =
            rad_format(chassis_move_update->chassis_6020[i].chassis_motor_measure->ecd / GM6020_Angle_Ratio);
    }

    // 陀螺仪姿态角（这里直接用 INS 解算出的欧拉角）
    chassis_move_update->chassis_yaw   = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET));
    chassis_move_update->chassis_roll  = rad_format(*(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET));
}

/**
  * @brief  初始化 "chassis_move" 结构体，包含 PID 初始化、指针获取、
  *         6020 初始角度设置等
  * @param[out] chassis_move_init  "chassis_move" 变量指针
  */
__attribute__((used)) void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    // 3508 速度环 PID 参数
    const static fp32 chas_3508_pid_param[3] = {
        M3505_MOTOR_SPEED_PID_KP,
        M3505_MOTOR_SPEED_PID_KI,
        M3505_MOTOR_SPEED_PID_KD
    };
    // 6020 角度环 PID 参数
    const static fp32 chas_6020_angle_pid_param[3] = {
        GM6020_MOTOR_ANGLE_PID_KP,
        GM6020_MOTOR_ANGLE_PID_KI,
        GM6020_MOTOR_ANGLE_PID_KD
    };
    // 6020 速度环 PID 参数
    const static fp32 chas_6020_speed_pid_param[3] = {
        GM6020_MOTOR_SPEED_PID_KP,
        GM6020_MOTOR_SPEED_PID_KI,
        GM6020_MOTOR_SPEED_PID_KD
    };
    // 底盘角度（跟随云台） PID 参数
    const static fp32 chassis_yaw_pid_param[3] = {
        CHASSIS_FOLLOW_GIMBAL_PID_KP,
        CHASSIS_FOLLOW_GIMBAL_PID_KI,
        CHASSIS_FOLLOW_GIMBAL_PID_KD
    };
    // 回正模式 PID 参数
    const static fp32 chassis_yaw_return_pid_param[3] = {
        YAW_RETURN_PID_KP,
        YAW_RETURN_PID_KI,
        YAW_RETURN_PID_KD
    };

    // 初始模式：无控制（不动）
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_NO_MOVE;

    // 获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    // 获取陀螺仪欧拉角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    // 获取云台电机数据指针
    chassis_move_init->chassis_yaw_motor   = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();

    // 获取底盘电机数据指针，并初始化 3508 / 6020 PID
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_move_init->chassis_3508[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->chas_3508_pid[i], PID_POSITION,
                 chas_3508_pid_param,
                 M3505_MOTOR_SPEED_PID_MAX_OUT,
                 M3505_MOTOR_SPEED_PID_MAX_IOUT);

        chassis_move_init->chassis_6020[i].chassis_motor_measure = get_chassis_motor_measure_point(i + 4);
        PID_init(&chassis_move_init->chas_6020_angle_pid[i], PID_POSITION,
                 chas_6020_angle_pid_param,
                 GM6020_MOTOR_ANGLE_PID_MAX_OUT,
                 GM6020_MOTOR_ANGLE_PID_MAX_IOUT);
        PID_init(&chassis_move_init->chas_6020_speed_pid[i], PID_POSITION,
                 chas_6020_speed_pid_param,
                 GM6020_MOTOR_SPEED_PID_MAX_OUT,
                 GM6020_MOTOR_SPEED_PID_MAX_IOUT);
    }

    // 初始化回正模式 PID
    PID_init(&chassis_move_init->chas_return_pid, PID_POSITION,
             chassis_yaw_return_pid_param,
             YAW_RETURN_PID_MAX_OUT,
             YAW_RETURN_PID_MAX_IOUT);

    // 初始化底盘角度 PID（跟随云台）
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION,
             chassis_yaw_pid_param,
             CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT,
             CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);

    // 用一阶滤波代替斜坡函数生成
    const static fp32 chassis_x_order_filter = CHASSIS_ACCEL_X_NUM;
    const static fp32 chassis_y_order_filter = CHASSIS_ACCEL_Y_NUM;
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx,
                            CHASSIS_CONTROL_TIME,
                            &chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy,
                            CHASSIS_CONTROL_TIME,
                            &chassis_y_order_filter);

    // 速度上下限
    chassis_move_init->vx_max_speed =  NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vy_max_speed =  NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    // 底盘舵向初始角度
    chassis_wheel_angle_offset_init();

    // 初始回正标志为 1（允许回正）
    chassis_move_init->chassis_return_flag = 1;

    // 更新一次反馈数据
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief  根据遥控器 / 键盘设置底盘控制模式
  * @param[out] chassis_move_mode  "chassis_move" 变量指针
  */
__attribute__((used)) void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    // 在 chassis_behaviour.c 里实现
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief  底盘模式改变时做过渡处理（如回正、零速等）
  * @param[out] chassis_move_transit "chassis_move" 变量指针
  */
__attribute__((used)) void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    // 切入无力模式：重置相对角度设定
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_MOVE) &&
        chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_MOVE)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    // 切入跟随云台模式：重置相对角度设定
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) &&
             chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    // 切入小陀螺模式：记录当前 yaw 作为参考
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_SPIN) &&
             chassis_move_transit->chassis_mode == CHASSIS_VECTOR_SPIN)
    {
        // 这里需要考虑云台和底盘 yaw 的偏差，直接用底盘 yaw 作为初始参考
        chassis_move_transit->chassis_relative_angle_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief  设置底盘控制设定值（vx_set / vy_set / wz_set），
  *         三个控制量由 chassis_behaviour_control_set 计算得到
  * @param[out] chassis_move_control  "chassis_move" 变量指针
  */
__attribute__((used)) void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }

    fp32 vx_set = 0.0f;
    fp32 vy_set = 0.0f;
    fp32 wz_set = 0.0f;

    // 从行为层获取三个控制设定值
    chassis_behaviour_control_set(&vx_set, &vy_set, &wz_set, chassis_move_control);
    wz_set = -wz_set;

    // 回正到当前云台朝向（CHASSIS_RT_FOLLOW_GIMBAL_YAW_SPIN）
    if (chassis_move_control->chassis_mode == CHASSIS_RT_FOLLOW_GIMBAL_YAW_SPIN)
    {
        // “return_wz_set” 为回正 PID 的输出，限幅后作为角速度设定
        chassis_move_control->wz_set = -chassis_move_control->return_wz_set * 0.00004f;

        // 回正结束后，清除回正标志
        if (fabs(chassis_move_control->wz_set) < 0.001f &&
            chassis_move_control->chassis_return_record == CHASSIS_RT_FOLLOW_GIMBAL_YAW_SPIN)
        {
            chassis_move_control->wz_set = 0.0f;
            chassis_move_control->chassis_return_flag = 0;
        }

        // 将速度向量从“控制坐标”旋转到“底盘坐标”
        fp32 vector[2] = {vx_set, vy_set};
        vector_rotate(chassis_move_control->chassis_relative_angle_set +
                          chassis_move_control->gimbal_radian_of_ecd - 2.0f,
                      vector);

        vx_set = vector[0];
        vy_set = vector[1];

        // 速度限幅
        chassis_move_control->vx_set =
            fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set =
            fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }

    // 跟随云台模式
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        // 底盘绕 yaw 轴旋转角速度由 PID 计算（跟随云台）
        chassis_move_control->wz_set = -PID_Calc(&chassis_move_control->chassis_angle_pid,
                                                 chassis_move_control->chassis_yaw_motor->relative_angle,
                                                 chassis_move_control->chassis_relative_angle_set);

        // 旋转矢量变换到底盘坐标系
        fp32 vector[2] = {vx_set, vy_set};
        vector_rotate(chassis_move_control->gimbal_radian_of_ecd - 2.0f, vector);

        vx_set = vector[0];
        vy_set = vector[1];

        // 速度限幅
        chassis_move_control->vx_set =
            fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set =
            fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    // 小陀螺模式
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_SPIN)
    {
        // 旋转速度直接由 behavior 层给出
        chassis_move_control->wz_set = wz_set;

        if (chassis_move_control->chassis_mode == CHASSIS_RT_FOLLOW_GIMBAL_YAW_SPIN)
        {
            chassis_move_control->wz_set = 0.0f;
        }

        fp32 vector[2] = {vx_set, vy_set};

        // 小陀螺时速度向量的旋转变换
        vector_rotate(chassis_move_control->chassis_relative_angle_set +
                          chassis_move_control->gimbal_radian_of_ecd,
                      vector);

        vx_set = vector[0];
        vy_set = vector[1];

        // 速度限幅
        chassis_move_control->vx_set =
            fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set =
            fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
}

/**
  * @brief  舵轮 PID 计算，抑制舵向电机角度跳变
  * @param[in]  chassis_pid_calc  底盘运动数据结构体指针
  */
static void PID_Calc_Jump_Restrain(chassis_move_t *chassis_pid_calc)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        // 回正模式下，优先计算回正角速度（基于云台 yaw 与目标角度）
        if (chassis_pid_calc->chassis_mode == CHASSIS_RT_FOLLOW_GIMBAL_YAW_SPIN)
        {
            fp32 taget  = rad_format(3.55f);
            fp32 actual = rad_format(chassis_pid_calc->gimbal_radian_of_ecd);

            if (fabs(actual - taget) > PI)
            {
                if (taget < 0.0f)
                {
                    chassis_pid_calc->return_wz_set =
                        PID_Calc(&chassis_pid_calc->chas_return_pid, actual, taget + 2 * PI);
                }
                else
                {
                    chassis_pid_calc->return_wz_set =
                        PID_Calc(&chassis_pid_calc->chas_return_pid, actual, taget - 2 * PI);
                }
            }
            else
            {
                chassis_pid_calc->return_wz_set =
                    PID_Calc(&chassis_pid_calc->chas_return_pid, actual, taget);
            }
        }

        // 判断是否为“几乎静止”状态
        if (fabsf(chassis_pid_calc->vx_set) < 0.1f &&
            fabsf(chassis_pid_calc->vy_set) < 0.1f &&
            fabsf(chassis_pid_calc->wz_set) < 0.1f)
        {
            // 跟随云台模式：在静止时缓慢回到初始角
            if (chassis_pid_calc->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
            {
                int32_t count = 0;

                if (chassis_pid_calc->chassis_return_record != chassis_pid_calc->chassis_mode)
                {
                    count = 2001;
                }

                if (count > 1)
                {
                    count--;
                    chassis_pid_calc->chassis_6020[i].angle_set =
                        rad_format(-chassis_pid_calc->wheel_angle_offset.initial[i] +
                                   1.15f +
                                   chassis_pid_calc->gimbal_radian_of_ecd);
                    chassis_pid_calc->wheel_angle_offset.last[i] =
                        chassis_pid_calc->chassis_6020[i].angle;
                }
                else
                {
                    chassis_pid_calc->chassis_6020[i].angle_set =
                        chassis_pid_calc->wheel_angle_offset.last[i];
                }
            }
            // 无力模式：恢复到上一次记录的角度
            else if (chassis_pid_calc->chassis_mode == CHASSIS_VECTOR_NO_MOVE)
            {
                chassis_pid_calc->chassis_3508[i].speed_set = 0.0f;
                chassis_pid_calc->chassis_6020[i].angle_set =
                    chassis_pid_calc->wheel_angle_offset.last[i];
                chassis_pid_calc->wheel_angle_offset.now[i] =
                    chassis_pid_calc->wheel_angle_offset.initial[i];
            }
            // 小陀螺模式：回到初始角度
            else if (chassis_pid_calc->chassis_mode == CHASSIS_VECTOR_SPIN)
            {
                chassis_pid_calc->wheel_angle_offset.now[i] =
                    chassis_pid_calc->wheel_angle_offset.initial[i];
            }
        }
        else
        {
            // 有控制时记录当前角度
            chassis_pid_calc->wheel_angle_offset.last[i] =
                chassis_pid_calc->chassis_6020[i].angle;
        }

        chassis_pid_calc->chassis_return_record = chassis_pid_calc->chassis_mode;

        // 3508 速度环 PID
        PID_Calc(&chassis_pid_calc->chas_3508_pid[i],
                 chassis_pid_calc->chassis_3508[i].speed,
                 chassis_pid_calc->chassis_3508[i].speed_set);

        // 6020 角度 + 速度双环 PID
        if (fabs(chassis_pid_calc->chassis_6020[i].angle -
                 chassis_pid_calc->chassis_6020[i].angle_set) > PI)
        {
            if (chassis_pid_calc->chassis_6020[i].angle_set < 0.0f)
            {
                PID_Calc(&chassis_pid_calc->chas_6020_angle_pid[i],
                         chassis_pid_calc->chassis_6020[i].angle,
                         chassis_pid_calc->chassis_6020[i].angle_set + 2 * PI);
            }
            else
            {
                PID_Calc(&chassis_pid_calc->chas_6020_angle_pid[i],
                         chassis_pid_calc->chassis_6020[i].angle,
                         chassis_pid_calc->chassis_6020[i].angle_set - 2 * PI);
            }

            PID_Calc(&chassis_pid_calc->chas_6020_speed_pid[i],
                     chassis_pid_calc->chassis_6020[i].chassis_motor_measure->speed_rpm,
                     chassis_pid_calc->chas_6020_angle_pid[i].out);
        }
        else
        {
            PID_Calc(&chassis_pid_calc->chas_6020_angle_pid[i],
                     chassis_pid_calc->chassis_6020[i].angle,
                     chassis_pid_calc->chassis_6020[i].angle_set);
            PID_Calc(&chassis_pid_calc->chas_6020_speed_pid[i],
                     chassis_pid_calc->chassis_6020[i].chassis_motor_measure->speed_rpm,
                     chassis_pid_calc->chas_6020_angle_pid[i].out);
        }

        // 将 PID 输出填入 give_current 方便统一功率控制
        chassis_pid_calc->chassis_3508[i].give_current =
            (int16_t)chassis_pid_calc->chas_3508_pid[i].out;
        chassis_pid_calc->chassis_6020[i].give_current =
            (int16_t)chassis_pid_calc->chas_6020_speed_pid[i].out;
    }
}

/**
  * @brief  控制循环：根据设定速度，计算各轮电机电流
  * @param[out] chassis_move_control_loop  "chassis_move" 变量指针
  */
__attribute__((used)) void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 wheel_speed[4] = {0.0f};
    fp32 wheel_angle[4] = {0.0f};

    // 舵轮运动学逆解：得到每个轮子的目标角度和速度
    chas_inv_cal(chassis_move_control_loop->vx_set,
                 chassis_move_control_loop->vy_set,
                 chassis_move_control_loop->wz_set,
                 wheel_angle, wheel_speed);

    // 设置 6020 角度目标和 3508 速度目标
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_move_control_loop->chassis_6020[i].angle_set = wheel_angle[i];
        chassis_move_control_loop->chassis_3508[i].speed_set = wheel_speed[i];
    }

    // 如需防滑，可在此调用 slip_control（当前注释掉）
    // slip_control(chassis_move_control_loop);

    // 在抑制角度跳变的前提下进行 PID 计算
    PID_Calc_Jump_Restrain(chassis_move_control_loop);

    // 底盘功率控制（会根据功率限制缩减 3508 / 6020 的 give_current）
    chassis_power_control(chassis_move_control_loop);
}

__attribute__((used)) void chassis_send_cmd(chassis_move_t *chassis_send_cmd)
{
    (void)chassis_send_cmd;

    if (toe_is_error(DBUS_TOE))
    {
        // 遥控断开，所有底盘电机电流清零
        CAN_cmd_chassis(0, 0, 0, 0);
        CAN_cmd_chassis_6020(0, 0, 0, 0);
    }
    else
    {
        // 发送 3508 行走电机电流
        CAN_cmd_chassis(chassis_move.chassis_3508[0].give_current,
                        chassis_move.chassis_3508[1].give_current,
                        chassis_move.chassis_3508[2].give_current,
                        chassis_move.chassis_3508[3].give_current);

        // 发送 6020 舵向电机电流
        CAN_cmd_chassis_6020(chassis_move.chassis_6020[0].give_current,
                             chassis_move.chassis_6020[1].give_current,
                             chassis_move.chassis_6020[2].give_current,
                             chassis_move.chassis_6020[3].give_current);
    }
}

// 舵轮底盘速度观测占位实现（如需要可参考 Omni/Mecanum 的 observer 进一步完善）
__attribute__((used)) void observer(chassis_move_t *chassis_move,
                                    KalmanFilter_t *vxEstimateKF,
                                    KalmanFilter_t *vyEstimateKF)
{
    (void)chassis_move;
    (void)vxEstimateKF;
    (void)vyEstimateKF;
}

#endif  // ROBOT_CHASSIS == Steering_wheel
