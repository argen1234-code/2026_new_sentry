/**
  * @file       chassis_calculate.c
  * @brief      底盘运动学与限幅计算
  * @note       提供速度正解、逆解、坐标旋转和横向加速度限幅。
  */
#include "bsp_usart.h"
#include "chassis_calculate.h"
#include "chassis_task.h"
#include "bsp_fdcan.h"
#include "robot_param.h"
#include "user_lib.h"
#include <math.h>
#include "stdlib.h"

/**
  * @brief          初始化舵轮角度零偏
  * @retval         none
  */
void chassis_wheel_angle_offset_init(void)
{
	for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		chassis_move.wheel_angle_offset.now[i] = 0.0f;
		chassis_move.wheel_angle_offset.last[i] = 0.0f;
		chassis_move.wheel_angle_offset.initial[i] = 0.0f;
	}
}

/**
  * @brief          旋转二维向量
  * @retval         none
  */
void vector_rotate(fp32 angle, fp32 *vector)
{
	if(vector == NULL) return;

	fp32 x_temp = vector[0];
	angle = rad_format(angle);

	fp32 cos_value = cosf(angle);
	fp32 sin_value = sinf(angle);

	vector[0] = cos_value * x_temp - sin_value * vector[1];
	vector[1] = sin_value * x_temp + cos_value * vector[1];
}

/**
  * @brief          限制底盘四轮速度比例
  * @retval         none
  */
static void limit_chassis_wheel_speed(fp32 *wheel_speed)
{
	fp32 max_speed = 0.0f;

	for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		fp32 speed_abs = fabsf(wheel_speed[i]);
		if(speed_abs > max_speed)
		{
			max_speed = speed_abs;
		}
	}

	if(max_speed > MAX_WHEEL_SPEED)
	{
		fp32 scale = MAX_WHEEL_SPEED / max_speed;
		for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
		{
			wheel_speed[i] *= scale;
		}
	}
}

fp32 offset = 0.0f;

/**
  * @brief          执行底盘打滑补偿
  * @retval         none
  */
void slip_control(chassis_move_t *chassis_move)
{
	(void)chassis_move;
}

/**
  * @brief          按横向加速度限制底盘角速度
  * @retval         限幅后的角速度
  */
fp32 chassis_limit_wz_by_lateral_accel(fp32 vx, fp32 vy, fp32 wz)
{
	fp32 translational_speed = sqrtf(vx * vx + vy * vy);
	fp32 wz_limit;

	if(translational_speed < CHASSIS_LAT_SPEED_EPS)
	{
		return wz;
	}

	wz_limit = CHASSIS_LAT_ACCEL_LIMIT / translational_speed;
	return fp32_constrain(wz, -wz_limit, wz_limit);
}

/**
  * @brief          执行底盘逆运动学解算
  * @retval         none
  */
void chas_inv_cal(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 *wheel_angle, fp32 *wheel_speed)
{
	if((wheel_angle == NULL) || (wheel_speed == NULL)) return;

	/* 输入按主机（ROS REP-103）坐标系：+x 向前、+y 向左、wz 逆时针为正。
	 *
	 * 2026-10-23 实车标定：相对上游“x 向右、y 向前”的内部系，本车需要把
	 * 平移量旋转 -90°（顺时针），即 R(-90°) = -R(+90°)，落地就是平移项整体
	 * 取负：205/207（前后轮组）吃 -vx，206/208（左右轮组）吃 +vy。
	 * 先按 +90° 改过一次，实车偏差从 90° 变成 180°（沿规划路径倒着走），
	 * 才确定方向取反。
	 *
	 * 2026-10-23 第二处实测：用 MID360 陀螺仪（与 LIO 无关的独立信源）验证
	 * 自转方向 —— 下发 +1.0 rad/s（REP-103 俯视逆时针）时 gyro.z 实测
	 * -0.14 rad/s，车体实际顺时针转。轮端分布（+,-,-,+）本身对，但整体
	 * 符号反了，故 wz 项取负；平移项不受影响。chas_for_cal 同步取负。 */
	const fp32 wz_speed = -wz_set * CHASSIS_OMNI_ROTATE_RADIUS;

	wheel_speed[WHEEL_REAR_205]  = (-vx_set + wz_speed) * CHASSIS_WHEEL_205_DIRECTION;
	wheel_speed[WHEEL_RIGHT_206] = ( vy_set - wz_speed) * CHASSIS_WHEEL_206_DIRECTION;
	wheel_speed[WHEEL_FRONT_207] = (-vx_set - wz_speed) * CHASSIS_WHEEL_207_DIRECTION;
	wheel_speed[WHEEL_LEFT_208]  = ( vy_set + wz_speed) * CHASSIS_WHEEL_208_DIRECTION;

	for(uint8_t i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		wheel_angle[i] = 0.0f;
	}

	limit_chassis_wheel_speed(wheel_speed);
}

/**
  * @brief          执行底盘正运动学解算
  * @retval         none
  */
void chas_for_cal(fp32 *wheel_angle, fp32 *wheel_speed, fp32 *vx, fp32 *vy, fp32 *wz)
{
	fp32 rear_speed;
	fp32 right_speed;
	fp32 front_speed;
	fp32 left_speed;

	(void)wheel_angle;

	if((wheel_speed == NULL) || (vx == NULL) || (vy == NULL) || (wz == NULL)) return;

	rear_speed  = wheel_speed[WHEEL_REAR_205]  * CHASSIS_WHEEL_205_DIRECTION;
	right_speed = wheel_speed[WHEEL_RIGHT_206] * CHASSIS_WHEEL_206_DIRECTION;
	front_speed = wheel_speed[WHEEL_FRONT_207] * CHASSIS_WHEEL_207_DIRECTION;
	left_speed  = wheel_speed[WHEEL_LEFT_208]  * CHASSIS_WHEEL_208_DIRECTION;

	/* 与 chas_inv_cal 使用同一套符号：前后轮组 -> -vx，左右轮组 -> +vy，
	 * 这样 0x81 回传的速度与 /cmd_vel 同号。wz 同逆解一样整体取负。 */
	*vx = -0.5f * (rear_speed + front_speed);
	*vy =  0.5f * (right_speed + left_speed);

	if(fabsf(CHASSIS_OMNI_ROTATE_RADIUS) > 0.0001f)
	{
		*wz = -((rear_speed - front_speed) + (left_speed - right_speed)) /
		      (4.0f * CHASSIS_OMNI_ROTATE_RADIUS);
	}
	else
	{
		*wz = 0.0f;
	}
}
