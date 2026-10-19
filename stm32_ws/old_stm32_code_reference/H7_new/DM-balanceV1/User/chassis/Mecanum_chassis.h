#ifndef _MECANUM_CHASSIS_H
#define _MECANUM_CHASSIS_H

#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "robot_param.h"
#if ROBOT_CHASSIS == Mecanum_wheel

#define MOTOR_DISTANCE_TO_CENTER      0.308904f

// 麦轮底盘观测：分开计算 vx / vy
#define MECANUM_WHEEL_NUM 4
#define MECANUM_RADIUS    0.0603f   // 轮半径，同原函数
#define RADIAN            0.01745329252f
extern void chassis_init(chassis_move_t *chassis_move_init);
extern void chassis_set_mode(chassis_move_t *chassis_move_mode);
extern void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
extern void chassis_feedback_update(chassis_move_t *chassis_move_update);
extern void chassis_set_contorl(chassis_move_t *chassis_move_control);
extern void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
extern void chassis_send_cmd(chassis_move_t *chassis_send_cmd);

#endif

#endif
