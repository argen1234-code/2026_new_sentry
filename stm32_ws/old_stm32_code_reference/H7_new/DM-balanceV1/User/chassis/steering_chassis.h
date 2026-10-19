#ifndef _STEERING_CHASSIS_H
#define _STEERING_CHASSIS_H

#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "robot_param.h"

#if ROBOT_CHASSIS == Steering_wheel   

// 将 GM6020 的编码值转换为弧度: 2 * PI / 8191
#define GM6020_ECD_to_RAD      7.67084032e-4f
// 电机到舵轮几何中心的距离（米）
#define MOTOR_TO_CENTER        0.27f
// 底盘中心到电机的等效半径（米）
#define MOTOR_DISTANCE_TO_CENTER      0.22f

// 轮子位置枚举
typedef enum
{
    LF = 0,   // 左前轮
    LB,       // 左后轮
    RB,       // 右后轮
    RF,       // 右前轮
} WHEEL_POSITION_e;

extern void chassis_init(chassis_move_t *chassis_move_init);
extern void chassis_set_mode(chassis_move_t *chassis_move_mode);
extern void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
extern void chassis_feedback_update(chassis_move_t *chassis_move_update);
extern void chassis_set_contorl(chassis_move_t *chassis_move_control);
extern void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
extern void chassis_send_cmd(chassis_move_t *chassis_send_cmd);
extern void observer(chassis_move_t *chassis_move, KalmanFilter_t *vxEstimateKF, KalmanFilter_t *vyEstimateKF);

#endif

#endif
