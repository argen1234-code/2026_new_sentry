/**
  * @file       chassis_behaviour.h
  * @brief      行为层桩：把主机速度指令直接映射为底盘模式与速度给定
  * @note       本工程只做 no-follow 速度直驱，不接遥控器与云台。
  */
#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H

#include "chassis_task.h"
#include "struct_typedef.h"

typedef enum
{
    CHASSIS_NO_MOVE,
    CHASSIS_FOLLOW_GIMBAL_YAW,
    CHASSIS_YAW_HOLD,
    CHASSIS_SPIN,
    CHASSIS_RETURN,
} chassis_behaviour_e;

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set,
                                   fp32 *wz_set,
                                   chassis_move_t *chassis_move_rc_to_vector);

#endif
