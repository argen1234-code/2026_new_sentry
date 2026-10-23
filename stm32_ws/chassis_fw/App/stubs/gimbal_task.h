/**
  * @file       gimbal_task.h
  * @brief      最小云台类型桩（本工程不含云台）
  * @note       仅保留底盘代码引用到的字段；getter 返回 NULL。
  */
#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "struct_typedef.h"

typedef struct
{
    float relative_angle;
    float relative_angle_set;
    float relative_angle_last;
    float gyro;
    float min_relative_angle;
    float max_relative_angle;
    uint8_t angle_offset_init;
} gimbal_motor_t;

const gimbal_motor_t *get_yaw_motor_point(void);
const gimbal_motor_t *get_pitch_motor_point(void);

#endif
