/**
  * @file       detect_task.h
  * @brief      设备在线检测的轻量实现
  * @note       保留上游 TOE 编号以便直接复用底盘代码；
  *             detect_hook() 记录最后反馈时刻，toe_is_error() 按超时判定。
  */
#ifndef DETECT_TASK_H
#define DETECT_TASK_H

#include "struct_typedef.h"

enum errorList
{
    DBUS_TOE = 0,
    CHASSIS_MOTOR1_TOE,
    CHASSIS_MOTOR2_TOE,
    CHASSIS_MOTOR3_TOE,
    CHASSIS_MOTOR4_TOE,
    CHASSIS_MOTOR5_TOE,
    CHASSIS_MOTOR6_TOE,
    CHASSIS_MOTOR7_TOE,
    CHASSIS_MOTOR8_TOE,
    PLUCK_MOTOR_TOE,
    FRIC1_MOTOR_TOE,
    FRIC2_MOTOR_TOE,
    FRIC3_MOTOR_TOE,
    YAW_GIMBAL_MOTOR_TOE,
    ROLL_GIMBAL_MOTOR_TOE,
    PITCH_GIMBAL_MOTOR_TOE,
    BOARD_GYRO_TOE,
    BOARD_ACCEL_TOE,
    BOARD_MAG_TOE,
    REFEREE_TOE,
    RM_IMU_TOE,
    OLED_TOE,

    ERROR_LIST_LENGHT,
};

bool_t toe_is_error(uint8_t err);
void detect_hook(uint8_t toe);

#endif
