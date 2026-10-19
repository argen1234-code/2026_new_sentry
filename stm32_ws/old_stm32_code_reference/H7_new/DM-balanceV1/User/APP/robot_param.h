/**
  ****************************(C) COPYRIGHT 2025 PRINTK****************************
  * @file       robot_param.h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     3-26-2025       yibu             1. start and done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 PRINTK****************************
  */
#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H
#include "struct_typedef.h"

/*是否在线*/
#define OFFLINE 0X00
#define ONLINE 	0X01

/* 超级电容开关 */
#define Cap_off 0X00
#define Cap_on  0X01

/*机器人类型*/
#define Hero_robot      0x01
#define Infantry_robot  0x02
#define Balance_robot   0x03
#define Sentinel_robot  0x04
#define Engineer_robot  0x05

/* 底盘类型 */
#define Mecanum_wheel   0X00
#define Omni_wheel      0X01
#define Steering_wheel  0X02
#define Balance_wheel   0X03

/* 云台类型 */
#define yaw_pitch_direct   			0X00
#define yaw_pitch_linkage  			0X01
#define double_yaw_pitch   			0X02
#define multi_axis_robotic_arm 	0X03

/* 发射机构类型 */
#define friction_3508   0X00
#define friction_double 0X01
#define friction_none   0X02

/* 开发板放置位置 */
#define chassis_board 0x00
#define gimbal_board  0x01

/* 电机类型 闲置*/
#define DJI_3508 0X00
#define DJI_6020 0X01
#define MIT   	 0X02
#define RD _motor 0X03

/* 代码类型 调试或者发布*/
#define debug   0
#define release 1

/* ================= 机器人整体配置 ================= */

#define ROBOT_TYPE       	Sentinel_robot
#define ROBOT_MODE        debug
#define ROBOT_CHASSIS     Steering_wheel
#define ROBOT_GIMBAL      double_yaw_pitch
#define ROBOT_FRICTION    friction_3508
#define ROBOT_BOARD      	gimbal_board
#define ROBOT_CAP         Cap_off

//把底盘、云台、发射机构等通用参数藏到各个兵种的私有头文件里去
#if ROBOT_TYPE == Hero_robot 
#include "Hero.h"
#endif
#if ROBOT_TYPE == Infantry_robot
#include "Infantry_robot.h"
#endif
#if ROBOT_TYPE == Balance_robot 
#include "Balance.h"
#endif
#if ROBOT_TYPE == Sentinel_robot
#include "Sentinel.h"
#endif
#if ROBOT_TYPE == Engineer_robot
#include "Engineer.h"
#endif

/**
  * @brief          remote control dealline solve,because the value of rocker is not zero in middle place,
  * @param          input:the raw channel value 
  * @param          output: the processed channel value
  * @param          deadline
  */
/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
  * @param          输入的遥控器值
  * @param          输出的死区处理后遥控器值
  * @param          死区值
  */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/* ================= 外部声明 ================= */
//小陀螺转速，在底盘任务中会进行修改
		
#define SPIN_FACTOR  10
#define SPIN_SPEED 0.7f

#endif

