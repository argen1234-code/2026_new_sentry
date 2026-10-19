/**
    ****************************(C) COPYRIGHT 2019 DJI****************************
    * @file       chassis_power_control.h
    * @brief      底盘功率控制相关头文件
    * @note       该文件定义了底盘功率控制所需的宏、结构体和函数声明，
 
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
	
	
#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H

// 引入底盘任务相关头文件（包含底盘运动控制的结构体/函数声明）
#include "chassis_task.h"  
// 包含机器人参数头文件（定义机器人硬件参数、配置等）
#include "robot_param.h"   

/**
 * @brief 数值限幅宏
 * @param val  需要被限制的数值
 * @param max  最大值上限
 * @param min  最小值下限
 * @return     限制在[min, max]范围内的数值
 * @note       用于防止数值超出合理范围，保护硬件和控制逻辑
 */
#ifndef LIMIT_MAX_MIN
#define LIMIT_MAX_MIN(val, max, min) ((val) > (max) ? (max) : ((val) < (min) ? (min) : (val)))
#endif


/**
 * @brief 根据机器人类型配置额定功率值
 * @note  不同类型机器人的供电功率上限不同，需匹配官方规范
 */
#ifdef Hero_robot
    #define SET_POWER_VALUE 100.0f  // 英雄机器人额定功率 (W)
#elif defined(Infantry_robot)
    #define SET_POWER_VALUE 75.0f   // 步兵机器人额定功率 (W)
#elif defined(Engineer_robot)
    #define SET_POWER_VALUE 100.0f  // 工程机器人额定功率 (W)
#elif defined(Sentinel_robot)
    #define SET_POWER_VALUE 75.0f   // 哨兵机器人额定功率 (W)
#elif defined(Balance_robot)
    #define SET_POWER_VALUE 75.0f   // 平衡机器人额定功率 (W)
#else
    #define SET_POWER_VALUE 75.0f   // 默认额定功率 (W)
#endif

/**
 * @brief 功率限制控制结构体
 * @note  存储底盘功率控制所需的参数、状态和控制量，涵盖3508和6020电机
 */
typedef struct {
    float k_1;                      // 速度平方项系数（功率计算用）
    float k_2;                      // 电流平方项系数（功率计算用）
    float a;                        // 功率补偿系数/偏移量
    float K_Reduction;              // 功率衰减系数（功率超限后降低输出的比例）
    float cur_motorspeed_set[4];    // 当前3508电机速度设定值 (rpm)
    float now_motorspeed[4];        // 当前3508电机实际速度 (rpm)
    float set_motorcurrent[4];      // 3508电机电流设定值 (A)
    float now_motorcurrent[4];      // 3508电机当前实际电流 (A)
    float set_power;                // 底盘总功率设定值 (W)
    // 6020电机相关字段（云台/其他关节电机）
    float set_6020_current[4];      // 6020电机电流设定值 (A，速度环输出/力矩控制)
    float now_6020_current[4];      // 6020电机实际反馈电流 (A)
    float cur_6020_speed[4];        // 6020电机当前速度 (rpm)
    float P_origin;                 // 原始功率计算值（未限幅前的功率）
    float P_bus;                    // 总线实际功率反馈值（W）
} PowerLimit_t;

// 外部可见的全局功率限制控制结构体实例（供其他文件调用）
extern PowerLimit_t PowerLimit;

/**
 * @brief 电流约束关系计算函数
 * @param PowerLimit_Cur 功率限制结构体指针
 * @note  根据功率限制目标，计算电机电流的约束值，防止总功率超限
 */
void Current_RestraintRelation_Calc(PowerLimit_t *PowerLimit_Cur);

/**
 * @brief 功率预测函数
 * @param PowerLimit_Pre 功率限制结构体指针
 * @param chassis_move   底盘运动控制结构体指针
 * @note  根据底盘运动指令和电机状态，预测即将消耗的功率，提前进行限制
 */
void Predict_Power(PowerLimit_t *PowerLimit_Pre, chassis_move_t *chassis_move);

/**
 * @brief 底盘功率控制主函数
 * @param chassis_move 底盘运动控制结构体指针
 * @note  整合功率预测、电流约束等逻辑，实现底盘功率的闭环控制
 */
void chassis_power_control(chassis_move_t *chassis_move);

#endif /* CHASSIS_POWER_CONTROL_H */ 
