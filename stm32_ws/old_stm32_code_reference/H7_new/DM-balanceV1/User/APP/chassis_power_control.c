/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c
  * @brief      ??????????
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
/**
  * @file       chassis_power_control.c
  * @brief      底盘功率控制实现文件
  * @details    实现基于功率预测的底盘电机电流限制逻辑，通过计算电机功率消耗、
  *             对比额定功率阈值，动态调整电机输出电流，防止功率超限
  * @author     用户自定义
  * @date       2026-1-9
  */
#include "arm_math.h"
//#include "capacitor_task.h"
#include "chassis_power_control.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "referee.h"
//#include "voltage_task.h"
#include <stdbool.h>

// 外部变量声明（来自其他任务模块）
extern RC_ctrl_t rc_ctrl;                // 遥控器/键鼠控制输入
//extern CAP_INFO_t cap_info;              // 电容信息结构体
extern robot_status_t robot_status;      // 机器人状态（裁判系统数据）
extern power_heat_data_t buffer_energy;  // 缓冲能量数据（裁判系统）
extern chassis_move_t chassis_move;      // 底盘运动控制结构体（含电机PID、电机测量数据）

// 全局功率限制控制结构体初始化
PowerLimit_t PowerLimit = {
    .k_1 = 1.453e-07f,          // 转速平方项系数（功率计算模型）
    .k_2 = 1.23e-07f,           // 电流平方项系数（功率计算模型）
    .a = 4.081f,                // 功率模型基础偏移量
    .K_Reduction = 1.0f,        // 功率衰减系数（初始为1，无衰减）
    .set_power = SET_POWER_VALUE,  // 额定功率设定值（来自头文件宏定义）
    .P_origin = 0.0f,           // 原始功率计算值（未衰减前）
    .P_bus = 0.0f               // 总线实际功率（衰减后）
};

/* 原始功率计算公式说明：
 * P_origin = a + ω*b*I_cmd + k1*ω² + k2*b²*I_cmd² 
 * 其中：b为衰减系数（初始值1），ω为电机转速，I_cmd为目标电流
 */

/**
 * @brief  电流约束关系计算函数
 * @param  PowerLimit_Cur 功率限制结构体指针
 * @note   从底盘运动结构体中读取3508/6020电机的实时状态数据，
 *         填充到功率限制结构体中，为后续功率计算提供基础数据
 */
void Current_RestraintRelation_Calc(PowerLimit_t *PowerLimit_Cur) {
    // 遍历4个电机通道（底盘通常为4个3508电机）
    for(int i = 0; i < 4; i++) {
        // 获取3508电机实时数据
        // 3508电机当前实际转速（rpm），来自电机反馈测量值
        PowerLimit_Cur->now_motorspeed[i] = chassis_move.chassis_3508[i].chassis_motor_measure->speed_rpm;
        // 3508电机目标电流（A），来自底盘运动解算的PID输出
        PowerLimit_Cur->set_motorcurrent[i] = chassis_move.chas_3508_pid[i].out;
        
        // 获取6020电机实时数据（修正字段名与结构体匹配）
        // 6020电机当前实际转速（rpm）
        PowerLimit_Cur->cur_6020_speed[i] = chassis_move.chassis_6020[i].chassis_motor_measure->speed_rpm;
        // 6020电机目标电流（A），来自速度环PID输出
        PowerLimit_Cur->set_6020_current[i] = chassis_move.chas_6020_speed_pid[i].out;
        // 6020电机实际发送的电流值（A）
        PowerLimit_Cur->now_6020_current[i] = chassis_move.chassis_6020[i].give_current;
    }
}

/**
 * @brief  功率预测与电流衰减控制函数
 * @param  PowerLimit_Pre 功率限制结构体指针
 * @param  chassis_move   底盘运动控制结构体指针
 * @note   核心功率控制逻辑：
 *         1. 计算3508电机的理论功率消耗
 *         2. 对比额定功率，若超限则求解衰减系数b
 *         3. 应用衰减系数到3508电机电流，限制总功率
 *         4. 6020电机不做衰减，保留预留功率
 */
void Predict_Power(PowerLimit_t *PowerLimit_Pre, chassis_move_t *chassis_move) {
    // 6020电机预留功率（W），保证云台/其他关节电机基础功耗
    const float reserved_power_for_6020 = 16.0f;
    // 3508底盘电机可用功率 = 总额定功率 - 6020预留功率
    const float available_power = PowerLimit_Pre->set_power - reserved_power_for_6020;

    // 初始化功率计算中间变量
    float sum_omega_I = 0.0f;   // ω*I_cmd 项累加和
    float sum_omega2 = 0.0f;    // ω² 项累加和
    float sum_I2 = 0.0f;        // I_cmd² 项累加和
    // 扭矩系数/单位转换系数（rpm转rad/s + 电流转扭矩的系数）
    float k_t = 0.01562 * 0.001220703125;

    // 遍历4个3508电机通道，累加功率计算项
    for(int i = 0; i < 4; i++) {
        float omega = PowerLimit_Pre->now_motorspeed[i];    // 3508电机当前转速（rpm）
        float I_cmd = PowerLimit_Pre->set_motorcurrent[i];  // 3508电机目标电流（A）
        
        sum_omega_I += omega * I_cmd * k_t / 9.55f;  // ω*b*I_cmd 项（b初始为1），9.55为rpm转rad/s系数
        sum_omega2 += omega * omega;                 // 转速平方项累加
        sum_I2 += I_cmd * I_cmd;                     // 电流平方项累加
    }

    // 计算原始功率（未衰减前的理论功率消耗）
    PowerLimit.P_origin = PowerLimit_Pre->a + sum_omega_I + PowerLimit_Pre->k_1 * sum_omega2 + PowerLimit_Pre->k_2 * sum_I2;

    // 构造一元二次方程求解衰减系数b（当原始功率超过可用功率时）
    if(PowerLimit.P_origin > available_power) {
        // 一元二次方程：A*b² + B*b + C = 0
        float A = PowerLimit_Pre->k_2 * sum_I2;
        float B = sum_omega_I;
        float C = PowerLimit_Pre->a + PowerLimit_Pre->k_1 * sum_omega2 - available_power;
        float discriminant = B * B - 4 * A * C;  // 判别式

        // 判别式非负时，有实数解
        if(discriminant >= 0) {
            float sqrt_disc = sqrtf(discriminant);  // 判别式开方
            float b1 = (-B + sqrt_disc) / (2 * A);  // 第一个解
            float b2 = (-B - sqrt_disc) / (2 * A);  // 第二个解
            
            // 选择0~1之间的衰减系数（保证电流只衰减不放大）
            PowerLimit_Pre->K_Reduction = (b1 > 0 && b1 <= 1) ? b1 : b2;
            // 限幅衰减系数，防止超出0.01~1范围
            PowerLimit_Pre->K_Reduction = LIMIT_MAX_MIN(PowerLimit_Pre->K_Reduction, 1.0f, 0.01f);
        } else {
            // 判别式为负（无实数解），默认衰减到50%
            PowerLimit_Pre->K_Reduction = 0.5f;
        }
    } else {
        // 原始功率未超限，衰减系数设为1（无衰减）
        PowerLimit_Pre->K_Reduction = 1.0f;
    }

    // 应用衰减系数到电机电流，更新最终发送给电机的电流值
    for(int i = 0; i < 4; i++) {
        // 3508电机电流应用衰减系数，转换为整型发送值
        chassis_move->chassis_3508[i].give_current = (int16_t)(PowerLimit_Pre->set_motorcurrent[i] * PowerLimit_Pre->K_Reduction);
        // 6020电机不做衰减，直接使用PID输出值
        chassis_move->chassis_6020[i].give_current = (int16_t)(chassis_move->chas_6020_speed_pid[i].out);
        
        // 计算衰减后的总线实际功率（3508电机）
        PowerLimit.P_bus = PowerLimit_Pre->a
                          + sum_omega_I * PowerLimit_Pre->K_Reduction
                          + PowerLimit_Pre->k_1 * sum_omega2
                          + PowerLimit_Pre->k_2 * sum_I2 * PowerLimit_Pre->K_Reduction * PowerLimit_Pre->K_Reduction;
    }
}

/**
 * @brief  底盘功率控制主函数
 * @param  chassis_move 底盘运动控制结构体指针
 * @note   底盘功率控制入口函数，供底盘任务主循环调用：
 *         1. 读取电机实时数据
 *         2. 计算功率并动态调整电机电流
 *         3. 可扩展对接裁判系统实时功率限制（当前注释掉）
 */
void chassis_power_control(chassis_move_t *chassis_move) {
    // 以下为扩展接口：从裁判系统获取实时功率限制（当前注释，使用固定额定功率）
    // fp32 current_max_power;    // 裁判系统当前限制最大功率
    // fp32 unused_buffer;        // 缓冲区剩余能量
    // get_chassis_power_and_buffer(&current_max_power, &unused_buffer);  // 获取裁判系统功率限制
    // PowerLimit.set_power = current_max_power;  // 动态更新功率限制值
    
    // 步骤1：读取电机实时状态数据
    Current_RestraintRelation_Calc(&PowerLimit);
    // 步骤2：功率预测与电流衰减控制
    Predict_Power(&PowerLimit, chassis_move);
}
