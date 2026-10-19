/**
 ******************************************************************************
 * @file    gimbal_double_yaw_pitch.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   double-yaw-pitch 云台构型接口。
 *
 * @note    本构型复用直驱 yaw/pitch 核心承接小 yaw 和 pitch，额外生成大 yaw
 *          MIT 执行器命令。
 * @note    本文件不依赖任务、遥控、INS、CAN 或电机全局状态；所有运行时数据由
 *          GimbalInput 注入。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_MECHANISM_GIMBAL_DOUBLE_YAW_PITCH_H
#define PRINTK_MECHANISM_GIMBAL_DOUBLE_YAW_PITCH_H

#include "gimbal_direct_yaw_pitch.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */

/**
 * @brief double 构型大 yaw 状态。
 */
typedef struct
{
    GimbalDirectPid abs_pid; /**< 大 yaw 绝对角速度外环。 */
    GimbalMitFb fb;          /**< 大 yaw 最近一次反馈。 */
    GimbalMitCmd cmd;        /**< 大 yaw 当前命令。 */
    float32_t absolute_set;  /**< 大 yaw 绝对角目标，单位 rad。 */
    uint8_t feedback_ready;  /**< 非 0 表示已同步过大 yaw 目标。 */
} GimbalDoubleBigYaw;

/**
 * @brief double-yaw-pitch 私有上下文。
 */
typedef struct
{
    GimbalDirectCtx core;       /**< 小 yaw + pitch 直驱核心。 */
    GimbalDoubleBigYaw big_yaw; /**< 大 yaw MIT 执行器状态。 */
} GimbalDoubleCtx;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化 double-yaw-pitch 云台。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalDoubleCtx 私有上下文。
 * @return 执行状态。
 */
GimbalStatus gimbal_double_init(Gimbal *gimbal, void *impl);

/**
 * @brief 执行一次 double-yaw-pitch 控制周期。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalDoubleCtx 私有上下文。
 * @param input 周期输入。
 * @param output 周期输出。
 * @return 执行状态。
 */
GimbalStatus gimbal_double_step(Gimbal *gimbal,
                                void *impl,
                                const GimbalInput *input,
                                GimbalOutput *output);

/**
 * @brief 更新 double-yaw-pitch 反馈。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalDoubleCtx 私有上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
GimbalStatus gimbal_double_update(Gimbal *gimbal, void *impl, const GimbalInput *input);

/**
 * @brief 执行 double-yaw-pitch 控制计算。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalDoubleCtx 私有上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
GimbalStatus gimbal_double_control(Gimbal *gimbal, void *impl, const GimbalInput *input);

/**
 * @brief 生成 double-yaw-pitch 输出。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalDoubleCtx 私有上下文。
 * @param output 周期输出。
 * @return 执行状态。
 */
GimbalStatus gimbal_double_output(const Gimbal *gimbal, void *impl, GimbalOutput *output);

/**
 * @brief 同步 double-yaw-pitch 控制目标到当前反馈。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalDoubleCtx 私有上下文。
 * @return 执行状态。
 */
GimbalStatus gimbal_double_sync(Gimbal *gimbal, void *impl);

/**
 * @brief 写入 double-yaw-pitch 校准结果。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalDoubleCtx 私有上下文。
 * @param cali 校准结果。
 * @return 执行状态。
 */
GimbalStatus gimbal_double_set_cali(Gimbal *gimbal, void *impl, const GimbalCali *cali);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_MECHANISM_GIMBAL_DOUBLE_YAW_PITCH_H
