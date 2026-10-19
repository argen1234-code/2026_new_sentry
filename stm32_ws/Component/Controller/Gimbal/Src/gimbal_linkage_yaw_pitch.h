/**
 ******************************************************************************
 * @file    gimbal_linkage_yaw_pitch.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   linkage-yaw-pitch 云台构型接口。
 *
 * @note    本构型在直驱 yaw/pitch 核心前追加 yaw 到 pitch 的联动输入变换。
 * @note    本文件不依赖任务、遥控、INS、CAN 或电机全局状态；所有运行时数据由
 *          GimbalInput 注入。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_MECHANISM_GIMBAL_LINKAGE_YAW_PITCH_H
#define PRINTK_MECHANISM_GIMBAL_LINKAGE_YAW_PITCH_H

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
 * @brief linkage-yaw-pitch 私有上下文。
 */
typedef struct
{
    GimbalDirectCtx core; /**< 直驱 yaw/pitch 核心。 */
    float32_t pitch_gain; /**< yaw 输入到 pitch 输入的联动系数。 */
} GimbalLinkageCtx;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化 linkage-yaw-pitch 云台。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalLinkageCtx 私有上下文。
 * @return 执行状态。
 */
GimbalStatus gimbal_linkage_init(Gimbal *gimbal, void *impl);

/**
 * @brief 执行一次 linkage-yaw-pitch 控制周期。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalLinkageCtx 私有上下文。
 * @param input 周期输入。
 * @param output 周期输出。
 * @return 执行状态。
 */
GimbalStatus gimbal_linkage_step(Gimbal *gimbal,
                                 void *impl,
                                 const GimbalInput *input,
                                 GimbalOutput *output);

/**
 * @brief 更新 linkage-yaw-pitch 反馈。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalLinkageCtx 私有上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
GimbalStatus gimbal_linkage_update(Gimbal *gimbal, void *impl, const GimbalInput *input);

/**
 * @brief 执行 linkage-yaw-pitch 控制计算。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalLinkageCtx 私有上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
GimbalStatus gimbal_linkage_control(Gimbal *gimbal, void *impl, const GimbalInput *input);

/**
 * @brief 生成 linkage-yaw-pitch 输出。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalLinkageCtx 私有上下文。
 * @param output 周期输出。
 * @return 执行状态。
 */
GimbalStatus gimbal_linkage_output(const Gimbal *gimbal, void *impl, GimbalOutput *output);

/**
 * @brief 同步 linkage-yaw-pitch 控制目标到当前反馈。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalLinkageCtx 私有上下文。
 * @return 执行状态。
 */
GimbalStatus gimbal_linkage_sync(Gimbal *gimbal, void *impl);

/**
 * @brief 写入 linkage-yaw-pitch 校准结果。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalLinkageCtx 私有上下文。
 * @param cali 校准结果。
 * @return 执行状态。
 */
GimbalStatus gimbal_linkage_set_cali(Gimbal *gimbal, void *impl, const GimbalCali *cali);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_MECHANISM_GIMBAL_LINKAGE_YAW_PITCH_H
