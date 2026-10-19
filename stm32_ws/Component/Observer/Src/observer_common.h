/**
 ******************************************************************************
 * @file    observer_common.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   观测器内部公共工具接口。
 *
 * @note    本文件只给 Component/Observer 内部实现使用，不访问外部硬件和任务。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_OBSERVER_COMMON_H
#define PRINTK_COMPONENT_OBSERVER_COMMON_H

#include "observer.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 判断浮点数是否为有限值。
 *
 * @param value 输入值。
 * @return 有限返回 1，否则返回 0。
 */
uint8_t observer_finite(float32_t value);

/**
 * @brief 判断三维向量是否全部为有限值。
 *
 * @param value 输入向量。
 * @return 有限返回 1，否则返回 0。
 */
uint8_t observer_vec_ok(const Vec3 *value);

/**
 * @brief 裁剪浮点数到闭区间。
 *
 * @param value 输入值。
 * @param min_v 下限。
 * @param max_v 上限。
 * @return 裁剪后的值。
 */
float32_t observer_clamp(float32_t value, float32_t min_v, float32_t max_v);

/**
 * @brief 对小幅值做死区处理。
 *
 * @param value 输入值。
 * @param dead 死区半宽。
 * @return 处理后的值。
 */
float32_t observer_deadband(float32_t value, float32_t dead);

/**
 * @brief 按 `dst = src` 复制 ObserverState。
 *
 * @param dst 目标状态。
 * @param src 源状态。
 * @return 执行状态。
 */
ObserverStatus observer_copy_state(ObserverState *dst, const ObserverState *src);

/**
 * @brief 清零整体观测状态。
 *
 * @param state 需要清零的状态。
 */
void observer_clear_state(ObserverState *state);

/**
 * @brief 清零 INS 状态。
 *
 * @param state 需要清零的 INS 状态。
 */
void observer_clear_ins(ObserverInsState *state);

/**
 * @brief 清零底盘速度状态。
 *
 * @param state 需要清零的底盘速度状态。
 */
void observer_clear_chassis(ObserverChassisState *state);

/**
 * @brief 清零平衡腿整体状态。
 *
 * @param state 需要清零的平衡腿状态。
 */
void observer_clear_balance(ObserverBalanceState *state);

/**
 * @brief 清零单条平衡腿状态。
 *
 * @param state 需要清零的单腿状态。
 */
void observer_clear_leg(ObserverLegState *state);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_OBSERVER_COMMON_H
