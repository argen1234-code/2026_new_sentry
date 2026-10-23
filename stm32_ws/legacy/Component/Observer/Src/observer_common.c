/**
 ******************************************************************************
 * @file    observer_common.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   观测器内部公共工具实现。
 *
 * @note    本文件只提供数值检查、清零和状态复制等轻量工具。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "observer_common.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
uint8_t observer_finite(float32_t value)
{
    return (isfinite(value) != 0) ? 1U : 0U;
}

uint8_t observer_vec_ok(const Vec3 *value)
{
    if (value == NULL) {
        return 0U;
    }

    return ((observer_finite(value->x) != 0U) &&
            (observer_finite(value->y) != 0U) &&
            (observer_finite(value->z) != 0U)) ? 1U : 0U;
}

float32_t observer_clamp(float32_t value, float32_t min_v, float32_t max_v)
{
    if (value < min_v) {
        return min_v;
    }

    if (value > max_v) {
        return max_v;
    }

    return value;
}

float32_t observer_deadband(float32_t value, float32_t dead)
{
    float32_t limit = (dead >= 0.0F) ? dead : -dead;

    if ((value > -limit) && (value < limit)) {
        return 0.0F;
    }

    return value;
}

ObserverStatus observer_copy_state(ObserverState *dst, const ObserverState *src)
{
    if ((dst == NULL) || (src == NULL)) {
        return ObserverNull;
    }

    *dst = *src;
    return ObserverOk;
}

void observer_clear_state(ObserverState *state)
{
    if (state != NULL) {
        (void)memset(state, 0, sizeof(*state));
    }
}

void observer_clear_ins(ObserverInsState *state)
{
    if (state != NULL) {
        (void)memset(state, 0, sizeof(*state));
    }
}

void observer_clear_chassis(ObserverChassisState *state)
{
    if (state != NULL) {
        (void)memset(state, 0, sizeof(*state));
    }
}

void observer_clear_balance(ObserverBalanceState *state)
{
    if (state != NULL) {
        (void)memset(state, 0, sizeof(*state));
    }
}

void observer_clear_leg(ObserverLegState *state)
{
    if (state != NULL) {
        (void)memset(state, 0, sizeof(*state));
    }
}
