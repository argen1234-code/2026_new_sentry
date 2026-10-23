/**
 ******************************************************************************
 * @file    input_cali_rc.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   RC 到校准命令的映射实现。
 *
 * @note    旧校准入口依赖一组很突兀的摇杆组合；这里原样保留为可配置
 *          手势，因为它可能是为了避免误触而存在的现场妥协。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "input_cali_rc.h"

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */
/** @brief 旧校准手势通道阈值。 */
#define INPUT_CALI_CH_LIMIT ((int16_t)600)

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具只处理参数、限幅、wrap、矩阵工作区等实现细节，不读取任务或设备全局。
 * ========================================================================== */

/**
 * @brief 判断按键上升沿。
 *
 * @param key 当前键位。
 * @param last 上一次键位。
 * @param mask 目标键位。
 * @return 上升沿返回 1，否则返回 0。
 */
static uint8_t cali_key_rise(uint16_t key, uint16_t last, uint16_t mask)
{
    if (mask == 0U) {
        return 0U;
    }

    return (((key & mask) != 0U) && ((last & mask) == 0U)) ? 1U : 0U;
}

/**
 * @brief 判断旧进入校准手势。
 *
 * @param rc DBUS 状态。
 * @return 成立返回 1，否则返回 0。
 */
static uint8_t cali_begin_gesture(const RcDbusState *rc)
{
    if (rc == NULL) {
        return 0U;
    }

    return (uint8_t)((rc->rc.sw[0] == RC_SW_MID) &&
                     (rc->rc.sw[1] == RC_SW_MID) &&
                     (rc->rc.ch[0] < -INPUT_CALI_CH_LIMIT) &&
                     (rc->rc.ch[1] < -INPUT_CALI_CH_LIMIT) &&
                     (rc->rc.ch[2] > INPUT_CALI_CH_LIMIT) &&
                     (rc->rc.ch[3] < -INPUT_CALI_CH_LIMIT));
}

/**
 * @brief 判断旧云台校准动作手势。
 *
 * @param rc DBUS 状态。
 * @return 成立返回 1，否则返回 0。
 */
static uint8_t cali_gimbal_gesture(const RcDbusState *rc)
{
    if (rc == NULL) {
        return 0U;
    }

    return (uint8_t)((rc->rc.sw[0] == RC_SW_MID) &&
                     (rc->rc.sw[1] == RC_SW_MID) &&
                     (rc->rc.ch[0] > INPUT_CALI_CH_LIMIT) &&
                     (rc->rc.ch[1] > INPUT_CALI_CH_LIMIT) &&
                     (rc->rc.ch[2] < -INPUT_CALI_CH_LIMIT) &&
                     (rc->rc.ch[3] > INPUT_CALI_CH_LIMIT));
}

/* ==========================================================================
 * 输入、状态更新与控制步骤
 * --------------------------------------------------------------------------
 * 这一组把注入的输入转换成组件输出；具体调度和硬件发送仍由 App 层负责。
 * ========================================================================== */

/**
 * @brief 按键或手势写入校准输出。
 *
 * @param out 输出参考。
 * @param action 校准动作。
 * @param map 映射配置。
 * @param tick_ms 时间戳。
 */
static void cali_write(InputCaliRef *out,
                       InputCaliAction action,
                       const InputCaliRcMap *map,
                       uint32_t tick_ms)
{
    out->action = action;
    out->online = 1U;
    out->action_valid = 1U;
    out->priority = map->priority;
    out->source = InputSourceRc;
    out->valid_mask = INPUT_CALI_ACTION_VALID;
    out->tick_ms = tick_ms;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
void input_cali_rc_init(InputCaliRcCtx *ctx)
{
    if (ctx == NULL) {
        return;
    }

    (void)memset(ctx, 0, sizeof(*ctx));
}

InputStatus input_cali_rc_map(InputCaliRcCtx *ctx,
                              const RcDbusState *rc,
                              const InputCaliRcMap *map,
                              InputCaliRef *out)
{
    uint8_t begin;
    uint8_t action;

    if ((ctx == NULL) || (map == NULL) || (out == NULL)) {
        return InputNull;
    }

    input_cali_clear(out);
    if ((rc == NULL) || (rc->online == 0U)) {
        ctx->begin_hold = 0U;
        ctx->action_hold = 0U;
        ctx->armed = 0U;
        ctx->last_action = 0U;
        return InputOk;
    }

    if (cali_key_rise(rc->key.v, ctx->last_key, map->gimbal_key) != 0U) {
        cali_write(out, InputCaliGimbal, map, rc->last_update_tick);
    } else if (cali_key_rise(rc->key.v, ctx->last_key, map->imu_key) != 0U) {
        cali_write(out, InputCaliImu, map, rc->last_update_tick);
    }

    if ((map->enable_old_gesture != 0U) && (out->valid_mask == 0U)) {
        begin = cali_begin_gesture(rc);
        if (begin != 0U) {
            if (ctx->begin_hold < map->begin_count) {
                ctx->begin_hold++;
            }
            if (ctx->begin_hold >= map->begin_count) {
                ctx->armed = 1U;
            }
        } else {
            ctx->begin_hold = 0U;
        }

        action = (uint8_t)((ctx->armed != 0U) && (cali_gimbal_gesture(rc) != 0U));
        if (action != 0U) {
            if (ctx->action_hold < map->action_count) {
                ctx->action_hold++;
            }
            if ((ctx->action_hold >= map->action_count) && (ctx->last_action == 0U)) {
                cali_write(out, InputCaliGimbal, map, rc->last_update_tick);
                ctx->last_action = 1U;
                ctx->armed = 0U;
            }
        } else {
            ctx->action_hold = 0U;
            ctx->last_action = 0U;
        }
    }

    ctx->last_key = rc->key.v;
    return InputOk;
}
